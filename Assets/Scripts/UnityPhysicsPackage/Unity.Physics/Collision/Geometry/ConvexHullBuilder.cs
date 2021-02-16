using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using Unity.Collections;
using UnityS.Mathematics;
using UnityEngine.Assertions;
using Unity.Collections.LowLevel.Unsafe;
using static UnityS.Physics.Math;
using Unity.Burst;

namespace UnityS.Physics
{
    /// <summary>
    /// Convex hull builder.
    /// </summary>
    [NoAlias]
    unsafe struct ConvexHullBuilder
    {
        // Mesh representation of the hull
        [NoAlias]
        private ElementPoolBase m_Vertices;
        [NoAlias]
        private ElementPoolBase m_Triangles;

        public unsafe ElementPool<Vertex> Vertices
        {
            get
            {
                fixed (ElementPoolBase* vertices = &m_Vertices)
                {
                    return new ElementPool<Vertex> { ElementPoolBase = vertices };
                }
            }
        }

        public unsafe ElementPool<Triangle> Triangles
        {
            get
            {
                fixed (ElementPoolBase* triangles = &m_Triangles)
                {
                    return new ElementPool<Triangle> { ElementPoolBase = triangles };
                }
            }
        }

        // Number of bits with which vertices are quantized
        public enum IntResolution
        {
            Low, // 16 bit, sufficient for ConvexConvexDistanceQueries
            High // 30 bit, required to build hulls larger than ~1m without nearly parallel faces
        }

        // Array of faces' planes, length = NumFaces, updated when BuildFaceIndices() is called
        public Plane* Planes { get; private set; }

        // -1 for empty, 0 for single point, 1 for line segment, 2 for flat convex polygon, 3 for convex polytope
        public int Dimension { get; private set; }

        // Number of faces, coplanar triangles make a single face, updated when BuildFaceIndices() is called
        public int NumFaces { get; private set; }

        // Sum of vertex counts of all faces.  This is greater than the number of elements in Vertices because
        // each vertex is appears on multiple faces.
        public int NumFaceVertices { get; private set; }

        // Valid only when Dimension == 2, the plane in which the hull lies
        public Plane ProjectionPlane { get; private set; }

        // Valid only after calling UpdateHullMassProperties()
        public MassProperties HullMassProperties { get; private set; }

        private long m_IntNormalDirectionX;
        private long m_IntNormalDirectionY;
        private long m_IntNormalDirectionZ;
        private IntResolution m_IntResolution;
        private IntegerSpace m_IntegerSpace;
        private Aabb m_IntegerSpaceAabb;
        private uint m_NextUid;

        private static sfloat k_PlaneEps = sfloat.FromRaw(0x38d1b717);  // Maximum distance any vertex in a face can be from the plane

        /// <summary>
        /// Convex hull vertex.
        /// </summary>
        [DebuggerDisplay("{Cardinality}:{Position}")]
        public struct Vertex : IPoolElement
        {
            public float3 Position;
            public int3 IntPosition;
            public int Cardinality;
            public uint UserData;
            public int NextFree { get { return (int)UserData; } set { UserData = (uint)value; } }

            public bool IsAllocated => Cardinality != -1;

            public Vertex(float3 position, uint userData)
            {
                Position = position;
                UserData = userData;
                Cardinality = 0;
                IntPosition = new int3(0);
            }

            void IPoolElement.MarkFree(int nextFree)
            {
                Cardinality = -1;
                NextFree = nextFree;
            }
        }

        /// <summary>
        /// Convex hull triangle.
        /// </summary>
        [DebuggerDisplay("#{FaceIndex}[{Vertex0}, {Vertex1}, {Vertex2}]")]
        public struct Triangle : IPoolElement
        {
            public int Vertex0, Vertex1, Vertex2;
            public Edge Link0, Link1, Link2;
            public int FaceIndex;
            public uint Uid { get; private set; }
            public int NextFree { get { return (int)Uid; } set { Uid = (uint)value; } }

            public bool IsAllocated => FaceIndex != -2;

            public Triangle(int vertex0, int vertex1, int vertex2, uint uid)
            {
                FaceIndex = 0;
                Vertex0 = vertex0;
                Vertex1 = vertex1;
                Vertex2 = vertex2;
                Link0 = Edge.Invalid;
                Link1 = Edge.Invalid;
                Link2 = Edge.Invalid;
                Uid = uid;
            }

            public unsafe int GetVertex(int index) { fixed (int* p = &Vertex0) { return p[index]; } }
            public unsafe void SetVertex(int index, int value) { fixed (int* p = &Vertex0) { p[index] = value; } }

            public unsafe Edge GetLink(int index) { fixed (Edge* p = &Link0) { return p[index]; } }
            public unsafe void SetLink(int index, Edge handle) { fixed (Edge* p = &Link0) { p[index] = handle; } }

            void IPoolElement.MarkFree(int nextFree)
            {
                FaceIndex = -2;
                NextFree = nextFree;
            }
        }

        /// <summary>
        /// An edge of a triangle, used internally to traverse hull topology.
        /// </summary>
        [DebuggerDisplay("[{value>>2}:{value&3}]")]
        public struct Edge : IEquatable<Edge>
        {
            public readonly int Value;

            public bool IsValid => Value != Invalid.Value;
            public int TriangleIndex => Value >> 2;
            public int EdgeIndex => Value & 3;

            public static readonly Edge Invalid = new Edge(0x7fffffff);

            public Edge(int value) { Value = value; }
            public Edge(int triangleIndex, int edgeIndex) { Value = triangleIndex << 2 | edgeIndex; }

            public Edge Next => IsValid ? new Edge(TriangleIndex, (EdgeIndex + 1) % 3) : Invalid;
            public Edge Prev => IsValid ? new Edge(TriangleIndex, (EdgeIndex + 2) % 3) : Invalid;

            public bool Equals(Edge other) => Value == other.Value;
        }

        /// <summary>
        /// An edge of a face (possibly made from multiple triangles).
        /// </summary>
        public struct FaceEdge
        {
            public Edge Start;      // the first edge of the face
            public Edge Current;    // the current edge of the face

            public bool IsValid => Current.IsValid;

            public static readonly FaceEdge Invalid = new FaceEdge { Start = Edge.Invalid, Current = Edge.Invalid };

            public static implicit operator Edge(FaceEdge fe) => fe.Current;
        }

        /// <summary>
        /// Convex hull mass properties.
        /// </summary>
        public struct MassProperties
        {
            public float3 CenterOfMass;
            public float3x3 InertiaTensor;
            public sfloat SurfaceArea;
            public sfloat Volume;
        }

        /// <summary>
        /// A quantized integer space.
        /// </summary>
        private struct IntegerSpace
        {
            // int * Scale + Offset = float
            public readonly float3 Offset;
            public readonly sfloat Scale;
            public readonly sfloat InvScale;

            public IntegerSpace(Aabb aabb, int resolution)
            {
                sfloat resolutionf = (sfloat)resolution;
                sfloat extent = math.cmax(aabb.Extents);
                Scale = extent / resolutionf;
                InvScale = math.select(resolutionf / extent, sfloat.Zero, extent <= sfloat.Zero);
                Offset = aabb.Center - (extent / (sfloat)2.0f);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public int3 ToIntegerSpace(float3 x) => new int3((x - Offset) * InvScale + (sfloat)0.5f);

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public float3 ToFloatSpace(int3 x) => x * new float3(Scale) + Offset;
        }

        // Create a hull builder with external storage
        // vertices must be at least large enough to hold verticesCapacity elements, triangles and planes must be large enough to hold 2 * verticesCapacity elements
        // domain is the AABB of all points that will be added to the hull
        // simplificationTolerance is the sum of tolerances that will be passed to SimplifyVertices() and SimplifyFacesAndShrink()
        public unsafe ConvexHullBuilder(int verticesCapacity, Vertex* vertices, Triangle* triangles, Plane* planes,
            Aabb domain, sfloat simplificationTolerance, IntResolution intResolution)
        {
            m_Vertices = new ElementPoolBase(vertices, verticesCapacity);
            m_Triangles = new ElementPoolBase(triangles, 2 * verticesCapacity);
            Planes = planes;
            Dimension = -1;
            NumFaces = 0;
            NumFaceVertices = 0;
            ProjectionPlane = new Plane(new float3(sfloat.Zero), sfloat.Zero);
            HullMassProperties = new MassProperties();
            m_IntNormalDirectionX = 0;
            m_IntNormalDirectionY = 0;
            m_IntNormalDirectionZ = 0;
            m_IntResolution = intResolution;
            m_NextUid = 1;

            // Add some margin for error to the domain.  This loses some quantization resolution and therefore some accuracy, but it's possible that
            // SimplifyVertices and SimplifyFacesAndMerge will not stay perfectly within the requested error limits, and expanding the limits avoids
            // clipping against the domain AABB in AddPoint
            sfloat constantMargin = sfloat.FromRaw(0x3c23d70a);
            sfloat linearMargin = sfloat.FromRaw(0x3dcccccd);
            domain.Expand(math.max(simplificationTolerance * (sfloat)2.0f, constantMargin));
            domain.Expand(math.cmax(domain.Extents) * linearMargin);
            m_IntegerSpaceAabb = domain;

            int quantizationBits = (intResolution == IntResolution.Low ? 16 : 30);
            m_IntegerSpace = new IntegerSpace(domain, (1 << quantizationBits) - 1);
        }

        /// <summary>
        /// Copy the content of another convex hull into this one.
        /// </summary>
        public unsafe ConvexHullBuilder(int verticesCapacity, Vertex* vertices, Triangle* triangles, Plane* planes,
            ConvexHullBuilder other)
        {
            m_Vertices = new ElementPoolBase(vertices, verticesCapacity);
            m_Triangles = new ElementPoolBase(triangles, 2 * verticesCapacity);
            Planes = planes;
            Dimension = other.Dimension;
            NumFaces = other.NumFaces;
            NumFaceVertices = other.NumFaceVertices;
            ProjectionPlane = other.ProjectionPlane;
            HullMassProperties = other.HullMassProperties;
            m_IntNormalDirectionX = other.m_IntNormalDirectionX;
            m_IntNormalDirectionY = other.m_IntNormalDirectionY;
            m_IntNormalDirectionZ = other.m_IntNormalDirectionZ;
            m_IntResolution = other.m_IntResolution;
            m_NextUid = other.m_NextUid;
            m_IntegerSpaceAabb = other.m_IntegerSpaceAabb;
            m_IntegerSpace = other.m_IntegerSpace;

            Vertices.CopyFrom(other.Vertices);
            Triangles.CopyFrom(other.Triangles);
            if (other.NumFaces > 0)
            {
                UnsafeUtility.MemCpy(Planes, other.Planes, other.NumFaces * sizeof(Plane));
            }
        }

        #region Construction

        /// <summary>
        /// Reset the convex hull.
        /// </summary>
        public void Reset()
        {
            Vertices.Clear();
            Triangles.Clear();
            Dimension = -1;
            NumFaces = 0;
            NumFaceVertices = 0;
            ProjectionPlane = new Plane(new float3(sfloat.Zero), sfloat.Zero);
        }

        // 
        public unsafe void Compact()
        {

            // Compact the vertices array
            NativeArray<int> vertexRemap = new NativeArray<int>(Vertices.PeakCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            if (Vertices.Compact((int*)vertexRemap.GetUnsafePtr()))
            {
                // Remap all of the vertices in triangles, then compact the triangles array
                foreach (int t in Triangles.Indices)
                {
                    Triangle tri = Triangles[t];
                    tri.Vertex0 = vertexRemap[tri.Vertex0];
                    tri.Vertex1 = vertexRemap[tri.Vertex1];
                    tri.Vertex2 = vertexRemap[tri.Vertex2];
                    Triangles.Set(t, tri);
                }
            }

            Triangles.Compact(null);
        }

        /// <summary>
        /// Add a point the the convex hull.
        /// </summary>
        /// <param name="point">Point to insert.</param>
        /// <param name="userData">User data attached to the new vertex if insertion succeeds.</param>        
        /// <param name="force2D">If true, the hull will not grow beyond two dimensions.</param>        
        /// <returns>true if the insertion succeeded, false otherwise.</returns>
        public unsafe bool AddPoint(float3 point, uint userData = 0, bool force2D = false)
        {
            // Reset faces.
            NumFaces = 0;
            NumFaceVertices = 0;

            // Return false if there is not enough room to allocate a vertex.
            if (!Vertices.CanAllocate)
            {
                return false;
            }

            // Point should be inside the quantization AABB, if not then clip it
            if (!m_IntegerSpaceAabb.Contains(point))
            {
                point = math.max(math.min(point, m_IntegerSpaceAabb.Max), m_IntegerSpaceAabb.Min);
            }
            int3 intPoint = m_IntegerSpace.ToIntegerSpace(point);

            // Insert vertex.
            switch (Dimension)
            {
                // Empty hull, just add a vertex.
                case -1:
                {
                    AllocateVertex(point, userData);
                    Dimension = 0;
                }
                break;

                // 0 dimensional hull, make a line.
                case 0:
                {
                    sfloat minDistanceFromPoint = sfloat.FromRaw(0x3727c5ac);
                    if (math.lengthsq(Vertices[0].Position - point) <= minDistanceFromPoint * minDistanceFromPoint) return false;
                    AllocateVertex(point, userData);
                    Dimension = 1;
                }
                break;

                // 1 dimensional hull, make a triangle.
                case 1:
                {
                    IntCross(Vertices[0].IntPosition - intPoint, Vertices[1].IntPosition - intPoint, out long normalDirectionX, out long normalDirectionY, out long normalDirectionZ);
                    if (normalDirectionX == 0 && normalDirectionY == 0 && normalDirectionZ == 0)
                    {
                        // Still 1D, keep whichever two vertices are farthest apart
                        float3x3 edgesTransposed = math.transpose(new float3x3(Vertices[1].Position - Vertices[0].Position, point - Vertices[1].Position, Vertices[0].Position - point));
                        float3 edgesLengthSq = edgesTransposed.c0 * edgesTransposed.c0 + edgesTransposed.c1 * edgesTransposed.c1 + edgesTransposed.c2 * edgesTransposed.c2;
                        bool3 isLongestEdge = edgesLengthSq == math.cmax(edgesLengthSq);
                        if (isLongestEdge.y)
                        {
                            Vertex newVertex = Vertices[0];
                            newVertex.Position = point;
                            newVertex.IntPosition = m_IntegerSpace.ToIntegerSpace(point);
                            newVertex.UserData = userData;
                            Vertices.Set(0, newVertex);
                        }
                        else if (isLongestEdge.z)
                        {
                            Vertex newVertex = Vertices[1];
                            newVertex.Position = point;
                            newVertex.IntPosition = m_IntegerSpace.ToIntegerSpace(point);
                            newVertex.UserData = userData;
                            Vertices.Set(1, newVertex);
                        } // else, point is on the edge between Vertices[0] and Vertices[1]
                    }
                    else
                    {
                        // Extend dimension.
                        AllocateVertex(point, userData);
                        Dimension = 2;
                        ProjectionPlane = ComputePlane(0, 1, 2, true);
                        m_IntNormalDirectionX = normalDirectionX;
                        m_IntNormalDirectionY = normalDirectionY;
                        m_IntNormalDirectionZ = normalDirectionZ;
                    }
                }
                break;

                // 2 dimensional hull, make a volume or expand face.
                case 2:
                {
                    long det = 0;
                    if (!force2D)
                    {
                        // Try to expand to a 3D hull
                        for (int i = Vertices.PeakCount - 2, j = Vertices.PeakCount - 1, k = 0; k < Vertices.PeakCount - 2; i = j, j = k, k++)
                        {
                            det = IntDet(i, j, k, intPoint);
                            if (det != 0)
                            {
                                // Extend dimension.
                                ProjectionPlane = new Plane(new float3(sfloat.Zero), sfloat.Zero);

                                // Orient tetrahedron.
                                if (det > 0)
                                {
                                    Vertex t = Vertices[k];
                                    Vertices.Set(k, Vertices[j]);
                                    Vertices.Set(j, t);
                                }

                                // Allocate vertex.
                                int nv = Vertices.PeakCount;
                                int vertexIndex = AllocateVertex(point, userData);

                                // Build tetrahedron.
                                Dimension = 3;
                                Edge nt0 = AllocateTriangle(i, j, k);
                                Edge nt1 = AllocateTriangle(j, i, vertexIndex);
                                Edge nt2 = AllocateTriangle(k, j, vertexIndex);
                                Edge nt3 = AllocateTriangle(i, k, vertexIndex);
                                BindEdges(nt0, nt1); BindEdges(nt0.Next, nt2); BindEdges(nt0.Prev, nt3);
                                BindEdges(nt1.Prev, nt2.Next); BindEdges(nt2.Prev, nt3.Next); BindEdges(nt3.Prev, nt1.Next);

                                // Re-insert other vertices.
                                bool success = true;
                                for (int v = 0; v < nv; v++)
                                {
                                    if (v == i || v == j || v == k)
                                    {
                                        continue;
                                    }
                                    Vertex vertex = Vertices[v];
                                    Vertices.Release(v);
                                    success = success & AddPoint(vertex.Position, vertex.UserData);
                                }
                                return success;
                            }
                        }
                    }
                    if (det == 0)
                    {
                        // Hull is still 2D
                        bool* isOutside = stackalloc bool[Vertices.PeakCount];
                        bool isOutsideAny = false;
                        for (int i = Vertices.PeakCount - 1, j = 0; j < Vertices.PeakCount; i = j++)
                        {
                            // Test if the point is inside the edge
                            // Note, even with 16 bit quantized coordinates, we cannot fit this calculation in 64 bit integers
                            int3 edge = Vertices[j].IntPosition - Vertices[i].IntPosition;
                            int3 delta = intPoint - Vertices[i].IntPosition;
                            IntCross(edge, delta, out long cx, out long cy, out long cz);
                            Int128 dot = Int128.Mul(m_IntNormalDirectionX, cx) + Int128.Mul(m_IntNormalDirectionY, cy) + Int128.Mul(m_IntNormalDirectionZ, cz);
                            isOutside[i] = dot.IsNegative;
                            isOutsideAny |= isOutside[i];
                        }

                        // If the point is outside the hull, insert it and remove any points that it was outside of
                        if (isOutsideAny)
                        {
                            Vertex* newVertices = stackalloc Vertex[Vertices.PeakCount + 1];
                            int numNewVertices = 1;
                            newVertices[0] = new Vertex(point, userData);
                            newVertices[0].IntPosition = intPoint;
                            for (int i = Vertices.PeakCount - 1, j = 0; j < Vertices.PeakCount; i = j++)
                            {
                                if (isOutside[i] && isOutside[i] != isOutside[j])
                                {
                                    newVertices[numNewVertices++] = Vertices[j];
                                    for (; ; )
                                    {
                                        if (isOutside[j]) break;
                                        j = (j + 1) % Vertices.PeakCount;
                                        newVertices[numNewVertices++] = Vertices[j];
                                    }
                                    break;
                                }
                            }

                            Vertices.CopyFrom(newVertices, numNewVertices);
                        }
                    }
                }
                break;

                // 3 dimensional hull, add vertex.
                case 3:
                {
                    int* nextTriangles = stackalloc int[Triangles.PeakCount];
                    for (int i = 0; i < Triangles.PeakCount; i++)
                    {
                        nextTriangles[i] = -1;
                    }

                    Edge* newEdges = stackalloc Edge[Vertices.PeakCount];
                    for (int i = 0; i < Vertices.PeakCount; i++)
                    {
                        newEdges[i] = Edge.Invalid;
                    }

                    // Classify all triangles as either front(faceIndex = 1) or back(faceIndex = -1).
                    int firstFrontTriangleIndex = -1, numFrontTriangles = 0, numBackTriangles = 0;
                    int lastFrontTriangleIndex = -1;
                    float3 floatPoint = m_IntegerSpace.ToFloatSpace(intPoint);
                    sfloat maxDistance = sfloat.Zero;
                    foreach (int triangleIndex in Triangles.Indices)
                    {
                        Triangle triangle = Triangles[triangleIndex];
                        long det = IntDet(triangle.Vertex0, triangle.Vertex1, triangle.Vertex2, intPoint);
                        if (det == 0)
                        {
                            // Check for duplicated vertex.
                            if (math.all(Vertices[triangle.Vertex0].IntPosition == intPoint)) return false;
                            if (math.all(Vertices[triangle.Vertex1].IntPosition == intPoint)) return false;
                            if (math.all(Vertices[triangle.Vertex2].IntPosition == intPoint)) return false;
                        }
                        if (det > 0)
                        {
                            nextTriangles[triangleIndex] = firstFrontTriangleIndex;
                            firstFrontTriangleIndex = triangleIndex;
                            if (lastFrontTriangleIndex == -1)
                            {
                                lastFrontTriangleIndex = triangleIndex;
                            }

                            triangle.FaceIndex = 1;
                            numFrontTriangles++;

                            Plane plane = ComputePlane(triangleIndex, true);
                            sfloat distance = math.dot(plane.Normal, floatPoint) + plane.Distance;
                            maxDistance = math.max(distance, maxDistance);
                        }
                        else
                        {
                            triangle.FaceIndex = -1;
                            numBackTriangles++;
                        }
                        Triangles.Set(triangleIndex, triangle);
                    }

                    // Return false if the vertex is inside the hull
                    if (numFrontTriangles == 0 || numBackTriangles == 0)
                    {
                        return false;
                    }

                    // Link boundary loop.
                    Edge loopEdge = Edge.Invalid;
                    int loopCount = 0;
                    for (int frontTriangle = firstFrontTriangleIndex; frontTriangle != -1; frontTriangle = nextTriangles[frontTriangle])
                    {
                        for (int j = 0; j < 3; ++j)
                        {
                            var edge = new Edge(frontTriangle, j);
                            Edge linkEdge = GetLinkedEdge(edge);
                            if (Triangles[linkEdge.TriangleIndex].FaceIndex == -1)
                            {
                                int vertexIndex = StartVertex(linkEdge);

                                // Vertex already bound.
                                Assert.IsTrue(newEdges[vertexIndex].Equals(Edge.Invalid));

                                // Link.
                                newEdges[vertexIndex] = linkEdge;
                                loopEdge = linkEdge;
                                loopCount++;
                            }
                        }
                    }

                    // Return false if there is not enough room to allocate new triangles.
                    if ((Triangles.PeakCount + loopCount - numFrontTriangles) > Triangles.Capacity)
                    {
                        return false;
                    }

                    // Release front triangles.
                    do
                    {
                        int next = nextTriangles[firstFrontTriangleIndex];
                        ReleaseTriangle(firstFrontTriangleIndex);
                        firstFrontTriangleIndex = next;
                    } while (firstFrontTriangleIndex != -1);

                    // Add vertex.
                    int newVertex = AllocateVertex(point, userData);

                    // Add fan of triangles.
                    {
                        Edge firstFanEdge = Edge.Invalid, lastFanEdge = Edge.Invalid;
                        for (int i = 0; i < loopCount; ++i)
                        {
                            int v0 = StartVertex(loopEdge);
                            int v1 = EndVertex(loopEdge);
                            Edge t = AllocateTriangle(v1, v0, newVertex);
                            BindEdges(loopEdge, t);
                            if (lastFanEdge.IsValid)
                                BindEdges(t.Next, lastFanEdge.Prev);
                            else
                                firstFanEdge = t;

                            lastFanEdge = t;
                            loopEdge = newEdges[v1];
                        }
                        BindEdges(lastFanEdge.Prev, firstFanEdge.Next);
                    }
                }
                break;
            }
            return true;
        }

        // Flatten the hull to 2D
        // This is used to handle edge cases where very thin 3D hulls become 2D or invalid during simplification.
        // Extremely thin 3D hulls inevitably have nearly parallel faces, which cause problems in collision detection,
        // so the best solution is to flatten them completely.
        public unsafe void Rebuild2D()
        {
            Assert.AreEqual(Dimension, 3);

            // Copy the vertices and compute the OLS plane
            Plane plane;
            float3* tempVertices = stackalloc float3[Vertices.PeakCount];
            Aabb aabb = Aabb.Empty;
            int numVertices = 0;
            {
                OLSData data = new OLSData();
                data.Init();
                foreach (int v in Vertices.Indices)
                {
                    float3 position = Vertices[v].Position;
                    tempVertices[numVertices++] = position;
                    data.Include(position, sfloat.One);
                    aabb.Include(position);
                }
                float3 direction = sfloat.One / math.max(sfloat.FromRaw(0x2edbe6ff), aabb.Extents); // Use the min aabb extent as regressand
                data.Solve(direction, direction);
                plane = data.Plane;
            }

            // Rebuild the hull from the projection of the vertices to the plane
            Reset();
            for (int i = 0; i < numVertices; i++)
            {
                const bool force2D = true;
                AddPoint(plane.Projection(tempVertices[i]), 0, force2D);
            }

            BuildFaceIndices();
        }

        // Helper to sort triangles in BuildFaceIndices
        unsafe struct CompareAreaDescending : IComparer<int>
        {
            public NativeArray<sfloat> Areas;
            public CompareAreaDescending(NativeArray<sfloat> areas) { Areas = areas; }
            public int Compare(int x, int y)
            {
                return Areas[y].CompareTo(Areas[x]);
            }
        }

        // Set the face index for each triangle. Triangles lying in the same plane will have the same face index.
        public void BuildFaceIndices(NativeArray<Plane> planes = default)
        {
            sfloat convexEps = sfloat.FromRaw(0x3727c5ac); // Maximum error allowed in face convexity

            NumFaces = 0;
            NumFaceVertices = 0;

            NativeArray<bool> planesUsed = new NativeArray<bool>();
            if (planes.IsCreated)
            {
                planesUsed = new NativeArray<bool>(planes.Length, Allocator.Temp, NativeArrayOptions.ClearMemory);
            }

            switch (Dimension)
            {
                case 2:
                    NumFaces = 2;
                    NumFaceVertices = 2 * Vertices.PeakCount;
                    break;

                case 3:
                {
                    // Make a compact list of triangles and their areas
                    NativeArray<int> triangleIndices = new NativeArray<int>(Triangles.PeakCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
                    NativeArray<sfloat> triangleAreas = new NativeArray<sfloat>(Triangles.PeakCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
                    int numTriangles = 0;
                    foreach (int triangleIndex in Triangles.Indices)
                    {
                        Triangle t = Triangles[triangleIndex];
                        float3 o = Vertices[t.Vertex0].Position;
                        float3 a = Vertices[t.Vertex1].Position - o;
                        float3 b = Vertices[t.Vertex2].Position - o;
                        triangleAreas[triangleIndex] = math.lengthsq(math.cross(a, b));
                        triangleIndices[numTriangles++] = triangleIndex;
                    }

                    // Sort the triangles by descending area. It is best to choose the face plane from the largest triangle
                    // because 1) it minimizes the distance to other triangles and therefore the plane error, and 2) it avoids numerical
                    // problems computing degenerate triangle normals
                    triangleIndices.GetSubArray(0, numTriangles).Sort(new CompareAreaDescending(triangleAreas));

                    // Clear faces
                    NativeArray<Edge> boundaryEdges = new NativeArray<Edge>(Triangles.PeakCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
                    for (int iTriangle = 0; iTriangle < numTriangles; iTriangle++)
                    {
                        int triangleIndex = triangleIndices[iTriangle];
                        Triangle t = Triangles[triangleIndex]; t.FaceIndex = -1; Triangles.Set(triangleIndex, t);
                    }

                    // Merge triangles into faces
                    for (int iTriangle = 0; iTriangle < numTriangles; iTriangle++)
                    {
                        // Check if the triangle is already part of a face
                        int triangleIndex = triangleIndices[iTriangle];
                        if (Triangles[triangleIndex].FaceIndex != -1)
                        {
                            continue;
                        }

                        // Create a new face
                        int newFaceIndex = NumFaces++;
                        Triangle t = Triangles[triangleIndex]; t.FaceIndex = newFaceIndex; Triangles.Set(triangleIndex, t);

                        // Search for the plane that best fits the triangle
                        int bestPlane = -1;
                        if (planes != null)
                        {
                            sfloat bestError = k_PlaneEps;
                            float3 a = Vertices[t.Vertex0].Position;
                            float3 b = Vertices[t.Vertex1].Position;
                            float3 c = Vertices[t.Vertex2].Position;
                            for (int i = 0; i < planes.Length; i++)
                            {
                                if (planesUsed[i]) continue;
                                Plane currentPlane = planes[i];
                                float3 errors = new float3(currentPlane.SignedDistanceToPoint(a), currentPlane.SignedDistanceToPoint(b), currentPlane.SignedDistanceToPoint(c));
                                sfloat error = math.cmax(math.abs(errors));
                                if (error < bestError)
                                {
                                    bestError = error;
                                    bestPlane = i;
                                }
                            }
                        }

                        // If a plane that fits the triangle was found, use it.  Otherwise compute one from the triangle vertices
                        Plane plane;
                        if (bestPlane < 0)
                        {
                            plane = ComputePlane(triangleIndex);
                        }
                        else
                        {
                            planesUsed[bestPlane] = true;
                            plane = planes[bestPlane];
                        }
                        Planes[newFaceIndex] = plane;

                        // Search for neighboring triangles that can be added to the face
                        boundaryEdges[0] = new Edge(triangleIndex, 0);
                        boundaryEdges[1] = new Edge(triangleIndex, 1);
                        boundaryEdges[2] = new Edge(triangleIndex, 2);
                        int numBoundaryEdges = 3;
                        while (true)
                        {
                            int openBoundaryEdgeIndex = -1;
                            sfloat maxArea = -sfloat.One;

                            for (int i = 0; i < numBoundaryEdges; ++i)
                            {
                                Edge edge = boundaryEdges[i];
                                Edge linkedEdge = GetLinkedEdge(edge);

                                int linkedTriangleIndex = linkedEdge.TriangleIndex;

                                if (Triangles[linkedTriangleIndex].FaceIndex != -1) continue;
                                if (triangleAreas[linkedTriangleIndex] <= maxArea) continue;

                                int apex = ApexVertex(linkedEdge);
                                float3 newVertex = Vertices[apex].Position;
                                if (math.abs(plane.SignedDistanceToPoint(newVertex)) > k_PlaneEps)
                                {
                                    continue;
                                }

                                float3 linkedNormal = ComputePlane(linkedTriangleIndex).Normal;
                                if (math.dot(plane.Normal, linkedNormal) < sfloat.Zero)
                                {
                                    continue;
                                }

                                Plane p0 = PlaneFromTwoEdges(newVertex, newVertex - Vertices[StartVertex(edge)].Position, plane.Normal);
                                Plane p1 = PlaneFromTwoEdges(newVertex, Vertices[EndVertex(edge)].Position - newVertex, plane.Normal);

                                var accept = true;
                                for (int j = 1; accept && j < (numBoundaryEdges - 1); ++j)
                                {
                                    float3 x = Vertices[EndVertex(boundaryEdges[(i + j) % numBoundaryEdges])].Position;
                                    sfloat d = math.max(Dotxyz1(p0, x), Dotxyz1(p1, x));
                                    accept &= d < convexEps;
                                }

                                if (accept)
                                {
                                    openBoundaryEdgeIndex = i;
                                    maxArea = triangleAreas[linkedTriangleIndex];
                                }
                            }

                            if (openBoundaryEdgeIndex != -1)
                            {
                                Edge linkedEdge = GetLinkedEdge(boundaryEdges[openBoundaryEdgeIndex]);

                                // Check if merge has made the shape 2D, if so then quit
                                if (numBoundaryEdges >= boundaryEdges.Length)
                                {
                                    NumFaces = 3; // force 2D rebuild
                                    break;
                                }

                                // Insert two edges in place of the open boundary edge
                                for (int i = numBoundaryEdges; i > openBoundaryEdgeIndex; i--)
                                {
                                    boundaryEdges[i] = boundaryEdges[i - 1];
                                }
                                numBoundaryEdges++;
                                boundaryEdges[openBoundaryEdgeIndex] = linkedEdge.Next;
                                boundaryEdges[openBoundaryEdgeIndex + 1] = linkedEdge.Prev;

                                Triangle tri = Triangles[linkedEdge.TriangleIndex];
                                tri.FaceIndex = newFaceIndex;
                                Triangles.Set(linkedEdge.TriangleIndex, tri);
                            }
                            else
                            {
                                break;
                            }
                        }
                        NumFaceVertices += numBoundaryEdges;
                    }

                    // Triangle merging may turn 3D shapes into 2D, check for that case and reduce the dimension
                    if (NumFaces < 4)
                    {
                        Rebuild2D();
                    }
                }
                break;
            }
        }

        private int AllocateVertex(float3 point, uint userData)
        {
            Assert.IsTrue(m_IntegerSpaceAabb.Contains(point));
            var vertex = new Vertex(point, userData) { IntPosition = m_IntegerSpace.ToIntegerSpace(point) };
            return Vertices.Allocate(vertex);
        }

        private Edge AllocateTriangle(int vertex0, int vertex1, int vertex2)
        {
            Triangle triangle = new Triangle(vertex0, vertex1, vertex2, m_NextUid++);
            int triangleIndex = Triangles.Allocate(triangle);

            Vertex v;
            v = Vertices[vertex0]; v.Cardinality++; Vertices.Set(vertex0, v);
            v = Vertices[vertex1]; v.Cardinality++; Vertices.Set(vertex1, v);
            v = Vertices[vertex2]; v.Cardinality++; Vertices.Set(vertex2, v);

            return new Edge(triangleIndex, 0);
        }

        private void ReleaseTriangle(int triangle, bool releaseOrphanVertices = true)
        {
            for (int i = 0; i < 3; ++i)
            {
                int j = Triangles[triangle].GetVertex(i);
                Vertex v = Vertices[j];
                v.Cardinality--;
                Vertices.Set(j, v);
                if (v.Cardinality == 0 && releaseOrphanVertices)
                {
                    Vertices.Release(j);
                }
            }

            Triangles.Release(triangle);
        }

        private void BindEdges(Edge lhs, Edge rhs)
        {
            // Incompatible edges.
            Assert.IsTrue(EndVertex(lhs) == StartVertex(rhs) && StartVertex(lhs) == EndVertex(rhs));

            Triangle lf = Triangles[lhs.TriangleIndex];
            Triangle rf = Triangles[rhs.TriangleIndex];
            lf.SetLink(lhs.EdgeIndex, rhs);
            rf.SetLink(rhs.EdgeIndex, lhs);
            Triangles.Set(lhs.TriangleIndex, lf);
            Triangles.Set(rhs.TriangleIndex, rf);
        }

        #endregion

        #region Simplification

        // Removes vertices that are colinear with two neighbors or coplanar with all neighbors.
        public unsafe void RemoveRedundantVertices()
        {
            sfloat toleranceSq = sfloat.FromRaw(0x2edbe6ff);

            NativeArray<Vertex> newVertices = new NativeArray<Vertex>(Vertices.PeakCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            NativeArray<bool> removed = new NativeArray<bool>(Vertices.PeakCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            while (true)
            {
                bool remove = false;
                if (Dimension != 3) break;

                for (int i = 0; i < Vertices.PeakCount; i++)
                {
                    removed[i] = false;
                }

                int numNewVertices = 0;
                foreach (int v in Vertices.Indices)
                {
                    float3 x = Vertices[v].Position;
                    bool keep = true;
                    bool coplanar = true;
                    bool anyRemoved = false;

                    // For each pair of edges incident to v
                    Edge firstEdge = GetVertexEdge(v);
                    Edge edge0 = firstEdge;
                    do
                    {
                        Triangle triangle0 = Triangles[edge0.TriangleIndex];
                        int index0 = triangle0.GetVertex((edge0.EdgeIndex + 1) % 3);
                        anyRemoved |= removed[index0];
                        if (!removed[index0]) // Ignore already removed vertices
                        {
                            // Double precision is necessary because the calculation involves comparing a difference of squares, which loses a lot of accuracy for long
                            // edges, against a very small fixed tolerance
                            float3 v0 = Vertices[index0].Position;
                            float3 edge0Vec = x - v0;
                            sfloat edge0LengthSq = math.lengthsq(edge0Vec);
                            Edge edge1 = GetLinkedEdge(edge0.Prev);

                            // Test if the triangle normals face the same direction.  This is necessary for nearly flat hulls, where a vertex can be on triangles that
                            // have nearly opposite face normals, so all of the points may be nearly coplanar but the vertex is not safe to remove
                            {
                                Triangle triangle1 = Triangles[edge1.TriangleIndex];
                                float3 v00 = Vertices[triangle0.Vertex0].Position, v01 = Vertices[triangle0.Vertex1].Position, v02 = Vertices[triangle0.Vertex2].Position;
                                float3 v10 = Vertices[triangle1.Vertex0].Position, v11 = Vertices[triangle1.Vertex1].Position, v12 = Vertices[triangle1.Vertex2].Position;
                                coplanar &= (math.dot(math.cross(v01 - v00, v02 - v00), math.cross(v11 - v10, v12 - v10)) >= sfloat.Zero);
                            }

                            while (edge1.Value != firstEdge.Value && keep)
                            {
                                // Test if the distance from x to the line through v1 and v0 is less than tolerance.  If not, then the three vertices
                                // are colinear and x is unnecessary.
                                // The math below is derived from the fact that the distance is the length of the rejection of (x - v0) from (v1 - v0), and
                                // lengthSq(rejection) + lengthSq(x - v0) = lengthSq(projection)
                                int index1 = Triangles[edge1.TriangleIndex].GetVertex((edge1.EdgeIndex + 1) % 3);
                                float3 v1 = Vertices[index1].Position;
                                if (!removed[index1]) // Ignore already removed vertices
                                {
                                    float3 lineVec = v1 - v0;
                                    sfloat lineVecLengthSq = math.lengthsq(lineVec);
                                    sfloat dot = math.dot(edge0Vec, lineVec);
                                    sfloat diffSq = edge0LengthSq * lineVecLengthSq - dot * dot;
                                    sfloat scaledTolSq = toleranceSq * lineVecLengthSq;
                                    keep &= (dot < sfloat.Zero || dot > lineVecLengthSq || diffSq > scaledTolSq);

                                    Edge edge2 = GetLinkedEdge(edge1.Prev);
                                    if (edge2.Value != firstEdge.Value)
                                    {
                                        int index2 = Triangles[edge2.TriangleIndex].GetVertex((edge2.EdgeIndex + 1) % 3);
                                        if (!removed[index2])
                                        {
                                            float3 v2 = Vertices[index2].Position;
                                            float3 n = math.cross(v2 - v0, v1 - v0);
                                            sfloat det = math.dot(n, edge0Vec);
                                            coplanar &= (det * det < math.lengthsq(n) * toleranceSq);
                                        }
                                    }
                                }
                                edge1 = GetLinkedEdge(edge1.Prev);
                            }
                        }
                        edge0 = GetLinkedEdge(edge0.Prev);
                    } while (edge0.Value != firstEdge.Value && keep);
                    keep &= (!coplanar || anyRemoved);

                    removed[v] = !keep;
                    if (keep)
                    {
                        newVertices[numNewVertices++] = Vertices[v];
                    }
                    else
                    {
                        remove = true;
                    }
                }

                if (!remove)
                {
                    break; // nothing to remove
                }

                if (numNewVertices < 4)
                {
                    // This can happen for nearly-flat hulls
                    Rebuild2D();
                    break;
                }

                Reset();
                for (int i = 0; i < numNewVertices; i++)
                {
                    Vertex vertex = newVertices[i];
                    AddPoint(vertex.Position, vertex.UserData);
                }
            }
        }

        // Simplification of two vertices into one new vertex
        struct Collapse
        {
            public int VertexA; // Source vertex, index into Vertices
            public int VertexB; // Source vertex
            public float3 Target; // Position to replace the original vertices with
            public sfloat Cost; // Sum of squared distances from Target to the planes incident to the original vertices
        }

        // Orders Collapses by Cost
        struct CollapseComparer : IComparer<Collapse>
        {
            public int Compare(Collapse x, Collapse y)
            {
                return x.Cost.CompareTo(y.Cost);
            }
        }

        void SetUserData(int v, uint data)
        {
            Vertex vertex = Vertices[v]; vertex.UserData = data; Vertices.Set(v, vertex);
        }

        // Returns a plane containing the edge through vertexIndex0 and vertexIndex1, with normal at equal angles
        // to normal0 and normal1 (those of the triangles that share the edge)
        Plane GetEdgePlane(int vertexIndex0, int vertexIndex1, float3 normal0, float3 normal1)
        {
            float3 vertex0 = Vertices[vertexIndex0].Position;
            float3 vertex1 = Vertices[vertexIndex1].Position;
            float3 edgeVec = vertex1 - vertex0;
            float3 edgeNormal0 = math.normalize(math.cross(edgeVec, normal0));
            float3 edgeNormal1 = math.normalize(math.cross(normal1, edgeVec));
            float3 edgeNormal = math.normalize(edgeNormal0 + edgeNormal1);
            return new Plane(edgeNormal, -math.dot(edgeNormal, vertex0));
        }

        // Returns a matrix M so that (x, y, z, 1) * M * (x, y, z, 1)^T = square of the distance from (x, y, z) to plane
        float4x4 GetPlaneDistSqMatrix(float4 plane)
        {
            return new float4x4(plane * plane.x, plane * plane.y, plane * plane.z, plane * plane.w);
        }

        // Finds the minimum cost Collapse for a, b using previously computed error matrices.  Returns false if preservFaces = true and there is no collapse that would not
        // violate a face, true otherwise.
        // faceIndex: if >=0, index of the one multi-triangle face on the collapsing edge. -1 if there are two multi-tri faces, -2 if there are none.
        Collapse GetCollapse(int a, int b, ref NativeArray<float4x4> matrices)
        {
            float4x4 matrix = matrices[(int)Vertices[a].UserData] + matrices[(int)Vertices[b].UserData];

            // error = x^T * matrix * x, its only extreme point is a minimum and its gradient is linear
            // the value of x that minimizes error is the root of the gradient
            float4 x = float4.zero;
            sfloat cost = sfloat.MaxValue;
            switch (Dimension)
            {
                case 2:
                {
                    // In 2D force vertices to collapse on their original edge (could potentially get lower error by restricting
                    // to the plane, but this is simpler and good enough)
                    float3 u = Vertices[a].Position;
                    float3 v = Vertices[b].Position;
                    float3 edge = v - u;
                    float3x3 solveMatrix = new float3x3(matrix.c0.xyz, matrix.c1.xyz, matrix.c2.xyz);
                    sfloat denom = math.dot(math.mul(solveMatrix, edge), edge);
                    if (denom.IsZero()) // unexpected, just take the midpoint to avoid divide by zero
                    {
                        x = new float4(u + edge * (sfloat)0.5f, sfloat.One);
                        cost = math.dot(math.mul(matrix, x), x);
                    }
                    else
                    {
                        // Find the extreme point on the line through u and v
                        float3 solveOffset = matrix.c3.xyz;
                        sfloat extremum = -math.dot(math.mul(solveMatrix, u) + solveOffset, edge) / denom;
                        x = new float4(u + edge * math.clamp(extremum, sfloat.Zero, sfloat.One), sfloat.One);

                        // Minimum error is at the extremum or one of the two boundaries, test all three and choose the least
                        sfloat uError = Math.Dotxyz1(math.mul(matrix, new float4(u, sfloat.One)), u);
                        sfloat vError = Math.Dotxyz1(math.mul(matrix, new float4(v, sfloat.One)), v);
                        sfloat xError = math.dot(math.mul(matrix, x), x);
                        cost = math.min(math.min(uError, vError), xError);
                        float3 point = math.select(u.xyz, v.xyz, cost == vError);
                        point = math.select(point, x.xyz, cost == xError);
                    }
                    break;
                }

                case 3:
                {
                    // 3D, collapse point does not have to be on the edge between u and v
                    float4x4 solveMatrix = new float4x4(
                        new float4(matrix.c0.xyz, sfloat.Zero),
                        new float4(matrix.c1.xyz, sfloat.Zero),
                        new float4(matrix.c2.xyz, sfloat.Zero),
                        new float4(matrix.c3.xyz, sfloat.One));
                    sfloat det = math.determinant(solveMatrix);
                    if (det < sfloat.FromRaw(0x358637bd)) // determinant should be positive, small values indicate fewer than three planes that are significantly distinct from each other
                    {
                        goto case 2;
                    }

                    x = math.mul(math.inverse(solveMatrix), new float4(sfloat.Zero, sfloat.Zero, sfloat.Zero, sfloat.One));
                    cost = math.dot(math.mul(matrix, x), x);

                    break;
                }
            }

            return new Collapse
            {
                VertexA = a,
                VertexB = b,
                Target = x.xyz,
                Cost = cost
            };
        }

        // Returns the index of pair i,j in an array of all unique unordered pairs of nonnegative ints less than n, sorted (0,0),(0,1),...(0,n-1),(1,1),(1,2),...,(1,n-1),...,(n-1,n-1)
        int Index2d(uint i, uint j, uint n)
        {
            return (int)(i * (n + n - i - 1) / 2 + j);
        }
        
        // Simplifies the hull by collapsing pairs of vertices until the number of vertices is no more than maxVertices and no further pairs can be collapsed without
        // introducing error in excess of maxError.
        // Based on QEM, but with contractions only allowed for vertices connected by a triangle edge, and only to be replaced by vertices on the same edge
        // Note, calling this function destroys vertex user data
        public unsafe void SimplifyVertices(sfloat maxError, int maxVertices)
        {
            // Simplification is only possible in 2D / 3D
            if (Dimension < 2)
            {
                return;
            }

            // Must build faces before calling
            if (NumFaces == 0)
            {
                Assert.IsTrue(false);
                return;
            }

            // Calculate initial error matrices
            NativeArray<Collapse> collapses = new NativeArray<Collapse>();
            NativeArray<float4x4> matrices = new NativeArray<float4x4>(Vertices.PeakCount, Allocator.Temp, NativeArrayOptions.ClearMemory);
            int numVertices = 0;
            if (Dimension == 3)
            {
                // Get an edge from each face
                NativeArray<Edge> firstEdges = new NativeArray<Edge>(NumFaces, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
                for (FaceEdge faceEdge = GetFirstFace(); faceEdge.IsValid; faceEdge = GetNextFace(faceEdge))
                {
                    int triangleIndex = faceEdge.Current.TriangleIndex;
                    int faceIndex = Triangles[triangleIndex].FaceIndex;
                    firstEdges[faceIndex] = faceEdge.Start;
                }

                // Build error matrices
                for (int i = 0; i < NumFaces; i++)
                {
                    // Calculate the error matrix for this face
                    float4 plane = Planes[i];
                    float4x4 faceMatrix = GetPlaneDistSqMatrix(plane);

                    // Add it to the error matrix of each vertex on the face
                    for (FaceEdge edge = new FaceEdge { Start = firstEdges[i], Current = firstEdges[i] }; edge.IsValid; edge = GetNextFaceEdge(edge)) // For each edge of the current face
                    {
                        // Add the error matrix
                        int vertex0 = Triangles[edge.Current.TriangleIndex].GetVertex(edge.Current.EdgeIndex);
                        matrices[vertex0] += faceMatrix;

                        // Check if the edge is acute
                        Edge opposite = GetLinkedEdge(edge);
                        Triangle oppositeTriangle = Triangles[opposite.TriangleIndex];
                        int vertex1 = oppositeTriangle.GetVertex(opposite.EdgeIndex);
                        if (vertex0 < vertex1) // Count each edge only once
                        {
                            float3 oppositeNormal = Planes[oppositeTriangle.FaceIndex].Normal;
                            if (math.dot(plane.xyz, oppositeNormal) < sfloat.FromRaw(0xbc8ef77f)) // 91 degrees -- right angles are common in input data, avoid creating edge planes that distort the original faces
                            {
                                // Add an edge plane to the cost metric for each vertex on the edge to preserve sharp features
                                float4 edgePlane = GetEdgePlane(vertex0, vertex1, plane.xyz, oppositeNormal);
                                float4x4 edgeMatrix = GetPlaneDistSqMatrix(edgePlane);
                                matrices[vertex0] += edgeMatrix;
                                matrices[vertex1] += edgeMatrix;
                            }
                        }
                    }
                }

                // Allocate space for QEM
                collapses = new NativeArray<Collapse>(Triangles.PeakCount * 3 / 2, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            }
            else
            {
                // In 2D, the vertices are sorted and each one has two edge planes
                for (int i = Vertices.PeakCount - 1, j = 0; j < Vertices.PeakCount; i = j, j++)
                {
                    float4 edgePlane = PlaneFromTwoEdges(Vertices[i].Position, Vertices[j].Position - Vertices[i].Position, ProjectionPlane.Normal);
                    float4x4 edgeMatrix = GetPlaneDistSqMatrix(edgePlane);
                    matrices[i] += edgeMatrix;
                    matrices[j] += edgeMatrix;
                }

                numVertices = Vertices.PeakCount;
                collapses = new NativeArray<Collapse>(Vertices.PeakCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            }

            // Write the original index of each vertex to its user data so that we can maintain its error matrix across rebuilds
            foreach (int v in Vertices.Indices)
            {
                SetUserData(v, (uint)v);
                numVertices++;
            }

            // Repeatedly simplify the hull until the count is less than maxVertices and there are no further changes that satisfy maxError
            NativeArray<Vertex> newVertices = new NativeArray<Vertex>(numVertices, Allocator.Temp, NativeArrayOptions.UninitializedMemory); // Note, only Position and UserData are used
            bool enforceMaxVertices = false;
            sfloat maxCost = maxError * maxError;
            while (true)
            {
                // Build a list of potential edge collapses
                int numCollapses = 0;
                bool force2D = false;
                if (Dimension == 3)
                {
                    // Build collapses for each edge
                    foreach (int t in Triangles.Indices)
                    {
                        Triangle triangle = Triangles[t];
                        for (int i = 0; i < 3; i++)
                        {
                            int opposite = triangle.GetLink(i).TriangleIndex;
                            if (t < opposite) // Count each edge only once
                            {
                                // Calculate the error matrix for the two vertex edges combined
                                int a = triangle.GetVertex(i);
                                int b = triangle.GetVertex((i + 1) % 3);

                                Collapse collapse = GetCollapse(a, b, ref matrices);
                                if (collapse.Cost <= maxCost || enforceMaxVertices)
                                {
                                    collapses[numCollapses++] = collapse;
                                }
                            }
                        }
                    }
                }
                else
                {
                    force2D = true; // if hull is ever 2D, it must remain 2D through simplification
                    for (int i = Vertices.PeakCount - 1, j = 0; j < Vertices.PeakCount; i = j, j++)
                    {
                        Collapse collapse = GetCollapse(i, j, ref matrices);
                        if (collapse.Cost <= maxCost || enforceMaxVertices)
                        {
                            collapses[numCollapses++] = collapse;
                        }
                    }
                }

                // Collapse vertices in order of increasing cost
                collapses.GetSubArray(0, numCollapses).Sort(new CollapseComparer());
                int numNewVertices = 0;
                for (int i = 0; i < numCollapses; i++)
                {
                    // If this pass is just to enforce the vertex count, then stop collapsing as soon as it's satisfied
                    Collapse collapse = collapses[i];
                    if (enforceMaxVertices && numVertices <= maxVertices)
                    {
                        break;
                    }

                    // If one of the vertices has been collapsed already, it can't collapse again until the next pass
                    uint matrixA = Vertices[collapse.VertexA].UserData;
                    uint matrixB = Vertices[collapse.VertexB].UserData;
                    if (matrixA == uint.MaxValue || matrixB == uint.MaxValue)
                    {
                        continue;
                    }

                    // Mark the vertices removed
                    SetUserData(collapse.VertexA, uint.MaxValue);
                    SetUserData(collapse.VertexB, uint.MaxValue);
                    numVertices--;

                    // Combine error matrices for use in the next pass
                    float4x4 combined = matrices[(int)matrixA] + matrices[(int)matrixB];
                    matrices[(int)matrixA] = combined;

                    newVertices[numNewVertices++] = new Vertex
                    {
                        Position = collapse.Target,
                        UserData = matrixA
                    };
                }

                // If nothing was collapsed, we're done
                if (numNewVertices == 0)
                {
                    if (numVertices > maxVertices)
                    {
                        // Now it's necessary to exceed maxError in order to get under the vertex limit
                        Assert.IsFalse(enforceMaxVertices);
                        enforceMaxVertices = true;
                        continue;
                    }
                    break;
                }

                // Add all of the original vertices that weren't removed to the list
                foreach (int v in Vertices.Indices)
                {
                    if (Vertices[v].UserData != uint.MaxValue)
                    {
                        newVertices[numNewVertices++] = Vertices[v];
                    }
                }

                // Rebuild
                Reset();
                for (int i = 0; i < numNewVertices; ++i)
                {
                    AddPoint(newVertices[i].Position, newVertices[i].UserData, force2D);
                }
                RemoveRedundantVertices();

                // If this was a max vertices pass and we are now under the limit, we're done
                if (enforceMaxVertices && numVertices <= maxVertices)
                {
                    break;
                }

                // Further simplification is only possible in 2D / 3D
                if (Dimension < 2)
                {
                    break;
                }

                // Count the vertices
                numVertices = 0;
                foreach (int v in Vertices.Indices)
                {
                    numVertices++;
                }
            }
        }

        // Simplification of two planes into a single new plane
        struct FaceMerge
        {
            public int Face0; // Face index
            public int Face1; // Face index
            public Plane Plane; // Plane to replace the two faces
            public sfloat Cost; // Sum of squared distances from original vertices to Plane
            public bool SmallAngle; // True if the angle between the source faces is below the minimum angle threshold
        }

        // Data required to calculate the OLS of a set of points, without needing to store the points themselves.
        // OLSData for the union of point sets can be computed from those sets' OLSDatas without needing the original points.
        struct OLSData
        {
            // Inputs
            private sfloat m_Weight; // Cost multiplier
            private int m_Count; // Number of points in the set
            private float3 m_Sums; // Sum of x, y, z
            private float3 m_SquareSums; // Sum of x^2, y^2, z^2
            private float3 m_ProductSums; // Sum of xy, yz, zx

            // Outputs, assigned when Solve() is called
            public Plane Plane; // OLS plane of the points in the set
            public sfloat Cost; // m_Weight * sum of squared distances from points in the set to Plane

            // Empty the set
            public void Init()
            {
                m_Weight = sfloat.Zero;
                m_Count = 0;
                m_Sums = float3.zero;
                m_SquareSums = float3.zero;
                m_ProductSums = float3.zero;
            }

            // Add a single point to the set
            public void Include(float3 v, sfloat weight)
            {
                m_Weight = math.max(m_Weight, weight);
                m_Count++;
                m_Sums += v;
                m_SquareSums += v * v;
                m_ProductSums += v * v.yzx;
            }

            // Add all points from the
            public void Include(OLSData source, sfloat weight)
            {
                m_Weight = math.max(math.max(m_Weight, source.m_Weight), weight);
                m_Count += source.m_Count;
                m_Sums += source.m_Sums;
                m_SquareSums += source.m_SquareSums;
                m_ProductSums += source.m_ProductSums;
            }

            // Calculate OLS of all included points and store the results in Plane and Cost.
            // Returned plane has normal dot direction >= 0.
            public void Solve(float3 normal0, float3 normal1)
            {
                float3 averageDirection = normal0 + normal1;
                float3 absAverageDirection = math.abs(averageDirection);
                bool3 maxAxis = math.cmax(absAverageDirection) == absAverageDirection;

                // Solve using the axis closest to the average normal for the regressand
                bool planeOk;
                if (maxAxis.x)
                {
                    planeOk = Solve(m_Count, m_Sums.yzx, m_SquareSums.yzx, m_ProductSums.yzx, out Plane plane);
                    Plane = new Plane(plane.Normal.zxy, plane.Distance);
                }
                else if (maxAxis.y)
                {
                    planeOk = Solve(m_Count, m_Sums.zxy, m_SquareSums.zxy, m_ProductSums.zxy, out Plane plane);
                    Plane = new Plane(plane.Normal.yzx, plane.Distance);
                }
                else
                {
                    planeOk = Solve(m_Count, m_Sums, m_SquareSums, m_ProductSums, out Plane);
                }

                // Calculate the error
                if (!planeOk)
                {
                    Cost = sfloat.MaxValue;
                }
                else
                {
                    float4x4 errorMatrix = new float4x4(
                        m_SquareSums.x, m_ProductSums.x, m_ProductSums.z, m_Sums.x,
                        m_ProductSums.x, m_SquareSums.y, m_ProductSums.y, m_Sums.y,
                        m_ProductSums.z, m_ProductSums.y, m_SquareSums.z, m_Sums.z,
                        m_Sums.x, m_Sums.y, m_Sums.z, (sfloat)m_Count);
                    Cost = math.dot(math.mul(errorMatrix, Plane), Plane) * m_Weight;
                }

                // Flip the plane if it's pointing the wrong way
                if (math.dot(Plane.Normal, averageDirection) < sfloat.Zero)
                {
                    Plane = Plane.Flipped;
                }
            }

            // Solve implementation, uses regressor xy regressand z
            // Returns false if the problem is singular
            private static bool Solve(int count, float3 sums, float3 squareSums, float3 productSums, out Plane plane)
            {
                // Calculate the plane with minimum sum of squares of distances to points in the set
                float3x3 gram = new float3x3(
                    (sfloat)count, sums.x, sums.y,
                    sums.x, squareSums.x, productSums.x,
                    sums.y, productSums.x, squareSums.y);
                if (math.determinant(gram).IsZero()) // check for singular gram matrix (unexpected, points should be from nondegenerate tris and so span at least 2 dimensions)
                {
                    plane = new Plane(new float3(sfloat.One, sfloat.Zero, sfloat.Zero), sfloat.Zero);
                    return false;
                }
                float3x3 gramInv = math.inverse(gram);
                float3 momentSum = new float3(sums.z, productSums.zy);
                float3 coeff = (float3)math.mul(gramInv, momentSum);
                float3 normal = new float3(coeff.yz, -sfloat.One);
                sfloat invLength = math.rsqrt(math.lengthsq(normal));
                plane = new Plane(normal * invLength, coeff.x * invLength);
                return true;
            }
        }

        // Helper for calculating edge weights, returns the squared sin of the angle between normal0 and normal1 if the angle is > 90 degrees, otherwise returns 1.
        sfloat SinAngleSq(float3 normal0, float3 normal1)
        {
            sfloat cosAngle = math.dot(normal0, normal1);
            return math.select(sfloat.One, sfloat.One - cosAngle * cosAngle, cosAngle < sfloat.Zero);
        }

        // 1) Simplifies the hull by combining pairs of neighboring faces until the estimated face count is below maxFaces, there are no faces left
        // with an angle below minAngleBetweenFaces, and no combinations that can be made without increasing the error above simplificationTolerance.
        // 2) Shrinks the hull by pushing its planes in as much as possible without moving a vertex further than shrinkDistance or zeroing the volume.
        // 3) Reduces the vertex count below a fixed maximum, this is necessary in case face simplification increased the count above the limit
        // Returns - the distance that the faces were moved in by shrinking
        // Merging and shrinking are combined into a single operation because both work on the planes of the hull and require vertices to be rebuilt
        // afterwards.  Rebuilding vertices is the slowest part of hull generation, so best to do it only once.
        public unsafe sfloat SimplifyFacesAndShrink(sfloat simplificationTolerance, sfloat minAngleBetweenFaces, sfloat shrinkDistance, int maxFaces, int maxVertices)
        {
            // Return if merging is not allowed and shrinking is off
            if (simplificationTolerance <= sfloat.Zero && minAngleBetweenFaces <= sfloat.Zero && shrinkDistance <= sfloat.Zero)
            {
                return sfloat.Zero;
            }

            // Only 3D shapes can shrink
            if (Dimension < 3)
            {
                return sfloat.Zero;
            }
            
            sfloat cosMinAngleBetweenFaces = math.cos(minAngleBetweenFaces);
            sfloat simplificationToleranceSq = simplificationTolerance * simplificationTolerance;
            sfloat k_cosMaxMergeAngle = sfloat.FromRaw(0x3f3504f7); // Don't merge planes at >45 degrees

            // Make a copy of the planes that we can edit
            int numPlanes = NumFaces;
            int maxNumPlanes = numPlanes + Triangles.PeakCount;
            NativeArray<Plane> planes = new NativeArray<Plane>(maxNumPlanes, Allocator.Temp, NativeArrayOptions.UninitializedMemory); // +Triangles.PeakCount to make room for edge planes
            for (int i = 0; i < numPlanes; i++)
            {
                planes[i] = Planes[i];
            }

            // Find the boundary edges of each face
            NativeArray<int> firstFaceEdgeIndex = new NativeArray<int>(numPlanes, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            NativeArray<int> numFaceEdges = new NativeArray<int>(numPlanes, Allocator.Temp, NativeArrayOptions.ClearMemory);
            NativeArray<Edge> faceEdges = new NativeArray<Edge>(Triangles.PeakCount * 3, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            int totalNumEdges = 0;
            foreach (int t in Triangles.Indices)
            {
                // Search each triangle to find one on each face
                Triangle triangle = Triangles[t];
                int faceIndex = triangle.FaceIndex;
                if (numFaceEdges[faceIndex] > 0)
                {
                    continue;
                }

                // Find a boundary edge on the triangle
                firstFaceEdgeIndex[faceIndex] = totalNumEdges;
                for (int i = 0; i < 3; i++)
                {
                    int linkedTriangle = triangle.GetLink(i).TriangleIndex;
                    if (Triangles[linkedTriangle].FaceIndex != faceIndex)
                    {
                        // Save all edges of the face
                        Edge edge = new Edge(t, i);
                        for (FaceEdge faceEdge = new FaceEdge { Start = edge, Current = edge }; faceEdge.IsValid; faceEdge = GetNextFaceEdge(faceEdge))
                        {
                            faceEdges[totalNumEdges++] = faceEdge.Current;
                        }
                        numFaceEdges[faceIndex] = totalNumEdges - firstFaceEdgeIndex[faceIndex];
                        break;
                    }
                }
            }

            // Build OLS data for each face, and calculate the minimum span of the hull among its plane normal directions
            NativeArray<OLSData> olsData = new NativeArray<OLSData>(maxNumPlanes, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            sfloat minSpan = sfloat.MaxValue;
            for (int i = 0; i < numPlanes; i++)
            {
                Plane plane = planes[i];
                OLSData ols = new OLSData(); ols.Init();

                sfloat lastSinAngleSq;
                {
                    Edge lastEdge = faceEdges[firstFaceEdgeIndex[i] + numFaceEdges[i] - 1];
                    Plane lastPlane = planes[Triangles[GetLinkedEdge(lastEdge).TriangleIndex].FaceIndex];
                    lastSinAngleSq = SinAngleSq(plane.Normal, lastPlane.Normal);
                }

                for (int j = 0; j < numFaceEdges[i]; j++)
                {
                    // Use the minimum angle of the two edges incident to the vertex to weight it
                    Edge nextEdge = faceEdges[firstFaceEdgeIndex[i] + j];
                    Plane nextPlane = planes[Triangles[GetLinkedEdge(nextEdge).TriangleIndex].FaceIndex];
                    sfloat nextSinAngleSq = SinAngleSq(plane.Normal, nextPlane.Normal);
                    sfloat weight = sfloat.One / math.max(sfloat.Epsilon, math.min(lastSinAngleSq, nextSinAngleSq));
                    lastSinAngleSq = nextSinAngleSq;

                    // Include the weighted vertex in OLS data
                    float3 vertex = Vertices[Triangles[nextEdge.TriangleIndex].GetVertex(nextEdge.EdgeIndex)].Position;
                    ols.Include(vertex, weight);
                }

                olsData[i] = ols;

                // Calculate the span in the plane normal direction
                sfloat span = sfloat.Zero;
                foreach (Vertex vertex in Vertices.Elements)
                {
                    span = math.max(span, -plane.SignedDistanceToPoint(vertex.Position));
                }
                minSpan = math.min(span, minSpan);
            }

            // If the minimum span is below the simplification tolerance then we can build a 2D hull without exceeding the tolerance.
            // This often gives a more accurate result, since nearly-flat hulls will get rebuilt from edge plane collisions.
            // Reserve it for extreme cases where the error from flattening is far less than the edge plane error.
            if (minSpan < simplificationTolerance * sfloat.FromRaw(0x3dcccccd))
            {
                Rebuild2D();
                return sfloat.Zero;
            }

            // Build a list of potential merges and calculate their costs
            // Also add edge planes at any sharp edges, because small changes in angle at those edges can introduce significant error. (Consider for example
            // a thin wedge, if one of the planes at the sharp end rotates so that the edge angle decreases further then the intersection of those planes
            // could move a long distance).
            // Note -- no merges are built for edge planes, which means that they could introduce faces with an angle below minAngleBetweenFaces.
            // This should be rare and edge faces should be extremely thin, which makes it very unlikely for a body to come to rest on one and jitter.
            NativeArray<FaceMerge> merges = new NativeArray<FaceMerge>(numPlanes * (numPlanes - 1), Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            NativeArray<Edge> edgePlanes = new NativeArray<Edge>(Triangles.PeakCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            int numMerges = 0;
            int numEdgePlanes = 0;
            for (int i = 0; i < numPlanes; i++)
            {
                for (int j = 0; j < numFaceEdges[i]; j++)
                {
                    Edge edge = faceEdges[firstFaceEdgeIndex[i] + j];

                    // Get the neighboring face
                    Edge neighborEdge = GetLinkedEdge(edge);
                    Triangle neighborTriangle = Triangles[neighborEdge.TriangleIndex];
                    int neighborFaceIndex = neighborTriangle.FaceIndex;
                    if (neighborFaceIndex < i)
                    {
                        continue; // One merge entry per pair
                    }

                    // Check for sharp angles
                    sfloat k_cosSharpAngle = sfloat.FromRaw(0xbf5db3d0); // 150deg
                    sfloat dot = math.dot(planes[i].Normal, planes[neighborFaceIndex].Normal);
                    if (dot < k_cosMaxMergeAngle)
                    {
                        if (dot < k_cosSharpAngle)
                        {
                            int edgeIndex = numEdgePlanes++;
                            edgePlanes[edgeIndex] = edge;

                            // Create an edge plane
                            float3 normal0 = planes[i].Normal;
                            float3 normal1 = planes[neighborFaceIndex].Normal;
                            int vertexIndex0 = Triangles[edge.TriangleIndex].GetVertex(edge.EdgeIndex);
                            int vertexIndex1 = neighborTriangle.GetVertex(neighborEdge.EdgeIndex);
                            int edgePlaneIndex = numPlanes + edgeIndex;
                            Plane edgePlane = GetEdgePlane(vertexIndex0, vertexIndex1, normal0, normal1);
                            edgePlane.Distance -= simplificationTolerance / (sfloat)2.0f; // push the edge plane out slightly so it only becomes active if the face planes change significiantly
                            planes[edgePlaneIndex] = edgePlane;

                            // Build its OLS data
                            OLSData ols = new OLSData(); ols.Init();
                            ols.Include(Vertices[vertexIndex0].Position, sfloat.One);
                            ols.Include(Vertices[vertexIndex1].Position, sfloat.One);
                            olsData[edgePlaneIndex] = ols;
                        }

                        // Don't merge faces with >90 degree angle
                        continue;
                    }

                    // Calculate the cost to merge the faces
                    OLSData combined = olsData[i];
                    combined.Include(olsData[neighborFaceIndex], sfloat.Zero);
                    combined.Solve(planes[i].Normal, planes[neighborFaceIndex].Normal);
                    bool smallAngle = (dot > cosMinAngleBetweenFaces);
                    if (combined.Cost <= simplificationToleranceSq || smallAngle)
                    {
                        merges[numMerges++] = new FaceMerge
                        {
                            Face0 = i,
                            Face1 = neighborFaceIndex,
                            Cost = combined.Cost,
                            Plane = combined.Plane,
                            SmallAngle = smallAngle
                        };
                    }
                }
            }

            // Calculate the plane offset for shrinking
            // shrinkDistance is the maximum distance that we may move a vertex.  Find the largest plane offset that respects that limit.
            sfloat offset = shrinkDistance;
            {
                // Find an edge incident to each vertex (doesn't matter which one)
                Edge* vertexEdges = stackalloc Edge[Vertices.PeakCount];
                foreach (int triangleIndex in Triangles.Indices)
                {
                    Triangle triangle = Triangles[triangleIndex];
                    vertexEdges[triangle.Vertex0] = new Edge(triangleIndex, 0);
                    vertexEdges[triangle.Vertex1] = new Edge(triangleIndex, 1);
                    vertexEdges[triangle.Vertex2] = new Edge(triangleIndex, 2);
                }

                // Calculates the square of the distance that each vertex moves if all of its incident planes' are moved unit distance along their normals
                sfloat maxShiftSq = sfloat.One;
                NativeArray<int> planeIndices = new NativeArray<int>(numPlanes, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
                foreach (int iVertex in Vertices.Indices)
                {
                    Edge vertexEdge = vertexEdges[iVertex];

                    // Build a list of planes of faces incident to the vertex
                    int numPlaneIndices = 0;
                    Edge edge = vertexEdge;
                    int lastFaceIndex = -1;
                    do
                    {
                        int faceIndex = Triangles[edge.TriangleIndex].FaceIndex;
                        if (faceIndex != lastFaceIndex) // there could be multiple incident triangles on the same face, only need to add it once
                        {
                            planeIndices[numPlaneIndices++] = faceIndex;
                            lastFaceIndex = faceIndex;
                        }
                        edge = GetLinkedEdge(edge).Next;
                    } while (edge.Value != vertexEdge.Value);
                    while (planeIndices[numPlaneIndices - 1] == planeIndices[0])
                    {
                        numPlaneIndices--; // first and last edge could be on different triangles on the same face
                    }

                    // Iterate over all triplets of planes
                    sfloat k_cosWideAngle = sfloat.FromRaw(0x3f5db3d0); // Only limit movement of vertices at corners sharper than 30 degrees
                    for (int i = 0; i < numPlaneIndices - 2; i++)
                    {
                        float3 iNormal = planes[planeIndices[i]].Normal;
                        for (int j = i + 1; j < numPlaneIndices - 1; j++)
                        {
                            float3 jNormal = planes[planeIndices[j]].Normal;
                            float3 ijCross = math.cross(iNormal, jNormal);

                            for (int k = j + 1; k < numPlaneIndices; k++)
                            {
                                float3 kNormal = planes[planeIndices[k]].Normal;

                                // Skip corners with wide angles
                                float3 dots = new float3(
                                    math.dot(iNormal, jNormal),
                                    math.dot(jNormal, kNormal),
                                    math.dot(kNormal, iNormal));
                                if (math.any(dots < k_cosWideAngle))
                                {
                                    // Calculate the movement of the planes' intersection with respect to the planes' shift
                                    sfloat det = math.dot(ijCross, kNormal);
                                    sfloat invDet = math.rcp(det);
                                    float3 jkCross = math.cross(jNormal, kNormal);
                                    float3 kiCross = math.cross(kNormal, iNormal);
                                    sfloat shiftSq = math.lengthsq(ijCross + jkCross + kiCross) * invDet * invDet;
                                    shiftSq = math.select(shiftSq, sfloat.FromRaw(0x501502f9), invDet.IsZero()); // avoid nan/inf in unexpected case of zero or extremely small det
                                    Assert.IsTrue(shiftSq >= sfloat.One);
                                    maxShiftSq = math.max(maxShiftSq, shiftSq);
                                }
                            }
                        }
                    }
                }

                // Calculate how far we can move the planes without moving vertices more than the limit
                offset *= math.rsqrt(maxShiftSq);

                // Can't shrink more than the inner sphere radius, minSpan / 4 is a lower bound on that radius so use it to clamp the offset
                offset = math.min(offset, minSpan / (sfloat)4.0f);
            }

            // Merge faces
            int numMerged = 0;
            int numOriginalPlanes = numPlanes;
            numPlanes += numEdgePlanes;
            NativeArray<bool> removed = new NativeArray<bool>(numPlanes, Allocator.Temp, NativeArrayOptions.ClearMemory);
            while (true)
            {
                while (numMerges > 0 && numPlanes > 4)
                {
                    // Find the cheapest merge
                    int mergeIndex = 0;
                    int smallAngleMergeIndex = -1;
                    sfloat smallAngleMergeCost = sfloat.MaxValue;
                    for (int i = 0; i < numMerges; i++)
                    {
                        if (merges[i].Cost < merges[mergeIndex].Cost)
                        {
                            mergeIndex = i;
                        }

                        if (merges[i].SmallAngle && merges[i].Cost < smallAngleMergeCost)
                        {
                            smallAngleMergeIndex = i;
                            smallAngleMergeCost = merges[i].Cost;
                        }
                    }

                    // If the cheapest merge is above the cost threshold, take the cheapest merge between a pair of planes that are below the angle
                    // threshold and therefore must be merged regardless of cost.  If there are none, then quit if the estimated face count is below
                    // the limit, otherwise stick with the cheapest merge
                    if (merges[mergeIndex].Cost > simplificationToleranceSq)
                    {
                        if (smallAngleMergeIndex < 0)
                        {
                            // We can't know the exact face count, because until we build the shape we don't know which planes will have intersections
                            // on the hull.  Eg. edge planes may or may not be used, or planes may be removed due to shrinking.  Make a rough guess.
                            int estimatedFaceCount = numPlanes - numEdgePlanes - numMerged;
                            if (estimatedFaceCount <= maxFaces)
                            {
                                break;
                            }
                        }
                        else
                        {
                            mergeIndex = smallAngleMergeIndex;
                        }
                    }

                    // Remove the selected merge from the list
                    FaceMerge merge = merges[mergeIndex];
                    merges[mergeIndex] = merges[--numMerges];
                    numMerged++;

                    // Replace plane 0 with the merged plane, and remove plane 1
                    planes[merge.Face0] = merge.Plane;
                    removed[merge.Face1] = true;

                    // Combine plane 1's OLS data into plane 0's
                    {
                        OLSData combined = olsData[merge.Face0];
                        combined.Include(olsData[merge.Face1], sfloat.Zero);
                        olsData[merge.Face0] = combined;
                    }

                    // Update any other potential merges involving either of the original planes to point to the new merged planes
                    for (int i = numMerges - 1; i >= 0; i--)
                    {
                        // Test if the merge includes one of the planes that was just updated
                        // If it references the plane that was removed, update it to point to the new combined plane
                        FaceMerge updateMerge = merges[i];
                        if (updateMerge.Face0 == merge.Face1)
                        {
                            updateMerge.Face0 = merge.Face0;
                        }
                        else if (updateMerge.Face1 == merge.Face1)
                        {
                            updateMerge.Face1 = merge.Face0;
                        }
                        else if (updateMerge.Face0 != merge.Face0 && updateMerge.Face1 != merge.Face0)
                        {
                            continue;
                        }

                        // Can't merge a plane with itself, this happens if there is eg. a trifan that gets merged together
                        if (updateMerge.Face0 == updateMerge.Face1) 
                        {
                            merges[i] = merges[--numMerges];
                            continue;
                        }

                        // Limit the maximum merge angle
                        sfloat dot = math.dot(planes[updateMerge.Face0].Normal, planes[updateMerge.Face1].Normal);
                        if (dot < k_cosMaxMergeAngle)
                        {
                            merges[i] = merges[--numMerges];
                            continue;
                        }

                        // Calculate the new plane and cost
                        sfloat weight = sfloat.One / math.max(sfloat.Epsilon, SinAngleSq(planes[updateMerge.Face0].Normal, planes[updateMerge.Face1].Normal));
                        OLSData combined = olsData[updateMerge.Face0];
                        combined.Include(olsData[updateMerge.Face1], weight);
                        combined.Solve(planes[updateMerge.Face0].Normal, planes[updateMerge.Face1].Normal);
                        bool smallAngle = (dot > cosMinAngleBetweenFaces);
                        if (updateMerge.Cost <= simplificationToleranceSq || smallAngle)
                        {
                            // Write back
                            updateMerge.Cost = combined.Cost;
                            updateMerge.Plane = combined.Plane;
                            updateMerge.SmallAngle = smallAngle;
                            merges[i] = updateMerge; 
                        }
                        else
                        {
                            // Remove the merge
                            merges[i] = merges[--numMerges];
                        }
                    }
                }

                if (numMerged == 0)
                {
                    break; // Nothing merged, quit
                }

                // Check for any planes with small angles.  It is somewhat uncommon, but sometimes planes that either were not neighbors, or whose merge was dropped, later become nearly
                // parallel to each other as a result of another merge, and therefore need to be merged to each other
                numMerges = 0;
                for (int i = 0; i < numOriginalPlanes - 1; i++)
                {
                    if (removed[i]) continue;
                    for (int j = i + 1; j < numOriginalPlanes; j++)
                    {
                        if (removed[j]) continue;
                        if (math.dot(planes[i].Normal, planes[j].Normal) > cosMinAngleBetweenFaces)
                        {
                            OLSData combined = olsData[i];
                            combined.Include(olsData[j], sfloat.Zero);
                            combined.Solve(planes[i].Normal, planes[j].Normal);
                            merges[numMerges++] = new FaceMerge
                            {
                                Face0 = i,
                                Face1 = j,
                                Cost = combined.Cost,
                                Plane = combined.Plane,
                                SmallAngle = true
                            };
                        }
                    }
                }
                if (numMerges == 0)
                {
                    break; // No new merges found, quit
                }
            }

            // Compact the planes and push them in
            for (int i = numPlanes - 1; i >= 0; i--)
            {
                if (removed[i])
                {
                    planes[i] = planes[--numPlanes];
                }
                else
                {
                    planes[i] = new Plane(planes[i].Normal, planes[i].Distance + offset);
                }
            }

            // Calculate cross products of all face pairs
            NativeArray<float3> crosses = new NativeArray<float3>(numPlanes * (numPlanes - 1) / 2, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            int crossIndex = 0;
            for (int i = 0; i < numPlanes - 1; i++)
            {
                Plane plane0 = planes[i];
                float3 point0 = -plane0.Normal * plane0.Distance; // A point on plane0
                for (int j = i + 1; j < numPlanes; j++)
                {
                    Plane plane1 = planes[j];
                    float3 cross = math.cross(plane0.Normal, plane1.Normal);

                    // Get the line through the two planes and check if it intersects the domain.
                    // If not, then it has no intersections that will be kept and we can skip it in the n^4 loop.
                    float3 tangent0 = math.cross(plane0.Normal, cross);
                    float3 point01 = point0 - tangent0 * plane1.SignedDistanceToPoint(point0) / math.dot(plane1.Normal, tangent0); // point on both plane0 and plane1
                    float3 invCross = math.select(math.rcp(cross), math.sqrt(sfloat.MaxValue), cross == float3.zero);
                    float3 tMin = (m_IntegerSpaceAabb.Min - point01) * invCross;
                    float3 tMax = (m_IntegerSpaceAabb.Max - point01) * invCross;
                    float3 tEnter = math.min(tMin, tMax);
                    float3 tExit = math.max(tMin, tMax);
                    bool hit = (math.cmax(tEnter) <= math.cmin(tExit));
                    if (hit)
                    {
                        crosses[crossIndex] = cross;
                    }
                    else
                    {
                        crosses[crossIndex] = float3.zero;
                    }
                    crossIndex++;
                }
            }

            // Find all intersections of three planes.  Note, this is a very slow O(numPlanes^4) operation.
            // Intersections are calculated with double precision, otherwise points more than a couple meters from the origin can have error
            // above the tolerance for the inner loop.
            NativeArray<float3> newVertices = new NativeArray<float3>(Vertices.PeakCount * 100, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            int numNewVertices = 0;
            int indexMultiplier = 2 * numPlanes - 3;
            for (int i = 0; i < numPlanes - 2; i++)
            {
                int iBase = i * (indexMultiplier - i) / 2 - 1;
                for (int j = i + 1; j < numPlanes - 1; j++)
                {
                    // Test if discs i and j intersect
                    float3 ijCross = crosses[iBase + j];
                    if (math.all(ijCross == sfloat.Zero)) // broadphase test
                    {
                        continue;
                    }

                    int jBase = j * (indexMultiplier - j) / 2 - 1;
                    for (int k = j + 1; k < numPlanes; k++)
                    {
                        // Test if all discs intersect pairwise
                        float3 ikCross = crosses[iBase + k];
                        float3 jkCross = crosses[jBase + k];
                        if (math.all(ikCross == sfloat.Zero) || math.all(jkCross == sfloat.Zero)) // broadphase test
                        {
                            continue;
                        }

                        // Find the planes' point of intersection
                        float3 x;
                        {
                            sfloat det = math.dot(planes[i].Normal, jkCross);
                            if (math.abs(det) < sfloat.FromRaw(0x322bcc77))
                            {
                                continue;
                            }
                            sfloat invDet = sfloat.One / det;
                            x = (float3)((planes[i].Distance * jkCross - planes[j].Distance * ikCross + planes[k].Distance * ijCross) * -invDet);
                        }

                        // Test if the point is inside of all of the other planes
                        {
                            sfloat tolerance = sfloat.FromRaw(0x3727c5ac);
                            bool inside = true;
                            for (int l = 0; l < numPlanes; l++)
                            {
                                if (math.dot(planes[l].Normal, x) > tolerance - planes[l].Distance)
                                {
                                    inside = false;
                                    break;
                                }
                            }

                            if (!inside)
                            {
                                continue;
                            }
                        }

                        // Check if we already found an intersection that is almost exactly the same as x
                        sfloat minDistanceSq = sfloat.FromRaw(0x2edbe6ff);
                        bool keep = true;
                        for (int l = 0; l < numNewVertices; l++)
                        {
                            if (math.distancesq(newVertices[l], x) < minDistanceSq)
                            {
                                keep = false;
                                break;
                            }
                        }

                        if (keep)
                        {
                            newVertices[numNewVertices++] = x;
                        }
                    }
                    crossIndex++;
                }
            }

            // Check if there are enough vertices to form a 3D shape
            if (numNewVertices < 4)
            {
                // This can happen if the hull was nearly flat
                Rebuild2D();
                return sfloat.Zero;
            }

            // Rebuild faces using the plane intersection vertices
            if (numNewVertices >= 4) 
            {
                Reset();
                for (int i = 0; i < numNewVertices; i++)
                {
                    AddPoint(newVertices[i]);
                }
            }

            // When more than three planes meet at one point, the intersections computed from each subset of three planes can be slightly different
            // due to float rounding.  This creates unnecessary extra points in the hull and sometimes also numerical problems for BuildFaceIndices
            // from degenerate triangles.  This is fixed by another round of simplification with the error tolerance set low enough that the vertices
            // cannot move far enough to introduce new unintended faces.
            RemoveRedundantVertices();
            BuildFaceIndices(planes.GetSubArray(0, numPlanes));
            SimplifyVertices(k_PlaneEps, maxVertices);

            // Snap coords to their quantized values for the last build
            foreach (int v in Vertices.Indices)
            {
                Vertex vertex = Vertices[v];
                vertex.Position = m_IntegerSpace.ToFloatSpace(vertex.IntPosition);
                Vertices.Set(v, vertex);
            }

            BuildFaceIndices(planes.GetSubArray(0, numPlanes));

            return offset;
        }

        #endregion

        #region Edge methods

        /// <summary>
        /// Returns one of the triangle edges starting from a given vertex.
        /// Note: May be one of the inner edges of a face.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Edge GetVertexEdge(int vertexIndex)
        {
            Assert.IsTrue(Dimension == 3);
            foreach (int triangleIndex in Triangles.Indices)
            {
                Triangle triangle = Triangles[triangleIndex];
                if (triangle.Vertex0 == vertexIndex) return new Edge(triangleIndex, 0);
                if (triangle.Vertex1 == vertexIndex) return new Edge(triangleIndex, 1);
                if (triangle.Vertex2 == vertexIndex) return new Edge(triangleIndex, 2);
            }
            return Edge.Invalid;
        }

        /// <summary>
        /// Returns an edge's linked edge on the neighboring triangle.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Edge GetLinkedEdge(Edge edge) => edge.IsValid ? Triangles[edge.TriangleIndex].GetLink(edge.EdgeIndex) : edge;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int StartVertex(Edge edge) => Triangles[edge.TriangleIndex].GetVertex(edge.EdgeIndex);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int EndVertex(Edge edge) => StartVertex(edge.Next);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int ApexVertex(Edge edge) => StartVertex(edge.Prev);

        /// <summary>
        /// Returns (the first edge of) the first face.
        /// </summary>
        public FaceEdge GetFirstFace()
        {
            return NumFaces > 0 ? GetFirstFace(0) : FaceEdge.Invalid;
        }

        /// <summary>
        /// Returns the first face edge from a given face index.
        /// </summary>
        public FaceEdge GetFirstFace(int faceIndex)
        {
            foreach (int triangleIndex in Triangles.Indices)
            {
                if (Triangles[triangleIndex].FaceIndex != faceIndex)
                {
                    continue;
                }
                for (int i = 0; i < 3; ++i)
                {
                    var edge = new Edge(triangleIndex, i);
                    if (Triangles[GetLinkedEdge(edge).TriangleIndex].FaceIndex != faceIndex)
                    {
                        return new FaceEdge { Start = edge, Current = edge };
                    }
                }
            }
            return FaceEdge.Invalid;
        }

        /// <summary>
        /// Returns (the first edge of) the next face.
        /// </summary>
        public FaceEdge GetNextFace(FaceEdge fe)
        {
            int faceIndex = fe.IsValid ? Triangles[fe.Start.TriangleIndex].FaceIndex + 1 : 0;
            if (faceIndex < NumFaces)
                return GetFirstFace(faceIndex);
            return FaceEdge.Invalid;
        }

        /// <summary>
        /// Returns the next edge within a face.
        /// </summary>
        public FaceEdge GetNextFaceEdge(FaceEdge fe)
        {
            int faceIndex = Triangles[fe.Start.TriangleIndex].FaceIndex;
            bool found = false;
            fe.Current = fe.Current.Next;
            for (int n = Vertices[StartVertex(fe.Current)].Cardinality; n > 0; --n)
            {
                if (Triangles[GetLinkedEdge(fe.Current).TriangleIndex].FaceIndex == faceIndex)
                {
                    fe.Current = GetLinkedEdge(fe.Current).Next;
                }
                else
                {
                    found = true;
                    break;
                }
            }

            if (!found || fe.Current.Equals(fe.Start))
                return FaceEdge.Invalid;
            return fe;
        }

        #endregion

        #region Queries

        /// <summary>
        /// Returns the centroid of the convex hull.
        /// </summary>
        public float3 ComputeCentroid()
        {
            float4 sum = new float4(0);
            foreach (Vertex vertex in Vertices.Elements)
            {
                sum += new float4(vertex.Position, sfloat.One);
            }

            if (sum.w > sfloat.Zero)
                return sum.xyz / sum.w;
            return new float3(0);
        }

        /// <summary>
        /// Compute the mass properties of the convex hull.
        /// Note: Inertia computation adapted from S. Melax, http://www.melax.com/volint.
        /// </summary>        
        public unsafe void UpdateHullMassProperties()
        {
            var mp = new MassProperties();
            switch (Dimension)
            {
                case 0:
                    mp.CenterOfMass = Vertices[0].Position;
                    break;
                case 1:
                    mp.CenterOfMass = (Vertices[0].Position + Vertices[1].Position) * (sfloat)0.5f;
                    break;
                case 2:
                {
                    float3 offset = ComputeCentroid();
                    for (int n = Vertices.PeakCount, i = n - 1, j = 0; j < n; i = j++)
                    {
                        sfloat w = math.length(math.cross(Vertices[i].Position - offset, Vertices[j].Position - offset));
                        mp.CenterOfMass += (Vertices[i].Position + Vertices[j].Position + offset) * w;
                        mp.SurfaceArea += w;
                    }
                    mp.CenterOfMass /= mp.SurfaceArea * (sfloat)3.0f;
                    mp.InertiaTensor = float3x3.identity; // <todo>
                    mp.SurfaceArea *= (sfloat)0.5f;
                }
                break;
                case 3:
                {
                    float3 offset = ComputeCentroid();
                    int numTriangles = 0;
                    sfloat* dets = stackalloc sfloat[Triangles.Capacity];
                    foreach (int i in Triangles.Indices)
                    {
                        float3 v0 = Vertices[Triangles[i].Vertex0].Position - offset;
                        float3 v1 = Vertices[Triangles[i].Vertex1].Position - offset;
                        float3 v2 = Vertices[Triangles[i].Vertex2].Position - offset;
                        sfloat w = Det(v0, v1, v2);
                        mp.CenterOfMass += (v0 + v1 + v2) * w;
                        mp.Volume += w;
                        mp.SurfaceArea += math.length(math.cross(v1 - v0, v2 - v0));
                        dets[i] = w;
                        numTriangles++;
                    }

                    mp.CenterOfMass = mp.CenterOfMass / (mp.Volume * (sfloat)4.0f) + offset;

                    var diag = new float3(0);
                    var offd = new float3(0);

                    foreach (int i in Triangles.Indices)
                    {
                        float3 v0 = Vertices[Triangles[i].Vertex0].Position - mp.CenterOfMass;
                        float3 v1 = Vertices[Triangles[i].Vertex1].Position - mp.CenterOfMass;
                        float3 v2 = Vertices[Triangles[i].Vertex2].Position - mp.CenterOfMass;
                        diag += (v0 * v1 + v1 * v2 + v2 * v0 + v0 * v0 + v1 * v1 + v2 * v2) * dets[i];
                        offd += (v0.yzx * v1.zxy + v1.yzx * v2.zxy + v2.yzx * v0.zxy +
                                v0.yzx * v2.zxy + v1.yzx * v0.zxy + v2.yzx * v1.zxy +
                                (v0.yzx * v0.zxy + v1.yzx * v1.zxy + v2.yzx * v2.zxy) * 2) * dets[i];
                        numTriangles++;
                    }

                    diag /= mp.Volume * ((sfloat)10.0f);
                    offd /= mp.Volume * ((sfloat)20.0f);

                    mp.InertiaTensor.c0 = new float3(diag.y + diag.z, -offd.z, -offd.y);
                    mp.InertiaTensor.c1 = new float3(-offd.z, diag.x + diag.z, -offd.x);
                    mp.InertiaTensor.c2 = new float3(-offd.y, -offd.x, diag.x + diag.y);

                    mp.SurfaceArea *= (sfloat)0.5f;
                    mp.Volume *= sfloat.FromRaw(0x3e2aaaab);
                }
                break;
            }

            HullMassProperties = mp;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private Plane ComputePlane(int vertex0, int vertex1, int vertex2, bool fromIntCoordinates)
        {
            float3 cross; // non-normalized plane direction
            float3 point; // point on the plane
            if (fromIntCoordinates)
            {
                int3 o = Vertices[vertex0].IntPosition;
                int3 a = Vertices[vertex1].IntPosition - o;
                int3 b = Vertices[vertex2].IntPosition - o;
                IntCross(a, b, out long cx, out long cy, out long cz);
                sfloat scaleSq = m_IntegerSpace.Scale * m_IntegerSpace.Scale; // scale down to avoid overflow normalizing
                cross = new float3((sfloat)(int)cx * scaleSq, (sfloat)(int)cy * scaleSq, (sfloat)(int)cz * scaleSq);
                point = m_IntegerSpace.ToFloatSpace(o);
            }
            else
            {
                point = Vertices[vertex0].Position;
                float3 a = Vertices[vertex1].Position - point;
                float3 b = Vertices[vertex2].Position - point;
                cross = math.cross(a, b);
            }
            float3 n = math.normalize(cross);
            return new Plane(n, -math.dot(n, point));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Plane ComputePlane(int triangleIndex, bool fromIntCoordinates = true)
        {
            return ComputePlane(Triangles[triangleIndex].Vertex0, Triangles[triangleIndex].Vertex1, Triangles[triangleIndex].Vertex2, fromIntCoordinates);
        }

        #endregion

        #region int math

        // Sets cx, cy, cz = a x b, note that all components of a and b must be 31 bits or fewer
        private static void IntCross(int3 a, int3 b, out long cx, out long cy, out long cz)
        {
            cx = (long)a.y * b.z - (long)a.z * b.y;
            cy = (long)a.z * b.x - (long)a.x * b.z;
            cz = (long)a.x * b.y - (long)a.y * b.x;
        }

        // Computes det (b-a, c-a, d-a) and returns an integer that is positive when det is positive, negative when det is negative, and zero when det is zero.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private long IntDet(int3 a, int3 b, int3 c, int3 d)
        {
            int3 ab = b - a, ac = c - a, ad = d - a;
            IntCross(ab, ac, out long kx, out long ky, out long kz);
            if (m_IntResolution == IntResolution.Low)
            {
                // abcd coords are 16 bit, k are 35 bit, dot product is 54 bit and fits in long
                return kx * ad.x + ky * ad.y + kz * ad.z;
            }
            else
            {
                // abcd coords are 30 bit, k are 63 bit, dot product is 96 bit and won't fit in long so use int128
                Int128 det = Int128.Mul(kx, ad.x) + Int128.Mul(ky, ad.y) + Int128.Mul(kz, ad.z);
                return (long)(det.High | (det.Low & 1) | (det.Low >> 1));
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private long IntDet(int a, int b, int c, int d)
        {
            return IntDet(Vertices[a].IntPosition, Vertices[b].IntPosition, Vertices[c].IntPosition, Vertices[d].IntPosition);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private long IntDet(int a, int b, int c, int3 d)
        {
            return IntDet(Vertices[a].IntPosition, Vertices[b].IntPosition, Vertices[c].IntPosition, d);
        }

        #endregion

        public ConvexHullBuilder(
            NativeArray<float3> points,
            ConvexHullGenerationParameters generationParameters,
            int maxVertices, int maxFaces, int maxFaceVertices,
            out sfloat convexRadius
        )
        {
            int verticesCapacity = math.max(maxVertices, points.Length);
            var vertices = new NativeArray<Vertex>(verticesCapacity, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var triangles = new NativeArray<Triangle>(verticesCapacity * 2, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var planes = new NativeArray<Plane>(verticesCapacity * 2, Allocator.Temp, NativeArrayOptions.UninitializedMemory);

            var simplificationTolerance = generationParameters.SimplificationTolerance;
            var shrinkDistance = generationParameters.BevelRadius;
            var minAngle = generationParameters.MinimumAngle;

            // Build the points' AABB
            Aabb domain = new Aabb();
            for (int iPoint = 0; iPoint < points.Length; iPoint++)
            {
                domain.Include(points[iPoint]);
            }

            // Build the initial hull
            ConvexHullBuilder builder = new ConvexHullBuilder(vertices.Length, (Vertex*)vertices.GetUnsafePtr(),
                (Triangle*)triangles.GetUnsafePtr(), (Plane*)planes.GetUnsafePtr(),
                domain, simplificationTolerance, IntResolution.High);
            for (int iPoint = 0; iPoint < points.Length; iPoint++)
            {
                builder.AddPoint(points[iPoint]);
            }

            builder.RemoveRedundantVertices();

            // Simplify the vertices using half of the tolerance
            builder.BuildFaceIndices();
            builder.SimplifyVertices(simplificationTolerance * (sfloat)0.5f, maxVertices);
            builder.BuildFaceIndices();

            // Build mass properties before shrinking
            builder.UpdateHullMassProperties();

            // SimplifyFacesAndShrink() can increase the vertex count, potentially above the size of the input vertices.  Check if there is enough space in the
            // buffers, and if not then allocate temporary storage
            NativeArray<Vertex> tempVertices = new NativeArray<Vertex>();
            NativeArray<Triangle> tempTriangles = new NativeArray<Triangle>();
            NativeArray<Plane> tempPlanes = new NativeArray<Plane>();
            bool allocateTempBuilder = false;
            if (builder.Dimension == 3)
            {
                int maxNumVertices = 0;
                foreach (int v in builder.Vertices.Indices)
                {
                    maxNumVertices += builder.Vertices[v].Cardinality - 1;
                }

                allocateTempBuilder = true; // TEMP TESTING maxNumVertices > Vertices.Length;
                if (allocateTempBuilder)
                {
                    tempVertices = new NativeArray<Vertex>(maxNumVertices, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
                    tempTriangles = new NativeArray<Triangle>(maxNumVertices * 2, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
                    tempPlanes = new NativeArray<Plane>(maxNumVertices * 2, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
                    ConvexHullBuilder tempBuilder = new ConvexHullBuilder(maxNumVertices,
                        (Vertex*)tempVertices.GetUnsafePtr(),
                        (Triangle*)tempTriangles.GetUnsafePtr(), (Plane*)tempPlanes.GetUnsafePtr(), builder);
                    builder = tempBuilder;
                }

                // Merge faces
                convexRadius = builder.SimplifyFacesAndShrink(simplificationTolerance * (sfloat)0.5f, minAngle, shrinkDistance, maxFaces, maxVertices);
            }
            else
            {
                convexRadius = sfloat.Zero;
            }

            // Simplifier cannot directly enforce k_MaxFaceVertices or k_MaxFaces.  It can also fail to satisfy k_MaxFaces due to numerical error.
            // In these cases the hull is simplified by collapsing vertices until the counts are low enough, at the cost of possibly violating
            // simplificationTolerance or minAngle.
            maxVertices = builder.Vertices.PeakCount;
            while (true)
            {
                // Check if the face count is low enough, and in the 2D case, if the vertices per face is low enough
                if (builder.NumFaces <= maxFaces && (builder.Dimension == 3 || builder.Vertices.PeakCount < maxFaceVertices))
                {
                    // Iterate over all faces to check if any have too many vertices
                    bool simplify = false;
                    for (FaceEdge hullFace = builder.GetFirstFace(); hullFace.IsValid; hullFace = builder.GetNextFace(hullFace))
                    {
                        int numFaceVertices = 0;
                        for (FaceEdge edge = hullFace; edge.IsValid; edge = builder.GetNextFaceEdge(edge))
                        {
                            numFaceVertices++;
                        }

                        if (numFaceVertices > maxFaceVertices)
                        {
                            simplify = true;
                            break;
                        }
                    }

                    if (!simplify)
                    {
                        break;
                    }
                }

                // Reduce the vertex count 20%, but no need to go below the highest vertex count that always satisfies k_MaxFaces and k_MaxFaceVertices
                int limit = math.min(maxFaces / 2, maxFaceVertices);
                maxVertices = math.max((int)(maxVertices * 0.8f), limit);
                builder.SimplifyVertices(simplificationTolerance, maxVertices);
                builder.BuildFaceIndices();
                if (maxVertices == limit)
                {
                    break; // We should now be within the limits, and if not then something has gone wrong and it's better not to loop forever
                }
            }

            if (allocateTempBuilder)
            {
                // The vertex, triangle and face counts should now be within limits, so we can copy back to the original storage
                builder.Compact();
                ConvexHullBuilder tempBuilder = new ConvexHullBuilder(vertices.Length, (Vertex*)vertices.GetUnsafePtr(),
                    (Triangle*)triangles.GetUnsafePtr(), (Plane*)planes.GetUnsafePtr(), builder);
                builder = tempBuilder;
            }

            // Write back
            this = builder;
        }
    }

    // ConvexHullBuilder combined with NativeArrays to store its data
    // Keeping NativeArray out of the ConvexHullBuilder itself allows ConvexHullBuilder to be passed to Burst jobs
    struct ConvexHullBuilderStorage : IDisposable
    {
        private NativeArray<ConvexHullBuilder.Vertex> m_Vertices;
        private NativeArray<ConvexHullBuilder.Triangle> m_Triangles;
        private NativeArray<Plane> m_Planes;
        public ConvexHullBuilder Builder;

        public unsafe ConvexHullBuilderStorage(int verticesCapacity, Allocator allocator, Aabb domain, sfloat simplificationTolerance, ConvexHullBuilder.IntResolution resolution)
        {
            int trianglesCapacity = 2 * verticesCapacity;
            m_Vertices = new NativeArray<ConvexHullBuilder.Vertex>(verticesCapacity, allocator);
            m_Triangles = new NativeArray<ConvexHullBuilder.Triangle>(trianglesCapacity, allocator);
            m_Planes = new NativeArray<Plane>(trianglesCapacity, allocator);
            Builder = new ConvexHullBuilder(verticesCapacity, (ConvexHullBuilder.Vertex*)NativeArrayUnsafeUtility.GetUnsafePtr(m_Vertices), 
                (ConvexHullBuilder.Triangle*)NativeArrayUnsafeUtility.GetUnsafePtr(m_Triangles), (Plane*)NativeArrayUnsafeUtility.GetUnsafePtr(m_Planes),
                domain, simplificationTolerance, resolution);
        }

        public unsafe ConvexHullBuilderStorage(int verticesCapacity, Allocator allocator, ref ConvexHullBuilder builder)
        {
            m_Vertices = new NativeArray<ConvexHullBuilder.Vertex>(verticesCapacity, allocator);
            m_Triangles = new NativeArray<ConvexHullBuilder.Triangle>(verticesCapacity * 2, allocator);
            m_Planes = new NativeArray<Plane>(verticesCapacity * 2, allocator);
            Builder = new ConvexHullBuilder(verticesCapacity,  (ConvexHullBuilder.Vertex*)NativeArrayUnsafeUtility.GetUnsafePtr(m_Vertices),
                (ConvexHullBuilder.Triangle*)NativeArrayUnsafeUtility.GetUnsafePtr(m_Triangles), (Plane*)NativeArrayUnsafeUtility.GetUnsafePtr(m_Planes), builder);
        }

        public void Dispose()
        {
            if (m_Vertices.IsCreated)
            {
                m_Vertices.Dispose();
            }
            if (m_Triangles.IsCreated)
            {
                m_Triangles.Dispose();
            }
            if (m_Planes.IsCreated)
            {
                m_Planes.Dispose();
            }
        }
    }

    // Basic 128 bit signed integer arithmetic
    internal struct Int128
    {
        public ulong Low;
        public ulong High;

        const ulong k_Low32 = 0xffffffffUL;

        public bool IsNegative => (High & 0x8000000000000000UL) != 0;
        public bool IsNonNegative => (High & 0x8000000000000000UL) == 0;
        public bool IsZero => (High | Low) == 0;
        public bool IsPositive => IsNonNegative && !IsZero;

        public static Int128 Zero => new Int128 { High = 0, Low = 0 };

        public static Int128 operator +(Int128 a, Int128 b)
        {
            ulong low = a.Low + b.Low;
            ulong high = a.High + b.High;
            if (low < a.Low) high++;
            return new Int128
            {
                Low = low,
                High = high
            };
        }

        public static Int128 operator -(Int128 a, Int128 b)
        {
            return a + (-b);
        }

        public static Int128 operator -(Int128 a)
        {
            ulong low = ~a.Low + 1;
            ulong high = ~a.High;
            if (a.Low == 0) high++;
            return new Int128
            {
                Low = low,
                High = high
            };
        }

        public static Int128 Mul(long x, int y)
        {
            ulong absX = (ulong)math.abs(x);
            ulong absY = (ulong)math.abs(y);
            ulong lowProduct = (absX & k_Low32) * absY;
            ulong highProduct = (absX >> 32) * absY;
            ulong low = (highProduct << 32) + lowProduct;
            ulong carry = ((highProduct & k_Low32) + (lowProduct >> 32)) >> 32;
            ulong high = ((highProduct >> 32) & k_Low32) + carry;
            Int128 product = new Int128
            {
                Low = low,
                High = high
            };
            if (x < 0 ^ y < 0)
            {
                product = -product;
            }
            return product;
        }

        public static Int128 Mul(long x, long y)
        {
            ulong absX = (ulong)math.abs(x);
            ulong absY = (ulong)math.abs(y);

            ulong loX = absX & k_Low32;
            ulong hiX = absX >> 32;
            ulong loY = absY & k_Low32;
            ulong hiY = absY >> 32;

            ulong lolo = loX * loY;
            ulong lohi = loX * hiY;
            ulong hilo = hiX * loY;
            ulong hihi = hiX * hiY;

            ulong low = lolo + (lohi << 32) + (hilo << 32);
            ulong carry = ((lolo >> 32) + (lohi & k_Low32) + (hilo & k_Low32)) >> 32;
            ulong high = hihi + (lohi >> 32) + (hilo >> 32) + carry;

            Int128 product = new Int128
            {
                Low = low,
                High = high
            };
            if (x < 0 ^ y < 0)
            {
                product = -product;
            }
            return product;
        }
    }
}
