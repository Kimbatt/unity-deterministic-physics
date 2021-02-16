using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityS.Mathematics;

namespace UnityS.Physics
{
    // Utilities for building physics meshes
    internal static class MeshBuilder
    {
        internal struct TempSectionRanges
        {
            public int PrimitivesFlagsMin;
            public int PrimitivesFlagsLength;
            public int PrimitivesMin;
            public int PrimitivesLength;
            public int VerticesMin;
            public int VerticesLength;
        }
        
        internal struct TempSection
        {
            public NativeList<Mesh.PrimitiveFlags>  PrimitivesFlags;
            public NativeList<Mesh.PrimitiveVertexIndices> Primitives;
            public NativeList<float3> Vertices;
            public NativeList<TempSectionRanges> Ranges;
        }

        internal static unsafe TempSection BuildSections(BoundingVolumeHierarchy.Node* nodes, int nodeCount, NativeList<MeshConnectivityBuilder.Primitive> primitives)
        {
            var tempSections = new TempSection()
            {
                PrimitivesFlags = new NativeList<Mesh.PrimitiveFlags>(Allocator.Temp),
                Primitives = new NativeList<Mesh.PrimitiveVertexIndices>(Allocator.Temp),
                Vertices = new NativeList<float3>(Allocator.Temp),
                Ranges = new NativeList<TempSectionRanges>(Allocator.Temp)
            };

            if (primitives.Length == 0)
            {
                // Early-out in the case of no input primitives
                return tempSections;
            }

            // Traverse the tree and break out geometry into sections
            int* nodesIndexStack = stackalloc int[BoundingVolumeHierarchy.Constants.UnaryStackSize];
            int stackSize = 1;
            nodesIndexStack[0] = 1;

            const float uniqueVerticesPerPrimitiveFactor = 1.5f;

            var primitivesCountInSubTree = ProducedPrimitivesCountPerSubTree(nodes, nodeCount);
            
            var subTreeIndices = new NativeList<int>(Allocator.Temp);
            var nodeIndices = new NativeList<int>(Allocator.Temp);
            var tmpVertices = new NativeList<float3>(Allocator.Temp);
            do
            {
                int nodeIndex = nodesIndexStack[--stackSize];
                int subTreeVertexCountEstimate = (int)(uniqueVerticesPerPrimitiveFactor * primitivesCountInSubTree[nodeIndex]);

                subTreeIndices.Clear();

                if (subTreeVertexCountEstimate < Mesh.Section.MaxNumVertices)
                {
                    subTreeIndices.Add(nodeIndex);
                }
                else
                {
                    // Sub tree is too big, break it up.
                    BoundingVolumeHierarchy.Node node = nodes[nodeIndex];

                    for (int i = 0; i < 4; i++)
                    {
                        if (node.IsChildValid(i))
                        {
                            int childNodeIndex = node.Data[i];
                            int nodeSubTreeVertexCount = (int)(uniqueVerticesPerPrimitiveFactor * primitivesCountInSubTree[childNodeIndex]);

                            if (nodeSubTreeVertexCount < Mesh.Section.MaxNumVertices)
                            {
                                subTreeIndices.Add(childNodeIndex);
                            }
                            else
                            {
                                nodesIndexStack[stackSize++] = childNodeIndex;
                            }
                        }
                    }
                }

                float tempUniqueVertexPrimitiveFactor = 1.0f;
                const float factorStepIncrement = 0.25f;

                while (subTreeIndices.Length > 0)
                {
                    // Try to combine sub trees if multiple sub trees can fit into one section.
                    nodeIndices.Clear();
                    int vertexCountEstimate = 0;

                    for(var i = 0; i < subTreeIndices.Length; ++i)
                    {
                        var subTreeNodeIndex = subTreeIndices[i];
                        int nodeIndexCount = (int)(tempUniqueVertexPrimitiveFactor * primitivesCountInSubTree[subTreeNodeIndex]);
                        if (vertexCountEstimate + nodeIndexCount < Mesh.Section.MaxNumVertices)
                        {
                            vertexCountEstimate += nodeIndexCount;
                            nodeIndices.Add(subTreeNodeIndex);
                        }
                    }

                    if (nodeIndices.Length == 0)
                    {
                        // We failed to fit any sub tree into sections.
                        // Split up nodes and push them to stack.
                        for(var index = 0; index < subTreeIndices.Length; ++index)
                        {
                            var subTreeNodeIndex = subTreeIndices[index];
                            BoundingVolumeHierarchy.Node nodeToSplit = nodes[subTreeNodeIndex];

                            for (int i = 0; i < 4; i++)
                            {
                                if (nodeToSplit.IsChildValid(i))
                                {
                                    nodesIndexStack[stackSize++] = nodeToSplit.Data[i];
                                }
                            }
                        }

                        subTreeIndices.Clear();
                        continue;
                    }

                    // Collect vertices from all sub trees.
                    tmpVertices.Clear();
                    for(var i = 0; i < nodeIndices.Length; ++i)
                    {
                        var subTreeNodeIndex = nodeIndices[i];
                        CollectAllVerticesFromSubTree(nodes, subTreeNodeIndex, primitives, tmpVertices);
                    }

                    var vertexIndices = new NativeArray<int>(tmpVertices.Length, Allocator.Temp);
                    for (int i = 0; i < vertexIndices.Length; i++)
                    {
                        vertexIndices[i] = i;
                    }

                    NativeList<float3> uniqueVertices = MeshConnectivityBuilder.WeldVertices(vertexIndices, new NativeArray<float3>(tmpVertices, Allocator.Temp));

                    if (uniqueVertices.Length < Mesh.Section.MaxNumVertices)
                    {
                        BuildSectionGeometry(tempSections, primitives, nodeIndices, nodes, new NativeArray<float3>(uniqueVertices, Allocator.Temp));

                        // Remove used indices
                        for(var i = 0; i < nodeIndices.Length; ++i)
                        {
                            var nodeTreeIndex = nodeIndices[i];
                            subTreeIndices.RemoveAtSwapBack(subTreeIndices.IndexOf(nodeTreeIndex));
                        }
                    }
                    else
                    {
                        // Estimate of num vertices per primitives was wrong.
                        // Increase the tempUniqueVertexPrimitiveFactor.
                        tempUniqueVertexPrimitiveFactor += factorStepIncrement;
                    }
                }
            }
            while (stackSize > 0);

            return tempSections;
        }

        private static unsafe NativeArray<int> ProducedPrimitivesCountPerSubTree(BoundingVolumeHierarchy.Node* nodes, int nodeCount)
        {
            var primitivesPerNode = new NativeArray<int>(nodeCount, Allocator.Temp);

            for (int nodeIndex = nodeCount - 1; nodeIndex >= 0; nodeIndex--)
            {
                BoundingVolumeHierarchy.Node node = nodes[nodeIndex];

                if (node.IsLeaf)
                {
                    primitivesPerNode[nodeIndex] = node.NumValidChildren();
                }
                else
                {
                    primitivesPerNode[nodeIndex] =
                        primitivesPerNode[node.Data[0]] + primitivesPerNode[node.Data[1]] +
                        primitivesPerNode[node.Data[2]] + primitivesPerNode[node.Data[3]];
                }
            }

            return primitivesPerNode;
        }

        private static unsafe void CollectAllVerticesFromSubTree(BoundingVolumeHierarchy.Node* nodes, int subTreeNodeIndex,
            NativeList<MeshConnectivityBuilder.Primitive> primitives, NativeList<float3> vertices)
        {

            int* nodesIndexStack = stackalloc int[BoundingVolumeHierarchy.Constants.UnaryStackSize];
            int stackSize = 1;
            nodesIndexStack[0] = subTreeNodeIndex;

            do
            {
                BoundingVolumeHierarchy.Node node = nodes[nodesIndexStack[--stackSize]];

                if (node.IsLeaf)
                {
                    for (int i = 0; i < 4; i++)
                    {
                        if (node.IsChildValid(i))
                        {
                            MeshConnectivityBuilder.Primitive p = primitives[node.Data[i]];
                            vertices.Add(p.Vertices[0]);
                            vertices.Add(p.Vertices[1]);
                            vertices.Add(p.Vertices[2]);

                            if ((p.Flags & MeshConnectivityBuilder.PrimitiveFlags.DefaultTrianglePairFlags) != 0)
                            {
                                vertices.Add(p.Vertices[3]);
                            }
                        }
                    }
                }
                else
                {
                    for (int i = 0; i < 4; i++)
                    {
                        if (node.IsChildValid(i))
                        {
                            nodesIndexStack[stackSize++] = node.Data[i];
                        }
                    }
                }
            } while (stackSize > 0);

        }

        private static Mesh.PrimitiveFlags ConvertPrimitiveFlags(MeshConnectivityBuilder.PrimitiveFlags flags)
        {
            Mesh.PrimitiveFlags newFlags = 0;
            newFlags |= (flags & MeshConnectivityBuilder.PrimitiveFlags.IsTrianglePair) != 0 ? Mesh.PrimitiveFlags.IsTrianglePair : Mesh.PrimitiveFlags.IsTriangle;

            if ((flags & MeshConnectivityBuilder.PrimitiveFlags.IsFlatConvexQuad) == MeshConnectivityBuilder.PrimitiveFlags.IsFlatConvexQuad)
            {
                newFlags |= Mesh.PrimitiveFlags.IsQuad;
            }

            return newFlags;
        }

        private static unsafe void BuildSectionGeometry(TempSection sections, NativeList<MeshConnectivityBuilder.Primitive> primitives, NativeList<int> subTreeNodeIndices, BoundingVolumeHierarchy.Node* nodes, NativeArray<float3> vertices)
        {
            var sectionIndex = sections.Ranges.Length;

            var newSectionRange = new TempSectionRanges
            {
                VerticesMin = sections.Vertices.Length,
                PrimitivesFlagsMin = sections.PrimitivesFlags.Length,
                PrimitivesMin = sections.Primitives.Length
            };
            
            sections.Vertices.AddRange(vertices);

            int* nodesIndexStack = stackalloc int[BoundingVolumeHierarchy.Constants.UnaryStackSize];

            for(var rootIndex = 0; rootIndex < subTreeNodeIndices.Length; ++rootIndex)
            {
                var root = subTreeNodeIndices[rootIndex];
                int stackSize = 1;
                nodesIndexStack[0] = root;

                do
                {
                    int nodeIndex = nodesIndexStack[--stackSize];
                    ref BoundingVolumeHierarchy.Node node = ref nodes[nodeIndex];

                    if (node.IsLeaf)
                    {
                        for (int i = 0; i < 4; i++)
                        {
                            if (node.IsChildValid(i))
                            {
                                MeshConnectivityBuilder.Primitive p = primitives[node.Data[i]];
                                sections.PrimitivesFlags.Add(ConvertPrimitiveFlags(p.Flags));

                                int vertexCount = (p.Flags &MeshConnectivityBuilder.PrimitiveFlags.IsTrianglePair) != 0 ? 4 : 3;

                                Mesh.PrimitiveVertexIndices sectionPrimitive = new Mesh.PrimitiveVertexIndices();
                                byte* vertexIndices = &sectionPrimitive.A;

                                for (int v = 0; v < vertexCount; v++)
                                {
                                    vertexIndices[v] = (byte)vertices.IndexOf(p.Vertices[v]);
                                }

                                if (vertexCount == 3)
                                {
                                    sectionPrimitive.D = sectionPrimitive.C;
                                }

                                sections.Primitives.Add(sectionPrimitive);

                                int primitiveSectionIndex = sections.Primitives.Length - newSectionRange.PrimitivesMin - 1;

                                // Update primitive index in the BVH.
                                node.Data[i] = (sectionIndex << 8) | primitiveSectionIndex;
                            }
                        }
                    }
                    else
                    {
                        for (int i = 0; i < 4; i++)
                        {
                            if (node.IsChildValid(i))
                            {
                                nodesIndexStack[stackSize++] = node.Data[i];
                            }
                        }
                    }
                } while (stackSize > 0);
            }

            newSectionRange.VerticesLength = sections.Vertices.Length - newSectionRange.VerticesMin;
            newSectionRange.PrimitivesLength = sections.Primitives.Length - newSectionRange.PrimitivesMin;
            newSectionRange.PrimitivesFlagsLength = sections.PrimitivesFlags.Length - newSectionRange.PrimitivesFlagsMin;
            
            sections.Ranges.Add(newSectionRange);
        }
    }

    internal struct MeshConnectivityBuilder
    {
        static sfloat k_MergeCoplanarTrianglesTolerance => sfloat.FromRaw(0x38d1b717);

        internal NativeArray<Vertex> Vertices;
        internal NativeArray<Triangle> Triangles;
        internal NativeArray<Edge> Edges;

        /// Vertex.
        internal struct Vertex
        {
            /// Number of triangles referencing this vertex, or, equivalently, number of edge starting from this vertex.
            internal int Cardinality;

            /// true if the vertex is on the boundary, false otherwise.
            /// Note: if true the first edge of the ring is naked.
            /// Conditions: number of naked edges in the 1-ring is greater than 0.
            internal bool Boundary;

            /// true if the vertex is on the border, false otherwise.
            /// Note: if true the first edge of the ring is naked.
            /// Conditions: number of naked edges in the 1-ring is equal to 1.
            internal bool Border;

            /// true is the vertex 1-ring is manifold.
            /// Conditions: number of naked edges in the 1-ring is less than 2 and cardinality is greater than 0.
            internal bool Manifold;

            /// Index of the first edge.
            internal int FirstEdge;
        }

        /// (Half) Edge.
        internal struct Edge
        {
            internal static Edge Invalid() => new Edge { IsValid = false };

            // Triangle index
            internal int Triangle;

            // Starting vertex index
            internal int Start;

            internal bool IsValid;
        }

        internal struct Triangle
        {
            public void Clear()
            {
                IsValid = false;
                Edge0.IsValid = false;
                Edge1.IsValid = false;
                Edge2.IsValid = false;
            }

            // Broken up rather than an array because we need native containers of triangles elsewhere in the code, and
            // nested native containers aren't supported.
            internal Edge Edge0;
            internal Edge Edge1;
            internal Edge Edge2;

            internal Edge Links(int edge)
            {
                SafetyChecks.CheckIndexAndThrow(edge, 3);
                switch (edge)
                {
                    case 0:
                        return Edge0;
                    case 1:
                        return Edge1;
                    case 2:
                        return Edge2;
                    default:
                        return default;
                }
            }

            internal void SetLinks(int edge, Edge newEdge)
            {
                SafetyChecks.CheckIndexAndThrow(edge, 3);
                switch (edge)
                {
                    case 0:
                        Edge0 = newEdge;
                        break;
                    case 1:
                        Edge1 = newEdge;
                        break;
                    case 2:
                        Edge2 = newEdge;
                        break;
                }
            }
            
            internal bool IsValid;
        }

        [Flags]
        internal enum PrimitiveFlags
        {
            IsTrianglePair = 1 << 0,
            IsFlat = 1 << 1,
            IsConvex = 1 << 2,

            DisableInternalEdge = 1 << 3,
            DisableAllEdges = 1 << 4,

            IsFlatConvexQuad = IsTrianglePair | IsFlat | IsConvex,

            DefaultTriangleFlags = IsFlat | IsConvex,
            DefaultTrianglePairFlags = IsTrianglePair
        }

        internal struct Primitive
        {
            internal float3x4 Vertices;
            internal PrimitiveFlags Flags;
        }

        /// Get the opposite edge.
        internal Edge GetLink(Edge e) => e.IsValid ? Triangles[e.Triangle].Links(e.Start) : e;
        internal bool IsBound(Edge e) => GetLink(e).IsValid;
        internal bool IsNaked(Edge e) => !IsBound(e);
        internal Edge GetNext(Edge e) => e.IsValid ? new Edge { Triangle = e.Triangle, Start = (e.Start + 1) % 3, IsValid = true } : e;
        internal Edge GetPrev(Edge e) => e.IsValid ? new Edge { Triangle = e.Triangle, Start = (e.Start + 2) % 3, IsValid = true } : e;

        internal int GetStartVertexIndex(Edge e) => Triangles[e.Triangle].Links(e.Start).Start;

        internal int GetEndVertexIndex(Edge e) => Triangles[e.Triangle].Links((e.Start + 1) % 3).Start;

        internal bool IsEdgeConcaveOrFlat(Edge edge, NativeArray<int3> triangles, NativeArray<float3> vertices, NativeArray<float4> planes)
        {
            if (IsNaked(edge))
            {
                return false;
            }

            float3 apex = vertices[GetApexVertexIndex(triangles, edge)];
            if (Math.Dotxyz1(planes[edge.Triangle], apex) < -k_MergeCoplanarTrianglesTolerance)
            {
                return false;
            }

            return true;
        }

        internal bool IsTriangleConcaveOrFlat(Edge edge, NativeArray<int3> triangles, NativeArray<float3> vertices, NativeArray<float4> planes)
        {
            for (int i = 0; i < 3; i++)
            {
                Edge e = GetNext(edge);
                if (!IsEdgeConcaveOrFlat(e, triangles, vertices, planes))
                {
                    return false;
                }
            }

            return true;
        }

        internal bool IsFlat(Edge edge, NativeArray<int3> triangles, NativeArray<float3> vertices, NativeArray<float4> planes)
        {
            Edge link = GetLink(edge);
            if (!link.IsValid)
            {
                return false;
            }

            float3 apex = vertices[GetApexVertexIndex(triangles, link)];
            bool flat = math.abs(Math.Dotxyz1(planes[edge.Triangle], apex)) < k_MergeCoplanarTrianglesTolerance;

            apex = vertices[GetApexVertexIndex(triangles, edge)];
            flat |= math.abs(Math.Dotxyz1(planes[link.Triangle], apex)) < k_MergeCoplanarTrianglesTolerance;

            return flat;
        }

        internal bool IsConvexQuad(Primitive quad, Edge edge, NativeArray<float4> planes)
        {
            float4x2 quadPlanes;
            quadPlanes.c0 = planes[edge.Triangle];
            quadPlanes.c1 = planes[GetLink(edge).Triangle];
            if (Math.Dotxyz1(quadPlanes[0], quad.Vertices[3]) < k_MergeCoplanarTrianglesTolerance)
            {
                if (Math.Dotxyz1(quadPlanes[1], quad.Vertices[1]) < k_MergeCoplanarTrianglesTolerance)
                {
                    bool convex = true;
                    for (int i = 0; convex && i < 4; i++)
                    {
                        float3 delta = quad.Vertices[(i + 1) % 4] - quad.Vertices[i];
                        float3 normal = math.normalize(math.cross(delta, quadPlanes[i >> 1].xyz));
                        float4 edgePlane = new float4(normal, math.dot(-normal, quad.Vertices[i]));
                        for (int j = 0; j < 2; j++)
                        {
                            if (Math.Dotxyz1(edgePlane, quad.Vertices[(i + j + 1) % 4]) > k_MergeCoplanarTrianglesTolerance)
                            {
                                convex = false;
                                break;
                            }
                        }
                    }
                    return convex;
                }
            }

            return false;
        }

        internal bool CanEdgeBeDisabled(Edge e, NativeArray<PrimitiveFlags> flags, NativeArray<int3> triangles, NativeArray<float3> vertices, NativeArray<float4> planes)
        {
            if (!e.IsValid || IsEdgeConcaveOrFlat(e, triangles, vertices, planes) || (flags[e.Triangle] & PrimitiveFlags.DisableAllEdges) != 0)
            {
                return false;
            }

            return true;
        }

        internal bool CanAllEdgesBeDisabled(NativeArray<Edge> edges, NativeArray<PrimitiveFlags> flags, NativeArray<int3> triangles, NativeArray<float3> vertices, NativeArray<float4> planes)
        {
            bool allDisabled = true;
            for(var i = 0; i < edges.Length; ++i)
            {
                allDisabled &= CanEdgeBeDisabled(edges[i], flags, triangles, vertices, planes);
            }

            return allDisabled;
        }

        // Utility function
        private static void Swap<T>(ref T a, ref T b) where T : struct
        {
            T t = a;
            a = b;
            b = t;
        }

        private struct VertexWithHash
        {
            internal float3 Vertex;
            internal ulong Hash;
            internal int Index;
        }

        private struct SortVertexWithHashByHash : IComparer<VertexWithHash>
        {
            public int Compare(VertexWithHash x, VertexWithHash y)
            {
                return x.Hash.CompareTo(y.Hash);
            }
        }

        private static ulong SpatialHash(float3 vertex)
        {
            uint x, y, z;
            x = vertex.x.RawValue;
            y = vertex.y.RawValue;
            z = vertex.z.RawValue;

            const ulong p1 = 73856093;
            const ulong p2 = 19349663;
            const ulong p3 = 83492791;

            return (x * p1) ^ (y * p2) ^ (z * p3);
        }

        public static NativeList<float3> WeldVertices(NativeArray<int> indices, NativeArray<float3> vertices)
        {
            int numVertices = vertices.Length;
            var verticesAndHashes = new NativeArray<VertexWithHash>(numVertices, Allocator.Temp);
            for (int i = 0; i < numVertices; i++)
            {
                verticesAndHashes[i] = new VertexWithHash()
                {
                    Index = i,
                    Vertex = vertices[i],
                    Hash = SpatialHash(vertices[i])
                };
            }

            var uniqueVertices = new NativeList<float3>(Allocator.Temp);
            var remap = new NativeArray<int>(numVertices, Allocator.Temp);
            verticesAndHashes.Sort(new SortVertexWithHashByHash());

            for (int i = 0; i < numVertices; i++)
            {
                if (verticesAndHashes[i].Index == int.MaxValue)
                {
                    continue;
                }

                uniqueVertices.Add(vertices[verticesAndHashes[i].Index]);
                remap[verticesAndHashes[i].Index] = uniqueVertices.Length - 1;

                for (int j = i + 1; j < numVertices; j++)
                {
                    if (verticesAndHashes[j].Index == int.MaxValue)
                    {
                        continue;
                    }

                    if (verticesAndHashes[i].Hash == verticesAndHashes[j].Hash)
                    {
                        if (verticesAndHashes[i].Vertex.x == verticesAndHashes[j].Vertex.x &&
                            verticesAndHashes[i].Vertex.y == verticesAndHashes[j].Vertex.y &&
                            verticesAndHashes[i].Vertex.z == verticesAndHashes[j].Vertex.z)
                        {
                            remap[verticesAndHashes[j].Index] = remap[verticesAndHashes[i].Index];

                            verticesAndHashes[j] = new VertexWithHash()
                            {
                                Index = int.MaxValue, 
                                Vertex = verticesAndHashes[j].Vertex, 
                                Hash = verticesAndHashes[j].Hash
                            };
                        }
                    }
                    else
                    {
                        break;
                    }
                }
            }

            for (int i = 0; i < indices.Length; i++)
            {
                indices[i] = remap[indices[i]];
            }

            return uniqueVertices;
        }

        public static bool IsTriangleDegenerate(float3 a, float3 b, float3 c)
        {
            sfloat defaultTriangleDegeneracyTolerance = sfloat.FromRaw(0x33d6bf95);

            // Small area check
            {
                float3 edge1 = a - b;
                float3 edge2 = a - c;
                float3 cross = math.cross(edge1, edge2);

                float3 edge1B = b - a;
                float3 edge2B = b - c;
                float3 crossB = math.cross(edge1B, edge2B);

                bool cmp0 = defaultTriangleDegeneracyTolerance > math.lengthsq(cross);
                bool cmp1 = defaultTriangleDegeneracyTolerance > math.lengthsq(crossB);
                if (cmp0 || cmp1)
                {
                    return true;
                }
            }

            // Point triangle distance check
            {
                float3 q = a - b;
                float3 r = c - b;

                sfloat qq = math.dot(q, q);
                sfloat rr = math.dot(r, r);
                sfloat qr = math.dot(q, r);

                sfloat qqrr = qq * rr;
                sfloat qrqr = qr * qr;
                sfloat det = (qqrr - qrqr);

                return det.IsZero();
            }
        }

        internal unsafe MeshConnectivityBuilder(NativeArray<int3> triangles, NativeArray<float3> vertices)
        {
            int numTriangles = triangles.Length;
            int numVertices = vertices.Length;

            Vertices = new NativeArray<Vertex>(numVertices, Allocator.Temp);
            Triangles = new NativeArray<Triangle>(numTriangles, Allocator.Temp);
            
            for (int i = 0; i < numTriangles; i++)
            {
                var triangle = new Triangle();
                triangle.Clear();
                Triangles[i] = triangle;
            }

            int numEdges = 0;

            // Compute cardinality and triangle flags.
            for (int triangleIndex = 0; triangleIndex < numTriangles; triangleIndex++)
            {
                ((Triangle*)Triangles.GetUnsafePtr())[triangleIndex].IsValid =
                    triangles[triangleIndex][0] != triangles[triangleIndex][1] &&
                    triangles[triangleIndex][1] != triangles[triangleIndex][2] &&
                    triangles[triangleIndex][0] != triangles[triangleIndex][2];
                if (Triangles[triangleIndex].IsValid)
                {
                    ((Vertex*)Vertices.GetUnsafePtr())[triangles[triangleIndex][0]].Cardinality++;
                    ((Vertex*)Vertices.GetUnsafePtr())[triangles[triangleIndex][1]].Cardinality++;
                    ((Vertex*)Vertices.GetUnsafePtr())[triangles[triangleIndex][2]].Cardinality++;
                }
            }

            // Compute vertex first edge index.
            for (int vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex)
            {
                int cardinality = Vertices[vertexIndex].Cardinality;
                ((Vertex*)Vertices.GetUnsafePtr())[vertexIndex].FirstEdge = cardinality > 0 ? numEdges : 0;
                numEdges += cardinality;
            }

            // Compute edges and triangles links.
            var counters = new NativeArray<int>(numVertices, Allocator.Temp);
            Edges = new NativeArray<Edge>(numEdges, Allocator.Temp);

            for (int triangleIndex = 0; triangleIndex < numTriangles; triangleIndex++)
            {
                if (!Triangles[triangleIndex].IsValid)
                {
                    continue;
                }

                for (int i = 2, j = 0; j < 3; i = j++)
                {
                    int vertexI = triangles[triangleIndex][i];
                    int thisEdgeIndex = Vertices[vertexI].FirstEdge + counters[vertexI]++;
                    Edges[thisEdgeIndex] = new Edge { Triangle = triangleIndex, Start = i, IsValid = true };

                    int vertexJ = triangles[triangleIndex][j];
                    Vertex other = Vertices[vertexJ];
                    int count = counters[vertexJ];

                    for (int k = 0; k < count; k++)
                    {
                        Edge edge = Edges[other.FirstEdge + k];

                        int endVertexOffset = (edge.Start + 1) % 3;
                        int endVertex = triangles[edge.Triangle][endVertexOffset];
                        if (endVertex == vertexI)
                        {
                            ((Triangle*)Triangles.GetUnsafePtr())[triangleIndex].SetLinks(i, edge);
                            ((Triangle*)Triangles.GetUnsafePtr())[edge.Triangle].SetLinks(edge.Start, Edges[thisEdgeIndex]);
                            break;
                        }
                    }
                }
            }
            
            // Compute vertices attributes.
            for (int vertexIndex = 0; vertexIndex < numVertices; vertexIndex++)
            {
                int nakedEdgeIndex = -1;
                int numNakedEdge = 0;
                {
                    int firstEdgeIndex = Vertices[vertexIndex].FirstEdge;
                    int numVertexEdges = Vertices[vertexIndex].Cardinality;
                    for (int i = 0; i < numVertexEdges; i++)
                    {
                        int edgeIndex = firstEdgeIndex + i;
                        if (IsNaked(Edges[edgeIndex]))
                        {
                            nakedEdgeIndex = i;
                            numNakedEdge++;
                        }
                    }
                }

                ref Vertex vertex = ref ((Vertex*)Vertices.GetUnsafePtr())[vertexIndex];
                vertex.Manifold = numNakedEdge < 2 && vertex.Cardinality > 0;
                vertex.Boundary = numNakedEdge > 0;
                vertex.Border = numNakedEdge == 1 && vertex.Manifold;

                // Make sure that naked edge appears first.
                if (nakedEdgeIndex > 0)
                {
                    Swap(ref ((Edge*)Edges.GetUnsafePtr())[vertex.FirstEdge], ref ((Edge*)Edges.GetUnsafePtr())[vertex.FirstEdge + nakedEdgeIndex]);
                }

                // Order ring as fan.
                if (vertex.Manifold)
                {
                    int firstEdge = vertex.FirstEdge;
                    int count = vertex.Cardinality;
                    for (int i = 0; i < count - 1; i++)
                    {
                        Edge prevEdge = GetPrev(Edges[firstEdge + i]);
                        if (IsBound(prevEdge))
                        {
                            int triangle = GetLink(prevEdge).Triangle;
                            if (Edges[firstEdge + i + 1].Triangle != triangle)
                            {
                                bool found = false;
                                for (int j = i + 2; j < count; ++j)
                                {
                                    if (Edges[firstEdge + j].Triangle == triangle)
                                    {
                                        Swap(ref ((Edge*)Edges.GetUnsafePtr())[firstEdge + i + 1], ref ((Edge*)Edges.GetUnsafePtr())[firstEdge + j]);
                                        found = true;
                                        break;
                                    }
                                }

                                if (!found)
                                {
                                    vertex.Manifold = false;
                                    vertex.Border = false;
                                    break;
                                }
                            }
                        }
                    }

                    if (vertex.Manifold)
                    {
                        Edge lastEdge = GetPrev(Edges[firstEdge + count - 1]);
                        if (vertex.Border)
                        {
                            if (IsBound(lastEdge))
                            {
                                vertex.Manifold = false;
                                vertex.Border = false;
                            }
                        }
                        else
                        {
                            if (IsNaked(lastEdge) || GetLink(lastEdge).Triangle != Edges[firstEdge].Triangle)
                            {
                                vertex.Manifold = false;
                            }
                        }
                    }
                }
            }
        }

        private struct EdgeData : IComparable<EdgeData>
        {
            internal Edge Edge;
            internal sfloat Value;

            public int CompareTo(EdgeData other)
            {
                return Value.CompareTo(other.Value);
            }
        }

        private static int4 GetVertexIndices(NativeArray<int3> triangles, Edge edge)
        {
            int4 vertexIndices;
            int triangle = edge.Triangle;
            vertexIndices.x = triangles[triangle][edge.Start];
            vertexIndices.y = triangles[triangle][(edge.Start + 1) % 3];
            vertexIndices.z = triangles[triangle][(edge.Start + 2) % 3];
            vertexIndices.w = 0;
            return vertexIndices;
        }

        private static int GetApexVertexIndex(NativeArray<int3> triangles, Edge edge)
        {
            int triangleIndex = edge.Triangle;
            return triangles[triangleIndex][(edge.Start + 2) % 3];
        }

        private static sfloat CalcTwiceSurfaceArea(float3 a, float3 b, float3 c)
        {
            float3 d0 = b - a;
            float3 d1 = c - a;
            return math.length(math.cross(d0, d1));
        }

        internal unsafe NativeList<Primitive> EnumerateQuadDominantGeometry(NativeArray<int3> triangles, NativeList<float3> vertices)
        {
            int numTriangles = triangles.Length;
            var flags = new NativeArray<PrimitiveFlags>(numTriangles, Allocator.Temp);
            var quadRoots = new NativeList<Edge>(Allocator.Temp);
            var triangleRoots = new NativeList<Edge>(Allocator.Temp);

            // Generate triangle planes
            var planes = new NativeArray<float4>(numTriangles, Allocator.Temp);
            
            for (int i = 0; i < numTriangles; i++)
            {
                float3 v0 = vertices[triangles[i][0]];
                float3 v1 = vertices[triangles[i][1]];
                float3 v2 = vertices[triangles[i][2]];

                float3 normal = math.normalize(math.cross(v0 - v1, v0 - v2));
                planes[i] = new float4(normal, -math.dot(normal, v0));
            }

            var edges = new NativeArray<EdgeData>(Edges.Length, Allocator.Temp);

            for (int i = 0; i < edges.Length; i++)
            {
                Edge e = Edges[i];

                ref EdgeData edgeData = ref ((EdgeData*)edges.GetUnsafePtr())[i];
                edgeData.Edge = Edge.Invalid();
                edgeData.Value = sfloat.MaxValue;

                if (IsBound(e))
                {
                    Edge linkEdge = GetLink(e);
                    int4 vis = GetVertexIndices(triangles, e);
                    vis[3] = GetApexVertexIndex(triangles, linkEdge);

                    float3x4 quadVertices = new float3x4(vertices[vis[0]], vertices[vis[1]], vertices[vis[2]], vertices[vis[3]]);
                    Aabb quadAabb = Aabb.CreateFromPoints(quadVertices);

                    sfloat aabbSurfaceArea = quadAabb.SurfaceArea;

                    if (aabbSurfaceArea > Math.Constants.Eps)
                    {
                        sfloat quadSurfaceArea = CalcTwiceSurfaceArea(quadVertices[0], quadVertices[1], quadVertices[2]) + CalcTwiceSurfaceArea(quadVertices[0], quadVertices[1], quadVertices[3]);
                        edgeData.Value = (aabbSurfaceArea - quadSurfaceArea) / aabbSurfaceArea;
                        edgeData.Edge = vis[0] < vis[1] ? e : linkEdge;
                    }
                }
            }

            edges.Sort();

            var freeTriangles = new NativeArray<bool>(numTriangles, Allocator.Temp);
            for (var i = 0; i < freeTriangles.Length; ++i)
            {
                freeTriangles[i] = true;
            }

            var primitives = new NativeList<Primitive>(Allocator.Temp);

            // Generate quads
            for(var edgeIndex = 0; edgeIndex < edges.Length; ++edgeIndex)
            {
                var edgeData = edges[edgeIndex];
                
                if (!edgeData.Edge.IsValid)
                {
                    break;
                }

                int t0 = edgeData.Edge.Triangle;
                Edge linkEdge = GetLink(edgeData.Edge);
                int t1 = linkEdge.Triangle;

                if (freeTriangles[t0] && freeTriangles[t1])
                {
                    Edge nextEdge = GetNext(edgeData.Edge);
                    int4 vis = GetVertexIndices(triangles, nextEdge);
                    vis[3] = GetApexVertexIndex(triangles, linkEdge);

                    var primitive = new Primitive
                    {
                        Vertices = new float3x4(vertices[vis[0]], vertices[vis[1]], vertices[vis[2]], vertices[vis[3]]),
                        Flags = PrimitiveFlags.DefaultTrianglePairFlags
                    };

                    if (IsTriangleDegenerate(primitive.Vertices[0], primitive.Vertices[1], primitive.Vertices[2]) ||
                        IsTriangleDegenerate(primitive.Vertices[0], primitive.Vertices[2], primitive.Vertices[3]))
                    {
                        continue;
                    }

                    if (IsEdgeConcaveOrFlat(edgeData.Edge, triangles, vertices, planes))
                    {
                        primitive.Flags |= PrimitiveFlags.DisableInternalEdge;

                        if (IsTriangleConcaveOrFlat(edgeData.Edge, triangles, vertices, planes) &&
                            IsTriangleConcaveOrFlat(linkEdge, triangles, vertices, planes))
                        {
                            primitive.Flags |= PrimitiveFlags.DisableAllEdges;
                        }
                    }

                    if (IsFlat(edgeData.Edge, triangles, vertices, planes))
                    {
                        primitive.Flags |= PrimitiveFlags.IsFlat;
                    }

                    if (IsConvexQuad(primitive, edgeData.Edge, planes))
                    {
                        primitive.Flags |= PrimitiveFlags.IsConvex;
                    }

                    primitives.Add(primitive);

                    freeTriangles[t0] = false;
                    freeTriangles[t1] = false;

                    flags[t0] = primitive.Flags;
                    flags[t1] = primitive.Flags;

                    quadRoots.Add(edgeData.Edge);
                }
            }

            // Generate triangles
            for (int i = 0; i < numTriangles; i++)
            {
                if (!freeTriangles[i])
                {
                    continue;
                }

                Edge edge = new Edge { Triangle = i, Start = 0, IsValid = true };
                while (edge.IsValid && GetStartVertexIndex(edge) < GetEndVertexIndex(edge))
                {
                    edge = GetNext(edge);
                }

                int4 vis = GetVertexIndices(triangles, edge);

                var primitive = new Primitive
                {
                    Vertices = new float3x4(vertices[vis[0]], vertices[vis[1]], vertices[vis[2]], vertices[vis[2]]),
                    Flags = PrimitiveFlags.DefaultTriangleFlags
                };

                if (IsTriangleConcaveOrFlat(edge, triangles, vertices, planes))
                {
                    primitive.Flags |= PrimitiveFlags.DisableAllEdges;
                }

                primitives.Add(primitive);
                triangleRoots.Add(edge);
                flags[edge.Triangle] = primitive.Flags;
            }

            DisableEdgesOfAdjacentPrimitives(primitives, triangles, vertices, planes, flags, quadRoots, triangleRoots);
            
            return primitives;
        }

        private void DisableEdgesOfAdjacentPrimitives(
            NativeList<Primitive> primitives, NativeArray<int3> triangles, NativeArray<float3> vertices, NativeArray<float4> planes, NativeArray<PrimitiveFlags> flags,
            NativeList<Edge> quadRoots, NativeList<Edge> triangleRoots)
        {
            var outerBoundary = new NativeArray<Edge>(4, Allocator.Temp);
                
            for (int quadIndex = 0; quadIndex < quadRoots.Length; quadIndex++)
            {
                Edge root = quadRoots[quadIndex];
                Edge link = GetLink(root);
                PrimitiveFlags quadFlags = flags[root.Triangle];
                if ((quadFlags & PrimitiveFlags.IsFlatConvexQuad) == PrimitiveFlags.IsFlatConvexQuad &&
                    (quadFlags & PrimitiveFlags.DisableAllEdges) != PrimitiveFlags.DisableAllEdges)
                {
                    outerBoundary[0] = GetLink(GetNext(root));
                    outerBoundary[1] = GetLink(GetPrev(root));
                    outerBoundary[2] = GetLink(GetNext(link));
                    outerBoundary[3] = GetLink(GetPrev(link));

                    if (CanAllEdgesBeDisabled(outerBoundary, flags, triangles, vertices, planes))
                    {
                        quadFlags |= PrimitiveFlags.DisableAllEdges;
                    }
                }

                // Sync triangle flags.
                flags[root.Triangle] = quadFlags;
                flags[link.Triangle] = quadFlags;

                // Write primitive flags.
                primitives[quadIndex] = new Primitive
                {
                    Vertices = primitives[quadIndex].Vertices,
                    Flags = quadFlags
                };
            }
            
            outerBoundary = new NativeArray<Edge>(3, Allocator.Temp);

            for (int triangleIndex = 0; triangleIndex < triangleRoots.Length; triangleIndex++)
            {
                Edge root = triangleRoots[triangleIndex];
                PrimitiveFlags triangleFlags = flags[root.Triangle];
                if ((triangleFlags & PrimitiveFlags.DisableAllEdges) == 0)
                {
                    outerBoundary[0] = GetLink(root);
                    outerBoundary[1] = GetLink(GetNext(root));
                    outerBoundary[2] = GetLink(GetPrev(root));

                    if (CanAllEdgesBeDisabled(outerBoundary, flags, triangles, vertices, planes))
                    {
                        triangleFlags |= PrimitiveFlags.DisableAllEdges;
                    }
                }

                // Sync triangle flags.
                flags[root.Triangle] = triangleFlags;

                // Write primitive flags.
                int primitiveIndex = quadRoots.Length + triangleIndex;
                primitives[primitiveIndex] = new Primitive
                {
                    Vertices = primitives[primitiveIndex].Vertices,
                    Flags = triangleFlags
                };
            }
        }
    }
}
