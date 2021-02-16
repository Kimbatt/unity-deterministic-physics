using System.Diagnostics;
using Unity.Burst;
using UnityS.Mathematics;
using static UnityS.Physics.Math;

namespace UnityS.Physics
{
    // Low level convex-convex distance query implementations
    static class ConvexConvexDistanceQueries
    {
        // Convex distance result
        public struct Result
        {
            public DistanceQueries.Result ClosestPoints;
            public uint3 Simplex;
            public int Iterations;

            public const uint InvalidSimplexVertex = 0xffffffff;

            public bool Valid => math.all(ClosestPoints.NormalInA == new float3(0));
            public int SimplexDimension => Simplex.z == InvalidSimplexVertex ? (Simplex.y == InvalidSimplexVertex ? 1 : 2) : 3;

            public int SimplexVertexA(int index) => (int)(Simplex[index] >> 16);
            public int SimplexVertexB(int index) => (int)(Simplex[index] & 0xffff);
        }

        public enum PenetrationHandling
        {
            DotNotCompute,
            Exact3D
        }

        // Supporting vertex
        [DebuggerDisplay("{Xyz}:{Id}")]
        private struct SupportVertex
        {
            public float3 Xyz;
            public uint Id;

            public int IdA => (int)(Id >> 16);
            public int IdB => (int)(Id & 0xffff);
        }

        // Simplex
        private struct Simplex
        {
            public SupportVertex A, B, C, D;
            public float3 Direction; // Points from the origin towards the closest point on the simplex
            public sfloat ScaledDistance; // ClosestPoint = Direction * ScaledDistance / lengthSq(Direction)
            public int NumVertices;

            /// <summary>
            /// Compute the closest point on the simplex, returns true if the simplex contains a duplicate vertex
            /// </summary>
            public void SolveDistance()
            {
                int inputVertices = NumVertices;

                switch (NumVertices)
                {
                    // Point.
                    case 1:
                        Direction = A.Xyz;
                        ScaledDistance = math.lengthsq(Direction);
                        break;

                    // Line.
                    case 2:
                    {
                        float3 delta = B.Xyz - A.Xyz;
                        sfloat den = math.dot(delta, delta);
                        sfloat num = math.dot(-A.Xyz, delta);

                        // Reduce if closest point do not project on the line segment.
                        if (num >= den) { NumVertices = 1; A = B; goto case 1; }

                        // Compute support direction
                        Direction = math.cross(math.cross(delta, A.Xyz), delta);
                        ScaledDistance = math.dot(Direction, A.Xyz);
                    }
                    break;

                    // Triangle.
                    case 3:
                    {
                        float3 ca = A.Xyz - C.Xyz;
                        float3 cb = B.Xyz - C.Xyz;
                        float3 n = math.cross(cb, ca);

                        // Reduce if closest point do not project in the triangle.
                        float3 crossA = math.cross(cb, n);
                        float3 crossB = math.cross(n, ca);
                        sfloat detA = math.dot(crossA, B.Xyz);
                        sfloat detB = math.dot(crossB, C.Xyz);
                        if (detA < sfloat.Zero)
                        {
                            if (detB >= sfloat.Zero || Det(n, crossA, C.Xyz) < sfloat.Zero)
                            {
                                A = B;
                            }
                        }
                        else if (detB >= sfloat.Zero)
                        {
                            sfloat dot = math.dot(C.Xyz, n);
                            if (dot < sfloat.Zero)
                            {
                                // Reorder vertices so that n points away from the origin
                                SupportVertex temp = A;
                                A = B;
                                B = temp;
                                n = -n;
                                dot = -dot;
                            }
                            Direction = n;
                            ScaledDistance = dot;
                            break;
                        }

                        B = C;
                        NumVertices = 2;
                        goto case 2;
                    }

                    // Tetrahedra.
                    case 4:
                    {
                        FourTransposedPoints tetra = new FourTransposedPoints(A.Xyz, B.Xyz, C.Xyz, D.Xyz);
                        FourTransposedPoints d = new FourTransposedPoints(D.Xyz);

                        // This routine finds the closest feature to the origin on the tetra by testing the origin against the planes of the
                        // voronoi diagram. If the origin is near the border of two regions in the diagram, then the plane tests might exclude
                        // it from both because of float rounding.  To avoid this problem we use some tolerance testing the face planes and let
                        // EPA handle those border cases.  1e-5 is a somewhat arbitrary value and the actual distance scales with the tetra, so
                        // this might need to be tuned later!
                        float3 faceTest = tetra.Cross(tetra.V1203).Dot(d).xyz;
                        if (math.all(faceTest >= sfloat.FromRaw(0xb727c5ac)))
                        {
                            // Origin is inside the tetra
                            Direction = float3.zero;
                            break;
                        }

                        // Check if the closest point is on a face
                        bool3 insideFace = (faceTest >= sfloat.Zero).xyz;
                        FourTransposedPoints edges = d - tetra;
                        FourTransposedPoints normals = edges.Cross(edges.V1203);
                        bool3 insideEdge0 = (normals.Cross(edges).Dot(d) >= sfloat.Zero).xyz;
                        bool3 insideEdge1 = (edges.V1203.Cross(normals).Dot(d) >= sfloat.Zero).xyz;
                        bool3 onFace = (insideEdge0 & insideEdge1 & !insideFace);
                        if (math.any(onFace))
                        {
                            if (onFace.y) { A = B; B = C; }
                            else if (onFace.z) { B = C; }
                        }
                        else
                        {
                            // Check if the closest point is on an edge
                            // TODO maybe we can safely drop two vertices in this case
                            bool3 insideVertex = (edges.Dot(d) >= 0).xyz;
                            bool3 onEdge = (!insideEdge0 & !insideEdge1.zxy & insideVertex);
                            if (math.any(onEdge.yz)) { A = B; B = C; }
                        }

                        C = D;
                        NumVertices = 3;
                        goto case 3;
                    }
                }
            }

            // Compute the barycentric coordinates of the closest point.
            public float4 ComputeBarycentricCoordinates(float3 closestPoint)
            {
                float4 coordinates = new float4(0);
                switch (NumVertices)
                {
                    case 1:
                        coordinates.x = sfloat.One;
                        break;
                    case 2:
                        sfloat distance = math.distance(A.Xyz, B.Xyz);
                        UnityEngine.Assertions.Assert.AreNotEqual(distance, sfloat.Zero); // TODO just checking if this happens in my tests
                        if (distance.IsZero()) // Very rare case, simplex is really 1D.
                        {
                            goto case 1;
                        }
                        coordinates.x = math.distance(B.Xyz, closestPoint) / distance;
                        coordinates.y = sfloat.One - coordinates.x;
                        break;
                    case 3:
                    {
                        coordinates.x = math.length(math.cross(B.Xyz - closestPoint, C.Xyz - closestPoint));
                        coordinates.y = math.length(math.cross(C.Xyz - closestPoint, A.Xyz - closestPoint));
                        coordinates.z = math.length(math.cross(A.Xyz - closestPoint, B.Xyz - closestPoint));
                        sfloat sum = math.csum(coordinates.xyz);
                        if (sum.IsZero()) // Very rare case, simplex is really 2D.  Happens because of int->float conversion from the hull builder.
                        {
                            // Choose the two farthest apart vertices to keep
                            float3 lengthsSq = new float3(math.lengthsq(A.Xyz - B.Xyz), math.lengthsq(B.Xyz - C.Xyz), math.lengthsq(C.Xyz - A.Xyz));
                            bool3 longest = math.cmin(lengthsSq) == lengthsSq;
                            if (longest.y)
                            {
                                A.Xyz = C.Xyz;
                            }
                            else if (longest.z)
                            {
                                A.Xyz = B.Xyz;
                                B.Xyz = C.Xyz;
                            }
                            coordinates.z = sfloat.Zero;
                            NumVertices = 2;
                            goto case 2;
                        }
                        coordinates /= sum;
                        break;
                    }
                    case 4:
                    {
                        coordinates.x = Det(D.Xyz, C.Xyz, B.Xyz);
                        coordinates.y = Det(D.Xyz, A.Xyz, C.Xyz);
                        coordinates.z = Det(D.Xyz, B.Xyz, A.Xyz);
                        coordinates.w = Det(A.Xyz, B.Xyz, C.Xyz);
                        sfloat sum = math.csum(coordinates.xyzw);
                        UnityEngine.Assertions.Assert.AreNotEqual(sum, sfloat.Zero); // TODO just checking that this doesn't happen in my tests
                        if (sum.IsZero()) // Unexpected case, may introduce significant error by dropping a vertex but it's better than nan
                        {
                            coordinates.zw = sfloat.Zero;
                            NumVertices = 3;
                            goto case 3;
                        }
                        coordinates /= sum;
                        break;
                    }
                }

                return coordinates;
            }
        }

        /// <summary>
        /// Generalized convex-convex distance.
        /// </summary>
        /// <param name="verticesA">Vertices of the first collider in local space</param>
        /// <param name="verticesB">Vertices of the second collider in local space</param>
        /// <param name="aFromB">Transform from the local space of B to the local space of A</param>
        /// <param name="penetrationHandling">How to compute penetration.</param>
        /// <returns></returns>
        public static unsafe Result ConvexConvex(
            float3* verticesA, int numVerticesA, float3* verticesB, int numVerticesB,
            [NoAlias] in MTransform aFromB, PenetrationHandling penetrationHandling)
        {
            sfloat epsTerminationSq = sfloat.FromRaw(0x322bcc77); // Main loop quits when it cannot find a point that improves the simplex by at least this much
            sfloat epsPenetrationSq = sfloat.FromRaw(0x3089705f); // Epsilon used to check for penetration.  Should be smaller than shape cast ConvexConvex keepDistance^2.

            // Initialize simplex.
            Simplex simplex = new Simplex();
            simplex.NumVertices = 1;
            simplex.A = GetSupportingVertex(new float3(sfloat.One, sfloat.Zero, sfloat.Zero), verticesA, numVerticesA, verticesB, numVerticesB, aFromB);
            simplex.Direction = simplex.A.Xyz;
            simplex.ScaledDistance = math.lengthsq(simplex.A.Xyz);
            sfloat scaleSq = simplex.ScaledDistance;

            // Iterate.        
            int iteration = 0;
            bool penetration = false;
            const int maxIterations = 64;
            for (; iteration < maxIterations; ++iteration)
            {
                // Find a new support vertex
                SupportVertex newSv = GetSupportingVertex(-simplex.Direction, verticesA, numVerticesA, verticesB, numVerticesB, aFromB);

                // If the new vertex is not significantly closer to the origin, quit
                sfloat scaledImprovement = math.dot(simplex.A.Xyz - newSv.Xyz, simplex.Direction);
                if (scaledImprovement * math.abs(scaledImprovement) < epsTerminationSq * scaleSq)
                {
                    break;
                }

                // Add the new vertex and reduce the simplex
                switch (simplex.NumVertices++)
                {
                    case 1: simplex.B = newSv; break;
                    case 2: simplex.C = newSv; break;
                    default: simplex.D = newSv; break;
                }
                simplex.SolveDistance();

                // Check for penetration
                scaleSq = math.lengthsq(simplex.Direction);
                sfloat scaledDistanceSq = simplex.ScaledDistance * simplex.ScaledDistance;
                if (simplex.NumVertices == 4 || scaledDistanceSq <= epsPenetrationSq * scaleSq)
                {
                    penetration = true;
                    break;
                }
            }

            // Finalize result.
            var ret = new Result { Iterations = iteration + 1 };

            // Handle penetration.
            if (penetration && penetrationHandling != PenetrationHandling.DotNotCompute)
            {
                // Allocate a hull for EPA
                int verticesCapacity = 64;
                int triangleCapacity = 2 * verticesCapacity;
                ConvexHullBuilder.Vertex* vertices = stackalloc ConvexHullBuilder.Vertex[verticesCapacity];
                ConvexHullBuilder.Triangle* triangles = stackalloc ConvexHullBuilder.Triangle[triangleCapacity];
                Aabb domain = GetSupportingAabb(verticesA, numVerticesA, verticesB, numVerticesB, aFromB);
                sfloat simplificationTolerance = sfloat.Zero;
                var hull = new ConvexHullBuilder(verticesCapacity, vertices, triangles, null,
                    domain, simplificationTolerance, ConvexHullBuilder.IntResolution.Low);

                // Add simplex vertices to the hull, remove any vertices from the simplex that do not increase the hull dimension
                hull.AddPoint(simplex.A.Xyz, simplex.A.Id);
                if (simplex.NumVertices > 1)
                {
                    hull.AddPoint(simplex.B.Xyz, simplex.B.Id);
                    if (simplex.NumVertices > 2)
                    {
                        int dimension = hull.Dimension;
                        hull.AddPoint(simplex.C.Xyz, simplex.C.Id);
                        if (dimension == 0 && hull.Dimension == 1)
                        {
                            simplex.B = simplex.C;
                        }
                        if (simplex.NumVertices > 3)
                        {
                            dimension = hull.Dimension;
                            hull.AddPoint(simplex.D.Xyz, simplex.D.Id);
                            if (dimension > hull.Dimension)
                            {
                                if (dimension == 0)
                                {
                                    simplex.B = simplex.D;
                                }
                                else if (dimension == 1)
                                {
                                    simplex.C = simplex.D;
                                }
                            }
                        }
                    }
                }
                simplex.NumVertices = (hull.Dimension + 1);

                // If the simplex is not 3D, try expanding the hull in all directions
                while (hull.Dimension < 3)
                {
                    // Choose expansion directions
                    float3 support0, support1, support2;
                    switch (simplex.NumVertices)
                    {
                        case 1:
                            support0 = new float3(sfloat.One, sfloat.Zero, sfloat.Zero);
                            support1 = new float3(sfloat.Zero, sfloat.One, sfloat.Zero);
                            support2 = new float3(sfloat.Zero, sfloat.Zero, sfloat.One);
                            break;
                        case 2:
                            Math.CalculatePerpendicularNormalized(math.normalize(simplex.B.Xyz - simplex.A.Xyz), out support0, out support1);
                            support2 = float3.zero;
                            break;
                        default:
                            UnityEngine.Assertions.Assert.IsTrue(simplex.NumVertices == 3);
                            support0 = math.cross(simplex.B.Xyz - simplex.A.Xyz, simplex.C.Xyz - simplex.A.Xyz);
                            support1 = float3.zero;
                            support2 = float3.zero;
                            break;
                    }

                    // Try each one
                    int numSupports = 4 - simplex.NumVertices;
                    bool success = false;
                    for (int i = 0; i < numSupports; i++)
                    {
                        for (int j = 0; j < 2; j++) // +/- each direction
                        {
                            SupportVertex vertex = GetSupportingVertex(support0, verticesA, numVerticesA, verticesB, numVerticesB, aFromB);
                            hull.AddPoint(vertex.Xyz, vertex.Id);
                            if (hull.Dimension == simplex.NumVertices)
                            {
                                switch (simplex.NumVertices)
                                {
                                    case 1: simplex.B = vertex; break;
                                    case 2: simplex.C = vertex; break;
                                    default: simplex.D = vertex; break;
                                }

                                // Next dimension
                                success = true;
                                simplex.NumVertices++;
                                i = numSupports;
                                break;
                            }
                            support0 = -support0;
                        }
                        support0 = support1;
                        support1 = support2;
                    }

                    if (!success)
                    {
                        break;
                    }
                }

                // We can still fail to build a tetrahedron if the minkowski difference is really flat.
                // In those cases just find the closest point to the origin on the infinite extension of the simplex (point / line / plane)
                if (hull.Dimension != 3)
                {
                    switch (simplex.NumVertices)
                    {
                        case 1:
                        {
                            ret.ClosestPoints.Distance = math.length(simplex.A.Xyz);
                            ret.ClosestPoints.NormalInA = -math.normalizesafe(simplex.A.Xyz, new float3(sfloat.One, sfloat.Zero, sfloat.Zero));
                            break;
                        }
                        case 2:
                        {
                            float3 edge = math.normalize(simplex.B.Xyz - simplex.A.Xyz);
                            float3 direction = math.cross(math.cross(edge, simplex.A.Xyz), edge);
                            Math.CalculatePerpendicularNormalized(edge, out float3 safeNormal, out float3 unused); // backup, take any direction perpendicular to the edge
                            float3 normal = math.normalizesafe(direction, safeNormal);
                            ret.ClosestPoints.Distance = math.dot(normal, simplex.A.Xyz);
                            ret.ClosestPoints.NormalInA = -normal;
                            break;
                        }
                        default:
                        {
                            UnityEngine.Assertions.Assert.IsTrue(simplex.NumVertices == 3);
                            float3 cross = math.cross(simplex.B.Xyz - simplex.A.Xyz, simplex.C.Xyz - simplex.A.Xyz);
                            sfloat crossLengthSq = math.lengthsq(cross);
                            if (crossLengthSq < sfloat.FromRaw(0x322bcc77)) // hull builder can accept extremely thin triangles for which we cannot compute an accurate normal
                            {
                                simplex.NumVertices = 2;
                                goto case 2;
                            }
                            float3 normal = cross * math.rsqrt(crossLengthSq);
                            sfloat dot = math.dot(normal, simplex.A.Xyz);
                            ret.ClosestPoints.Distance = math.abs(dot);
                            ret.ClosestPoints.NormalInA = math.select(-normal, normal, dot < sfloat.Zero);
                            break;
                        }
                    }
                }
                else
                {
                    int closestTriangleIndex;
                    Plane closestPlane = new Plane();
                    sfloat stopThreshold = sfloat.FromRaw(0x38d1b717);
                    uint* uidsCache = stackalloc uint[triangleCapacity];
                    for (int i = 0; i < triangleCapacity; i++)
                    {
                        uidsCache[i] = 0;
                    }
                    sfloat* distancesCache = stackalloc sfloat[triangleCapacity];
                    do
                    {
                        // Select closest triangle.
                        closestTriangleIndex = -1;
                        foreach (int triangleIndex in hull.Triangles.Indices)
                        {
                            if (hull.Triangles[triangleIndex].Uid != uidsCache[triangleIndex])
                            {
                                uidsCache[triangleIndex] = hull.Triangles[triangleIndex].Uid;
                                distancesCache[triangleIndex] = hull.ComputePlane(triangleIndex).Distance;
                            }
                            if (closestTriangleIndex == -1 || distancesCache[closestTriangleIndex] < distancesCache[triangleIndex])
                            {
                                closestTriangleIndex = triangleIndex;
                            }
                        }
                        closestPlane = hull.ComputePlane(closestTriangleIndex);

                        // Add supporting vertex or exit. 
                        SupportVertex sv = GetSupportingVertex(closestPlane.Normal, verticesA, numVerticesA, verticesB, numVerticesB, aFromB);
                        sfloat d2P = math.dot(closestPlane.Normal, sv.Xyz) + closestPlane.Distance;
                        if (math.abs(d2P) > stopThreshold && hull.AddPoint(sv.Xyz, sv.Id))
                            stopThreshold *= sfloat.FromRaw(0x3fa66666);
                        else
                            break;
                    } while (++iteration < maxIterations);

                    // There could be multiple triangles in the closest plane, pick the one that has the closest point to the origin on its face
                    foreach (int triangleIndex in hull.Triangles.Indices)
                    {
                        if (distancesCache[triangleIndex] >= closestPlane.Distance - sfloat.FromRaw(0x38d1b717))
                        {
                            ConvexHullBuilder.Triangle triangle = hull.Triangles[triangleIndex];
                            float3 a = hull.Vertices[triangle.Vertex0].Position;
                            float3 b = hull.Vertices[triangle.Vertex1].Position;
                            float3 c = hull.Vertices[triangle.Vertex2].Position;
                            float3 cross = math.cross(b - a, c - a);
                            float3 dets = new float3(
                                math.dot(math.cross(a - c, cross), a),
                                math.dot(math.cross(b - a, cross), b),
                                math.dot(math.cross(c - b, cross), c));
                            if (math.all(dets >= 0))
                            {
                                Plane plane = hull.ComputePlane(triangleIndex);
                                if (math.dot(plane.Normal, closestPlane.Normal) > sfloat.FromRaw(0x3f7ff972))
                                {
                                    closestTriangleIndex = triangleIndex;
                                    closestPlane = hull.ComputePlane(triangleIndex);
                                }
                                break;
                            }
                        }
                    }

                    // Generate simplex.
                    {
                        ConvexHullBuilder.Triangle triangle = hull.Triangles[closestTriangleIndex];
                        simplex.NumVertices = 3;
                        simplex.A.Xyz = hull.Vertices[triangle.Vertex0].Position; simplex.A.Id = hull.Vertices[triangle.Vertex0].UserData;
                        simplex.B.Xyz = hull.Vertices[triangle.Vertex1].Position; simplex.B.Id = hull.Vertices[triangle.Vertex1].UserData;
                        simplex.C.Xyz = hull.Vertices[triangle.Vertex2].Position; simplex.C.Id = hull.Vertices[triangle.Vertex2].UserData;
                        simplex.Direction = -closestPlane.Normal;
                        simplex.ScaledDistance = closestPlane.Distance;

                        // Set normal and distance.
                        ret.ClosestPoints.NormalInA = -closestPlane.Normal;
                        ret.ClosestPoints.Distance = closestPlane.Distance;
                    }
                }
            }
            else
            {
                // Compute distance and normal.
                sfloat lengthSq = math.lengthsq(simplex.Direction);
                sfloat invLength = math.rsqrt(lengthSq);
                bool smallLength = lengthSq.IsZero();
                ret.ClosestPoints.Distance = math.select(simplex.ScaledDistance * invLength, sfloat.Zero, smallLength);
                ret.ClosestPoints.NormalInA = math.select(simplex.Direction * invLength, new float3(sfloat.One, sfloat.Zero, sfloat.Zero), smallLength);

                // Make sure the normal is always valid.
                if (!math.all(math.isfinite(ret.ClosestPoints.NormalInA)))
                {
                    ret.ClosestPoints.NormalInA = new float3(sfloat.One, sfloat.Zero, sfloat.Zero);
                }
            }

            // Compute position.
            float3 closestPoint = ret.ClosestPoints.NormalInA * ret.ClosestPoints.Distance;
            float4 coordinates = simplex.ComputeBarycentricCoordinates(closestPoint);
            ret.ClosestPoints.PositionOnAinA =
                verticesA[simplex.A.IdA] * coordinates.x +
                verticesA[simplex.B.IdA] * coordinates.y +
                verticesA[simplex.C.IdA] * coordinates.z +
                verticesA[simplex.D.IdA] * coordinates.w;

            // Encode simplex.
            ret.Simplex.x = simplex.A.Id;
            ret.Simplex.y = simplex.NumVertices >= 2 ? simplex.B.Id : Result.InvalidSimplexVertex;
            ret.Simplex.z = simplex.NumVertices >= 3 ? simplex.C.Id : Result.InvalidSimplexVertex;

            // Done.
            UnityEngine.Assertions.Assert.IsTrue(math.isfinite(ret.ClosestPoints.Distance));
            UnityEngine.Assertions.Assert.IsTrue(math.abs(math.lengthsq(ret.ClosestPoints.NormalInA) - sfloat.One) < sfloat.FromRaw(0x3727c5ac));
            return ret;
        }

        // Returns the supporting vertex index given a direction in local space.
        private static unsafe int GetSupportingVertexIndex(float3 direction, float3* vertices, int numVertices)
        {
            int maxI = -1;
            sfloat maxD = sfloat.Zero;
            for (int i = 0; i < numVertices; ++i)
            {
                sfloat d = math.dot(direction, vertices[i]);
                if (maxI == -1 || d > maxD)
                {
                    maxI = i;
                    maxD = d;
                }
            }
            return maxI;
        }

        // Returns the supporting vertex of the CSO given a direction in 'A' space.
        private static unsafe SupportVertex GetSupportingVertex(
            float3 direction, float3* verticesA, int numVerticesA, float3* verticesB, int numVerticesB, [NoAlias] in MTransform aFromB)
        {
            int ia = GetSupportingVertexIndex(direction, verticesA, numVerticesA);
            int ib = GetSupportingVertexIndex(math.mul(aFromB.InverseRotation, -direction), verticesB, numVerticesB);
            return new SupportVertex { Xyz = verticesA[ia] - Mul(aFromB, verticesB[ib]), Id = ((uint)ia) << 16 | (uint)ib };
        }

        // Returns an AABB containing the CSO in A-space
        private static unsafe Aabb GetSupportingAabb(
            float3* verticesA, int numVerticesA, float3* verticesB, int numVerticesB, MTransform aFromB)
        {
            Aabb aabbA = new Aabb { Min = verticesA[0], Max = verticesA[0] };
            for (int i = 1; i < numVerticesA; i++)
            {
                aabbA.Min = math.min(aabbA.Min, verticesA[i]);
                aabbA.Max = math.max(aabbA.Max, verticesA[i]);
            }

            Aabb aabbB = new Aabb { Min = verticesB[0], Max = verticesB[0] };
            for (int i = 1; i < numVerticesB; i++)
            {
                aabbB.Min = math.min(aabbB.Min, verticesB[i]);
                aabbB.Max = math.max(aabbB.Max, verticesB[i]);
            }

            Aabb aabbBinA = Math.TransformAabb(aFromB, aabbB);
            return new Aabb { Min = aabbA.Min - aabbBinA.Max, Max = aabbA.Max - aabbBinA.Min };
        }
    }
}
