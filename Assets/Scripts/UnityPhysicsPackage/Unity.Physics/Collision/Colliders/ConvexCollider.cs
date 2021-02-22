using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityS.Mathematics;
using Unity.Entities;
using UnityEngine.Assertions;

namespace UnityS.Physics
{
    [Serializable]
    public struct ConvexHullGenerationParameters : IEquatable<ConvexHullGenerationParameters>
    {
        internal const string k_BevelRadiusTooltip =
            "Determines how rounded the edges of the convex shape will be. A value greater than 0 results in more optimized collision, at the expense of some shape detail.";

        static sfloat k_DefaultSimplificationTolerance => sfloat.FromRaw(0x3c75c28f);
        static sfloat k_DefaultBevelRadius => sfloat.FromRaw(0x3d4ccccd);
        static sfloat k_DefaultMinAngle => sfloat.FromRaw(0x3d32b8c2); // 2.5 degrees

        public static readonly ConvexHullGenerationParameters Default = new ConvexHullGenerationParameters
        {
            SimplificationTolerance = k_DefaultSimplificationTolerance,
            BevelRadius = k_DefaultBevelRadius,
            MinimumAngle = k_DefaultMinAngle
        };

        public sfloat SimplificationTolerance { get => m_SimplificationTolerance; set => m_SimplificationTolerance = value; }
        [UnityEngine.Tooltip("Specifies maximum distance that any input point may be moved when simplifying convex hull.")]
        [UnityEngine.SerializeField]
        sfloat m_SimplificationTolerance;

        public sfloat BevelRadius { get => m_BevelRadius; set => m_BevelRadius = value; }
        [UnityEngine.Tooltip(k_BevelRadiusTooltip)]
        [UnityEngine.SerializeField]
        sfloat m_BevelRadius;

        public sfloat MinimumAngle { get => m_MinimumAngle; set => m_MinimumAngle = value; }
        [UnityEngine.Tooltip("Specifies the angle between adjacent faces below which they should be made coplanar.")]
        [UnityEngine.SerializeField]
        sfloat m_MinimumAngle;

        public bool Equals(ConvexHullGenerationParameters other) =>
            m_SimplificationTolerance == other.m_SimplificationTolerance
            && m_BevelRadius == other.m_BevelRadius
            && m_MinimumAngle == other.m_MinimumAngle;

        public override int GetHashCode() =>
            unchecked((int)math.hash(new float3(m_SimplificationTolerance, m_BevelRadius, m_MinimumAngle)));
    }

    // A collider in the shape of an arbitrary convex hull.
    // Warning: This is just the header, it is followed by variable sized data in memory.
    // Therefore this struct must always be passed by reference, never by value.
    public struct ConvexCollider : IConvexCollider
    {
        // Header
        private ConvexColliderHeader m_Header;
        internal ConvexHull ConvexHull;

        internal const int k_MaxVertices = 252;
        internal const int k_MaxFaces = 252;
        internal const int k_MaxFaceVertices = ConvexConvexManifoldQueries.Manifold.k_MaxNumContacts;

        // followed by variable sized convex hull data

        #region Construction

        // Create a convex collider from the given point cloud.
        public static BlobAssetReference<Collider> Create(
            NativeArray<float3> points, ConvexHullGenerationParameters generationParameters
        ) =>
            Create(points, generationParameters, CollisionFilter.Default, Material.Default);

        public static BlobAssetReference<Collider> Create(
            NativeArray<float3> points, ConvexHullGenerationParameters generationParameters, CollisionFilter filter
        ) =>
            Create(points, generationParameters, filter, Material.Default);

        public static BlobAssetReference<Collider> Create(
            NativeArray<float3> points, ConvexHullGenerationParameters generationParameters, CollisionFilter filter, Material material
        ) =>
            Create(points, generationParameters, filter, material, k_MaxVertices, k_MaxFaces, k_MaxFaceVertices);

        internal static BlobAssetReference<Collider> Create(
            NativeArray<float3> points, ConvexHullGenerationParameters generationParameters, CollisionFilter filter, Material material,
            int maxVertices, int maxFaces, int maxFaceVertices
        )
        {
            SafetyChecks.CheckValidAndThrow(points, nameof(points), generationParameters, nameof(generationParameters));

            // Build convex hull
            var builder = new ConvexHullBuilder(
                points,
                generationParameters,
                maxVertices,
                maxFaces,
                maxFaceVertices,
                out var builderConvexRadius
            );

            return Create(builder, builderConvexRadius, filter, material);
        }

        internal static unsafe BlobAssetReference<Collider> Create(ConvexHullBuilder builder, sfloat convexRadius, CollisionFilter filter, Material material)
        {
            // Convert hull to compact format
            var tempHull = new TempHull(ref builder);

            // Allocate collider
            int totalSize = UnsafeUtility.SizeOf<ConvexCollider>();
            totalSize += tempHull.Vertices.Length * sizeof(float3);
            totalSize = Math.NextMultipleOf16(totalSize);  // planes currently must be aligned for Havok
            totalSize += tempHull.Planes.Length * sizeof(Plane);
            totalSize += tempHull.Faces.Length * sizeof(ConvexHull.Face);
            totalSize += tempHull.FaceVertexIndices.Length * sizeof(byte);
            totalSize += tempHull.VertexEdges.Length * sizeof(ConvexHull.Edge);
            totalSize += tempHull.FaceLinks.Length * sizeof(ConvexHull.Edge);
            ConvexCollider* collider = (ConvexCollider*)UnsafeUtility.Malloc(totalSize, 16, Allocator.Temp);

            // Initialize it
            {
                UnsafeUtility.MemClear(collider, totalSize);
                collider->MemorySize = totalSize;

                collider->m_Header.Type = ColliderType.Convex;
                collider->m_Header.CollisionType = CollisionType.Convex;
                collider->m_Header.Version = 1;
                collider->m_Header.Magic = 0xff;
                collider->m_Header.Filter = filter;
                collider->m_Header.Material = material;

                ref var hull = ref collider->ConvexHull;

                hull.ConvexRadius = convexRadius;

                // Initialize blob arrays
                {
                    byte* end = (byte*)collider + UnsafeUtility.SizeOf<ConvexCollider>();

                    hull.VerticesBlob.Offset = UnsafeEx.CalculateOffset(end, ref hull.VerticesBlob);
                    hull.VerticesBlob.Length = tempHull.Vertices.Length;
                    end += sizeof(float3) * tempHull.Vertices.Length;

                    end = (byte*)Math.NextMultipleOf16((ulong)end); // planes currently must be aligned for Havok

                    hull.FacePlanesBlob.Offset = UnsafeEx.CalculateOffset(end, ref hull.FacePlanesBlob);
                    hull.FacePlanesBlob.Length = tempHull.Planes.Length;
                    end += sizeof(Plane) * tempHull.Planes.Length;

                    hull.FacesBlob.Offset = UnsafeEx.CalculateOffset(end, ref hull.FacesBlob);
                    hull.FacesBlob.Length = tempHull.Faces.Length;
                    end += sizeof(ConvexHull.Face) * tempHull.Faces.Length;

                    hull.FaceVertexIndicesBlob.Offset = UnsafeEx.CalculateOffset(end, ref hull.FaceVertexIndicesBlob);
                    hull.FaceVertexIndicesBlob.Length = tempHull.FaceVertexIndices.Length;
                    end += sizeof(byte) * tempHull.FaceVertexIndices.Length;

                    hull.VertexEdgesBlob.Offset = UnsafeEx.CalculateOffset(end, ref hull.VertexEdgesBlob);
                    hull.VertexEdgesBlob.Length = tempHull.VertexEdges.Length;
                    end += sizeof(ConvexHull.Edge) * tempHull.VertexEdges.Length;

                    hull.FaceLinksBlob.Offset = UnsafeEx.CalculateOffset(end, ref hull.FaceLinksBlob);
                    hull.FaceLinksBlob.Length = tempHull.FaceLinks.Length;
                    end += sizeof(ConvexHull.Edge) * tempHull.FaceLinks.Length;
                }

                // Fill blob arrays
                {
                    for (int i = 0; i < tempHull.Vertices.Length; i++)
                    {
                        hull.Vertices[i] = tempHull.Vertices[i];
                        hull.VertexEdges[i] = tempHull.VertexEdges[i];
                    }

                    for (int i = 0; i < tempHull.Faces.Length; i++)
                    {
                        hull.Planes[i] = tempHull.Planes[i];
                        hull.Faces[i] = tempHull.Faces[i];
                    }

                    for (int i = 0; i < tempHull.FaceVertexIndices.Length; i++)
                    {
                        hull.FaceVertexIndices[i] = tempHull.FaceVertexIndices[i];
                        hull.FaceLinks[i] = tempHull.FaceLinks[i];
                    }
                }

                // Fill mass properties
                {
                    // Build the mass properties if they haven't been computed already.
                    if (builder.HullMassProperties.Volume.IsZero())
                    {
                        builder.UpdateHullMassProperties();
                    }

                    var massProperties = builder.HullMassProperties;
                    Math.DiagonalizeSymmetricApproximation(massProperties.InertiaTensor, out float3x3 orientation, out float3 inertia);

                    sfloat maxLengthSquared = sfloat.Zero;
                    for (int v = 0, count = hull.Vertices.Length; v < count; ++v)
                    {
                        maxLengthSquared = math.max(maxLengthSquared, math.lengthsq(hull.Vertices[v] - massProperties.CenterOfMass));
                    }

                    collider->MassProperties = new MassProperties
                    {
                        MassDistribution = new MassDistribution
                        {
                            Transform = new RigidTransform(orientation, massProperties.CenterOfMass),
                            InertiaTensor = inertia
                        },
                        Volume = massProperties.Volume,
                        AngularExpansionFactor = math.sqrt(maxLengthSquared)
                    };
                }
            }

            // Copy it into blob
            var asset = BlobAssetReference<Collider>.Create(collider, totalSize);

            UnsafeUtility.Free(collider, Allocator.Temp);
            return asset;
        }

        // Temporary hull of managed arrays, used during construction
        unsafe struct TempHull
        {
            public readonly NativeList<float3> Vertices;
            public readonly NativeList<Plane> Planes;
            public readonly NativeList<ConvexHull.Face> Faces;
            public readonly NativeList<byte> FaceVertexIndices;
            public readonly NativeList<ConvexHull.Edge> VertexEdges;
            public readonly NativeList<ConvexHull.Edge> FaceLinks;

            public TempHull(ref ConvexHullBuilder builder)
            {
                Vertices = new NativeList<float3>(builder.Vertices.PeakCount, Allocator.Temp);
                Faces = new NativeList<ConvexHull.Face>(builder.NumFaces, Allocator.Temp);
                Planes = new NativeList<Plane>(builder.NumFaces, Allocator.Temp);
                FaceVertexIndices = new NativeList<byte>(builder.NumFaceVertices, Allocator.Temp);
                VertexEdges = new NativeList<ConvexHull.Edge>(builder.Vertices.PeakCount, Allocator.Temp);
                FaceLinks = new NativeList<ConvexHull.Edge>(builder.NumFaceVertices, Allocator.Temp);

                // Copy the vertices
                var vertexIndexMap = new NativeArray<byte>(builder.Vertices.PeakCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
                foreach (int i in builder.Vertices.Indices)
                {
                    vertexIndexMap[i] = (byte)Vertices.Length;
                    Vertices.Add(builder.Vertices[i].Position);
                    VertexEdges.Add(new ConvexHull.Edge());  // filled below
                }

                // Copy the faces
                switch (builder.Dimension)
                {
                    case 3:
                    {
                        var edgeMap = new NativeHashMap<ConvexHull.Edge, ConvexHull.Edge>(builder.NumFaceVertices, Allocator.Temp);
                        for (ConvexHullBuilder.FaceEdge hullFace = builder.GetFirstFace(); hullFace.IsValid; hullFace = builder.GetNextFace(hullFace))
                        {
                            // Store the plane
                            ConvexHullBuilder.Edge firstEdge = hullFace;
                            Plane facePlane = builder.Planes[builder.Triangles[firstEdge.TriangleIndex].FaceIndex];
                            Planes.Add(facePlane);

                            // Walk the face's outer vertices & edges
                            short firstVertexIndex = (short)FaceVertexIndices.Length;
                            byte numEdges = 0;
                            sfloat maxCosAngle = -sfloat.One;
                            for (ConvexHullBuilder.FaceEdge edge = hullFace; edge.IsValid; edge = builder.GetNextFaceEdge(edge))
                            {
                                byte vertexIndex = vertexIndexMap[builder.StartVertex(edge)];
                                FaceVertexIndices.Add(vertexIndex);

                                var hullEdge = new ConvexHull.Edge { FaceIndex = (short)edge.Current.TriangleIndex, EdgeIndex = (byte)edge.Current.EdgeIndex }; // will be mapped to the output hull below
                                edgeMap.TryAdd(hullEdge, new ConvexHull.Edge { FaceIndex = (short)Faces.Length, EdgeIndex = numEdges });

                                VertexEdges[vertexIndex] = hullEdge;

                                ConvexHullBuilder.Edge linkedEdge = builder.GetLinkedEdge(edge);
                                FaceLinks.Add(new ConvexHull.Edge { FaceIndex = (short)linkedEdge.TriangleIndex, EdgeIndex = (byte)linkedEdge.EdgeIndex }); // will be mapped to the output hull below

                                ConvexHullBuilder.Triangle linkedTriangle = builder.Triangles[linkedEdge.TriangleIndex];
                                Plane linkedPlane = builder.Planes[linkedTriangle.FaceIndex];
                                maxCosAngle = math.max(maxCosAngle, math.dot(facePlane.Normal, linkedPlane.Normal));

                                numEdges++;
                            }
                            Assert.IsTrue(numEdges >= 3);

                            // Store the face
                            Faces.Add(new ConvexHull.Face
                            {
                                FirstIndex = firstVertexIndex,
                                NumVertices = numEdges,
                                MinHalfAngle = math.acos(maxCosAngle) * (sfloat)0.5f
                            });
                        }

                        // Remap the edges
                        {
                            for (int i = 0; i < VertexEdges.Length; i++)
                            {
                                edgeMap.TryGetValue(VertexEdges[i], out ConvexHull.Edge vertexEdge);
                                VertexEdges[i] = vertexEdge;
                            }

                            for (int i = 0; i < FaceLinks.Length; i++)
                            {
                                edgeMap.TryGetValue(FaceLinks[i], out ConvexHull.Edge faceLink);
                                FaceLinks[i] = faceLink;
                            }
                        }

                        break;
                    }

                    case 2:
                    {
                        // Make face vertices and edges
                        for (byte i = 0; i < Vertices.Length; i++)
                        {
                            FaceVertexIndices.Add(i);
                            VertexEdges.Add(new ConvexHull.Edge
                            {
                                FaceIndex = 0,
                                EdgeIndex = i
                            });
                            FaceLinks.Add(new ConvexHull.Edge
                            {
                                FaceIndex = 1,
                                EdgeIndex = (byte)(Vertices.Length - 1 - i)
                            });
                        }

                        for (byte i = 0; i < Vertices.Length; i++)
                        {
                            FaceVertexIndices.Add((byte)(Vertices.Length - 1 - i));
                            FaceLinks.Add(VertexEdges[i]);
                        }

                        // Make planes and faces
                        float3 normal;
                        {
                            float3 edge0 = Vertices[1] - Vertices[0];
                            float3 cross = float3.zero;
                            for (int i = 2; i < Vertices.Length; i++)
                            {
                                cross = math.cross(edge0, Vertices[i] - Vertices[0]);
                                if (math.lengthsq(cross) > sfloat.FromRaw(0x322bcc77)) // take the first cross product good enough to calculate a normal
                                {
                                    break;
                                }
                            }
                            normal = math.normalizesafe(cross, new float3(sfloat.One, sfloat.Zero, sfloat.Zero));
                        }
                        sfloat distance = math.dot(normal, Vertices[0]);
                        Planes.Add(new Plane(normal, -distance));
                        Planes.Add(Planes[0].Flipped);
                        Faces.Add(new ConvexHull.Face
                        {
                            FirstIndex = 0,
                            NumVertices = (byte)Vertices.Length,
                            MinHalfAngleCompressed = 255
                        });
                        Faces.Add(new ConvexHull.Face
                        {
                            FirstIndex = (byte)Vertices.Length,
                            NumVertices = (byte)Vertices.Length,
                            MinHalfAngleCompressed = 255
                        });

                        break;
                    }

                    default: break; // nothing to do for lower-dimensional hulls
                }
            }
        }

        #endregion

        #region IConvexCollider

        public ColliderType Type => m_Header.Type;
        public CollisionType CollisionType => m_Header.CollisionType;
        public int MemorySize { get; private set; }

        public CollisionFilter Filter { get => m_Header.Filter; set { if (!m_Header.Filter.Equals(value)) { m_Header.Version += 1; m_Header.Filter = value; } } }
        internal bool RespondsToCollision => m_Header.Material.CollisionResponse != CollisionResponsePolicy.None;
        public Material Material { get => m_Header.Material; set { if (!m_Header.Material.Equals(value)) { m_Header.Version += 1; m_Header.Material = value; } } }
        public MassProperties MassProperties { get; private set; }

        internal sfloat CalculateBoundingRadius(float3 pivot)
        {
            return ConvexHull.CalculateBoundingRadius(pivot);
        }

        public Aabb CalculateAabb()
        {
            return CalculateAabb(RigidTransform.identity);
        }

        public Aabb CalculateAabb(RigidTransform transform)
        {
            BlobArray.Accessor<float3> vertices = ConvexHull.Vertices;
            float3 min = math.rotate(transform, vertices[0]);
            float3 max = min;
            for (int i = 1; i < vertices.Length; ++i)
            {
                float3 v = math.rotate(transform, vertices[i]);
                min = math.min(min, v);
                max = math.max(max, v);
            }
            return new Aabb
            {
                Min = min + transform.pos - new float3(ConvexHull.ConvexRadius),
                Max = max + transform.pos + new float3(ConvexHull.ConvexRadius)
            };
        }

        // Cast a ray against this collider.
        public bool CastRay(RaycastInput input) => QueryWrappers.RayCast(ref this, input);
        public bool CastRay(RaycastInput input, out RaycastHit closestHit) => QueryWrappers.RayCast(ref this, input, out closestHit);
        public bool CastRay(RaycastInput input, ref NativeList<RaycastHit> allHits) => QueryWrappers.RayCast(ref this, input, ref allHits);
        public unsafe bool CastRay<T>(RaycastInput input, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            fixed (ConvexCollider* target = &this)
            {
                return RaycastQueries.RayCollider(input, (Collider*)target, ref collector);
            }
        }

        // Cast another collider against this one.
        public bool CastCollider(ColliderCastInput input) => QueryWrappers.ColliderCast(ref this, input);
        public bool CastCollider(ColliderCastInput input, out ColliderCastHit closestHit) => QueryWrappers.ColliderCast(ref this, input, out closestHit);
        public bool CastCollider(ColliderCastInput input, ref NativeList<ColliderCastHit> allHits) => QueryWrappers.ColliderCast(ref this, input, ref allHits);
        public unsafe bool CastCollider<T>(ColliderCastInput input, ref T collector) where T : struct, ICollector<ColliderCastHit>
        {
            fixed (ConvexCollider* target = &this)
            {
                return ColliderCastQueries.ColliderCollider(input, (Collider*)target, ref collector);
            }
        }

        // Calculate the distance from a point to this collider.
        public bool CalculateDistance(PointDistanceInput input) => QueryWrappers.CalculateDistance(ref this, input);
        public bool CalculateDistance(PointDistanceInput input, out DistanceHit closestHit) => QueryWrappers.CalculateDistance(ref this, input, out closestHit);
        public bool CalculateDistance(PointDistanceInput input, ref NativeList<DistanceHit> allHits) => QueryWrappers.CalculateDistance(ref this, input, ref allHits);
        public unsafe bool CalculateDistance<T>(PointDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            fixed (ConvexCollider* target = &this)
            {
                return DistanceQueries.PointCollider(input, (Collider*)target, ref collector);
            }
        }

        // Calculate the distance from another collider to this one.
        public bool CalculateDistance(ColliderDistanceInput input) => QueryWrappers.CalculateDistance(ref this, input);
        public bool CalculateDistance(ColliderDistanceInput input, out DistanceHit closestHit) => QueryWrappers.CalculateDistance(ref this, input, out closestHit);
        public bool CalculateDistance(ColliderDistanceInput input, ref NativeList<DistanceHit> allHits) => QueryWrappers.CalculateDistance(ref this, input, ref allHits);
        public unsafe bool CalculateDistance<T>(ColliderDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            fixed (ConvexCollider* target = &this)
            {
                return DistanceQueries.ColliderCollider(input, (Collider*)target, ref collector);
            }
        }

        #region GO API Queries

        // Interfaces that represent queries that exist in the GameObjects world.

        public bool CheckSphere(float3 position, sfloat radius, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CheckSphere(ref this, position, radius, filter, queryInteraction);
        public bool OverlapSphere(float3 position, sfloat radius, ref NativeList<DistanceHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.OverlapSphere(ref this, position, radius, ref outHits, filter, queryInteraction);
        public bool OverlapSphereCustom<T>(float3 position, sfloat radius, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<DistanceHit>
            => QueryWrappers.OverlapSphereCustom(ref this, position, radius, ref collector, filter, queryInteraction);

        public bool CheckCapsule(float3 point1, float3 point2, sfloat radius, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CheckCapsule(ref this, point1, point2, radius, filter, queryInteraction);
        public bool OverlapCapsule(float3 point1, float3 point2, sfloat radius, ref NativeList<DistanceHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.OverlapCapsule(ref this, point1, point2, radius, ref outHits, filter, queryInteraction);
        public bool OverlapCapsuleCustom<T>(float3 point1, float3 point2, sfloat radius, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<DistanceHit>
            => QueryWrappers.OverlapCapsuleCustom(ref this, point1, point2, radius, ref collector, filter, queryInteraction);

        public bool CheckBox(float3 center, quaternion orientation, float3 halfExtents, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CheckBox(ref this, center, orientation, halfExtents, filter, queryInteraction);
        public bool OverlapBox(float3 center, quaternion orientation, float3 halfExtents, ref NativeList<DistanceHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.OverlapBox(ref this, center, orientation, halfExtents, ref outHits, filter, queryInteraction);
        public bool OverlapBoxCustom<T>(float3 center, quaternion orientation, float3 halfExtents, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<DistanceHit>
            => QueryWrappers.OverlapBoxCustom(ref this, center, orientation, halfExtents, ref collector, filter, queryInteraction);

        public bool SphereCast(float3 origin, sfloat radius, float3 direction, sfloat maxDistance, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.SphereCast(ref this, origin, radius, direction, maxDistance, filter, queryInteraction);
        public bool SphereCast(float3 origin, sfloat radius, float3 direction, sfloat maxDistance, out ColliderCastHit hitInfo, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.SphereCast(ref this, origin, radius, direction, maxDistance, out hitInfo, filter, queryInteraction);
        public bool SphereCastAll(float3 origin, sfloat radius, float3 direction, sfloat maxDistance, ref NativeList<ColliderCastHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.SphereCastAll(ref this, origin, radius, direction, maxDistance, ref outHits, filter, queryInteraction);
        public bool SphereCastCustom<T>(float3 origin, sfloat radius, float3 direction, sfloat maxDistance, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<ColliderCastHit>
            => QueryWrappers.SphereCastCustom(ref this, origin, radius, direction, maxDistance, ref collector, filter, queryInteraction);

        public bool BoxCast(float3 center, quaternion orientation, float3 halfExtents, float3 direction, sfloat maxDistance, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.BoxCast(ref this, center, orientation, halfExtents, direction, maxDistance, filter, queryInteraction);
        public bool BoxCast(float3 center, quaternion orientation, float3 halfExtents, float3 direction, sfloat maxDistance, out ColliderCastHit hitInfo, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.BoxCast(ref this, center, orientation, halfExtents, direction, maxDistance, out hitInfo, filter, queryInteraction);
        public bool BoxCastAll(float3 center, quaternion orientation, float3 halfExtents, float3 direction, sfloat maxDistance, ref NativeList<ColliderCastHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.BoxCastAll(ref this, center, orientation, halfExtents, direction, maxDistance, ref outHits, filter, queryInteraction);
        public bool BoxCastCustom<T>(float3 center, quaternion orientation, float3 halfExtents, float3 direction, sfloat maxDistance, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<ColliderCastHit>
            => QueryWrappers.BoxCastCustom(ref this, center, orientation, halfExtents, direction, maxDistance, ref collector, filter, queryInteraction);

        public bool CapsuleCast(float3 point1, float3 point2, sfloat radius, float3 direction, sfloat maxDistance, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CapsuleCast(ref this, point1, point2, radius, direction, maxDistance, filter, queryInteraction);
        public bool CapsuleCast(float3 point1, float3 point2, sfloat radius, float3 direction, sfloat maxDistance, out ColliderCastHit hitInfo, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CapsuleCast(ref this, point1, point2, radius, direction, maxDistance, out hitInfo, filter, queryInteraction);
        public bool CapsuleCastAll(float3 point1, float3 point2, sfloat radius, float3 direction, sfloat maxDistance, ref NativeList<ColliderCastHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CapsuleCastAll(ref this, point1, point2, radius, direction, maxDistance, ref outHits, filter, queryInteraction);
        public bool CapsuleCastCustom<T>(float3 point1, float3 point2, sfloat radius, float3 direction, sfloat maxDistance, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<ColliderCastHit>
            => QueryWrappers.CapsuleCastCustom(ref this, point1, point2, radius, direction, maxDistance, ref collector, filter, queryInteraction);

        #endregion

        #endregion
    }
}
