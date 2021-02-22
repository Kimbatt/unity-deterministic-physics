using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityS.Mathematics;
using Unity.Entities;
using Unity.Burst;

namespace UnityS.Physics
{
    // A collider representing a mesh comprised of triangles and quads.
    // Warning: This is just the header, it is followed by variable sized data in memory.
    // Therefore this struct must always be passed by reference, never by value.
    public struct MeshCollider : ICompositeCollider
    {
        ColliderHeader m_Header;
        Aabb m_Aabb;
        internal Mesh Mesh;

        // followed by variable sized mesh data

        #region Construction

        // Create a mesh collider asset from a set of triangles
        public static BlobAssetReference<Collider> Create(NativeArray<float3> vertices, NativeArray<int3> triangles) =>
            Create(vertices, triangles, CollisionFilter.Default, Material.Default);

        public static BlobAssetReference<Collider> Create(NativeArray<float3> vertices, NativeArray<int3> triangles, CollisionFilter filter) =>
            Create(vertices, triangles, filter, Material.Default);

        public static unsafe BlobAssetReference<Collider> Create(NativeArray<float3> vertices, NativeArray<int3> triangles, CollisionFilter filter, Material material)
        {
            SafetyChecks.CheckTriangleIndicesInRangeAndThrow(triangles, vertices.Length, nameof(triangles));

            // Copy vertices
            var tempVertices = new NativeArray<float3>(vertices, Allocator.Temp);

            // Triangle indices - needed for WeldVertices
            var tempIndices = new NativeArray<int>(triangles.Reinterpret<int>(UnsafeUtility.SizeOf<int3>()), Allocator.Temp);

            // Build connectivity and primitives

            NativeList<float3> uniqueVertices = MeshConnectivityBuilder.WeldVertices(tempIndices, tempVertices);

            var tempTriangleIndices = new NativeArray<int3>(triangles.Length, Allocator.Temp);
            UnsafeUtility.MemCpy(tempTriangleIndices.GetUnsafePtr(), tempIndices.GetUnsafePtr(), tempIndices.Length * UnsafeUtility.SizeOf<int>());

            var connectivity = new MeshConnectivityBuilder(tempTriangleIndices, uniqueVertices);
            NativeList<MeshConnectivityBuilder.Primitive> primitives = connectivity.EnumerateQuadDominantGeometry(tempTriangleIndices, uniqueVertices);

            // Build bounding volume hierarchy
            int nodeCount = math.max(primitives.Length * 2 + 1, 2); // We need at least two nodes - an "invalid" node and a root node.
            var nodes = new NativeArray<BoundingVolumeHierarchy.Node>(nodeCount, Allocator.Temp);
            int numNodes = 0;

            {
                // Prepare data for BVH
                var points = new NativeList<BoundingVolumeHierarchy.PointAndIndex>(primitives.Length, Allocator.Temp);
                var aabbs = new NativeArray<Aabb>(primitives.Length, Allocator.Temp);

                for (int i = 0; i < primitives.Length; i++)
                {
                    MeshConnectivityBuilder.Primitive p = primitives[i];

                    // Skip degenerate triangles
                    if (MeshConnectivityBuilder.IsTriangleDegenerate(p.Vertices[0], p.Vertices[1], p.Vertices[2]))
                    {
                        continue;
                    }

                    aabbs[i] = Aabb.CreateFromPoints(p.Vertices);
                    points.Add(new BoundingVolumeHierarchy.PointAndIndex
                    {
                        Position = aabbs[i].Center,
                        Index = i
                    });
                }

                var bvh = new BoundingVolumeHierarchy(nodes);

                bvh.Build(points.AsArray(), aabbs, out numNodes, useSah: true);
            }

            // Build mesh sections
            BoundingVolumeHierarchy.Node* nodesPtr = (BoundingVolumeHierarchy.Node*)nodes.GetUnsafePtr();
            MeshBuilder.TempSection sections = MeshBuilder.BuildSections(nodesPtr, numNodes, primitives);

            // Allocate collider
            int meshDataSize = Mesh.CalculateMeshDataSize(numNodes, sections.Ranges);
            int totalColliderSize = Math.NextMultipleOf(sizeof(MeshCollider), 16) + meshDataSize;

            MeshCollider* meshCollider = (MeshCollider*)UnsafeUtility.Malloc(totalColliderSize, 16, Allocator.Temp);

            // Initialize it
            {
                UnsafeUtility.MemClear(meshCollider, totalColliderSize);
                meshCollider->MemorySize = totalColliderSize;

                meshCollider->m_Header.Type = ColliderType.Mesh;
                meshCollider->m_Header.CollisionType = CollisionType.Composite;
                meshCollider->m_Header.Version += 1;
                meshCollider->m_Header.Magic = 0xff;

                ref var mesh = ref meshCollider->Mesh;

                mesh.Init(nodesPtr, numNodes, sections, filter, material);
                mesh.UpdateCachedBoundingRadius();

                // Calculate combined filter
                meshCollider->m_Header.Filter = mesh.Sections.Length > 0 ? mesh.Sections[0].Filters[0] : CollisionFilter.Default;
                for (int i = 0; i < mesh.Sections.Length; ++i)
                {
                    for (var j = 0; j < mesh.Sections[i].Filters.Length; ++j)
                    {
                        var f = mesh.Sections[i].Filters[j];
                        meshCollider->m_Header.Filter = CollisionFilter.CreateUnion(meshCollider->m_Header.Filter, f);
                    }
                }

                meshCollider->m_Aabb = meshCollider->Mesh.BoundingVolumeHierarchy.Domain;
                meshCollider->NumColliderKeyBits = meshCollider->Mesh.NumColliderKeyBits;
            }

            // Copy collider into blob
            var blob = BlobAssetReference<Collider>.Create(meshCollider, totalColliderSize);
            UnsafeUtility.Free(meshCollider, Allocator.Temp);
            return blob;
        }

        #endregion

        #region ICompositeCollider

        public ColliderType Type => m_Header.Type;
        public CollisionType CollisionType => m_Header.CollisionType;
        public int MemorySize { get; private set; }

        public CollisionFilter Filter
        {
            get => m_Header.Filter;
            set
            {
                m_Header.Version += 1;
                m_Header.Filter = value;

                for (int i = 0; i < Mesh.Sections.Length; i++)
                {
                    var filters = Mesh.Sections[i].Filters;
                    for (var j = 0; j < filters.Length; j++)
                    {
                        filters[j] = value;
                    }
                }
            }
        }

        internal bool RespondsToCollision
        {
            get
            {
                for (int i = 0; i < Mesh.Sections.Length; i++)
                {
                    var materials = Mesh.Sections[i].Materials;
                    for (var j = 0; j < materials.Length; j++)
                    {
                        if (materials[j].CollisionResponse != CollisionResponsePolicy.None)
                        {
                            return true;
                        }
                    }
                }
                return false;
            }
        }

        // Mass properties are calculated based on the AABB of the mesh.
        // This is a rough approximation, but only makes a difference
        // in the case of dynamic meshes, which shouldn't be used a lot
        // for performance reasons.
        public MassProperties MassProperties
        {
            get
            {
                // Rough approximation based on AABB
                float3 size = m_Aabb.Extents;
                return new MassProperties
                {
                    MassDistribution = new MassDistribution
                    {
                        Transform = new RigidTransform(quaternion.identity, m_Aabb.Center),
                        InertiaTensor = new float3(
                            (size.y * size.y + size.z * size.z) * sfloat.FromRaw(0x3daaaaab),
                            (size.x * size.x + size.z * size.z) * sfloat.FromRaw(0x3daaaaab),
                            (size.x * size.x + size.y * size.y) * sfloat.FromRaw(0x3daaaaab))
                    },
                    Volume = size.x * size.y * size.z,
                    AngularExpansionFactor = math.length(m_Aabb.Extents) * (sfloat)0.5f
                };
            }
        }

        internal sfloat CalculateBoundingRadius(float3 pivot)
        {
            return math.distance(pivot, Mesh.BoundingVolumeHierarchy.Domain.Center) + Mesh.m_BoundingRadius;
        }

        public Aabb CalculateAabb()
        {
            return CalculateAabb(RigidTransform.identity);
        }

        public Aabb CalculateAabb(RigidTransform transform)
        {
            var outAabb = Math.TransformAabb(transform, m_Aabb);
            float3 center = outAabb.Center;
            Aabb sphereAabb = new Aabb
            {
                Min = new float3(center - Mesh.m_BoundingRadius),
                Max = new float3(center + Mesh.m_BoundingRadius)
            };
            outAabb.Intersect(sphereAabb);

            return outAabb;
        }

        // Cast a ray against this collider.
        public bool CastRay(RaycastInput input) => QueryWrappers.RayCast(ref this, input);
        public bool CastRay(RaycastInput input, out RaycastHit closestHit) => QueryWrappers.RayCast(ref this, input, out closestHit);
        public bool CastRay(RaycastInput input, ref NativeList<RaycastHit> allHits) => QueryWrappers.RayCast(ref this, input, ref allHits);
        public unsafe bool CastRay<T>(RaycastInput input, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            fixed (MeshCollider* target = &this)
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
            fixed (MeshCollider* target = &this)
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
            fixed (MeshCollider* target = &this)
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
            fixed (MeshCollider* target = &this)
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

        public uint NumColliderKeyBits { get; private set; }

        internal uint TotalNumColliderKeyBits => NumColliderKeyBits;

        public bool GetChild(ref ColliderKey key, out ChildCollider child)
        {
            if (key.PopSubKey(NumColliderKeyBits, out uint subKey))
            {
                int primitiveKey = (int)(subKey >> 1);
                int polygonIndex = (int)(subKey & 1);

                Mesh.GetPrimitive(primitiveKey, out float3x4 vertices, out Mesh.PrimitiveFlags flags, out CollisionFilter filter, out Material material);

                if (Mesh.IsPrimitiveFlagSet(flags, Mesh.PrimitiveFlags.IsQuad))
                {
                    child = new ChildCollider(vertices[0], vertices[1], vertices[2], vertices[3], filter, material);
                }
                else
                {
                    child = new ChildCollider(vertices[0], vertices[1 + polygonIndex], vertices[2 + polygonIndex], filter, material);
                }

                return true;
            }

            child = new ChildCollider();
            return false;
        }

        public bool GetLeaf(ColliderKey key, out ChildCollider leaf)
        {
            return GetChild(ref key, out leaf);
        }

        public unsafe void GetLeaves<T>([NoAlias] ref T collector) where T : struct, ILeafColliderCollector
        {
            var polygon = new PolygonCollider();
            polygon.InitNoVertices(CollisionFilter.Default, Material.Default);
            if (Mesh.GetFirstPolygon(out uint meshKey, ref polygon))
            {
                do
                {
                    var leaf = new ChildCollider((Collider*)&polygon, RigidTransform.identity);
                    collector.AddLeaf(new ColliderKey(NumColliderKeyBits, meshKey), ref leaf);
                }
                while (Mesh.GetNextPolygon(meshKey, out meshKey, ref polygon));
            }
        }

        #endregion
    }
}
