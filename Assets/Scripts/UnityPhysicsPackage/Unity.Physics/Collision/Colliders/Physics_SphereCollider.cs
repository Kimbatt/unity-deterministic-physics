using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using UnityS.Mathematics;

namespace UnityS.Physics
{
    public struct SphereGeometry : IEquatable<SphereGeometry>
    {
        // The center of the sphere
        public float3 Center { get => m_Center; set => m_Center = value; }
        float3 m_Center;

        // The radius of the sphere
        public sfloat Radius { get => m_Radius; set => m_Radius = value; }
        private sfloat m_Radius;

        public bool Equals(SphereGeometry other)
        {
            return m_Center.Equals(other.m_Center)
                && m_Radius.Equals(other.m_Radius);
        }

        public override int GetHashCode() => unchecked((int)math.hash(new float4(m_Center, m_Radius)));
    }

    // A collider in the shape of a sphere
    public struct SphereCollider : IConvexCollider
    {
        // Header
        private ConvexColliderHeader m_Header;
        internal ConvexHull ConvexHull;

        private float3 m_Vertex;

        public float3 Center => m_Vertex;
        public sfloat Radius => ConvexHull.ConvexRadius;

        public SphereGeometry Geometry
        {
            get => new SphereGeometry
            {
                Center = m_Vertex,
                Radius = ConvexHull.ConvexRadius
            };
            set
            {
                if (!value.Equals(Geometry))
                {
                    SetGeometry(value);
                }
            }
        }

        #region Construction

        public static BlobAssetReference<Collider> Create(SphereGeometry geometry) =>
            Create(geometry, CollisionFilter.Default, Material.Default);

        public static BlobAssetReference<Collider> Create(SphereGeometry geometry, CollisionFilter filter) =>
            Create(geometry, filter, Material.Default);

        public static unsafe BlobAssetReference<Collider> Create(SphereGeometry geometry, CollisionFilter filter, Material material)
        {
            var collider = default(SphereCollider);
            collider.Initialize(geometry, filter, material);
            return BlobAssetReference<Collider>.Create(&collider, sizeof(SphereCollider));
        }

        // Initializes the sphere collider, enables it to be created on stack.
        public void Initialize(SphereGeometry geometry, CollisionFilter filter, Material material)
        {
            m_Header.Type = ColliderType.Sphere;
            m_Header.CollisionType = CollisionType.Convex;
            m_Header.Version = 0;
            m_Header.Magic = 0xff;
            m_Header.Filter = filter;
            m_Header.Material = material;

            ConvexHull.VerticesBlob.Offset = UnsafeEx.CalculateOffset(ref m_Vertex, ref ConvexHull.VerticesBlob);
            ConvexHull.VerticesBlob.Length = 1;
            // note: no faces

            SetGeometry(geometry);
        }

        void SetGeometry(SphereGeometry geometry)
        {
            SafetyChecks.CheckValidAndThrow(geometry, nameof(geometry));

            m_Header.Version += 1;
            ConvexHull.ConvexRadius = geometry.Radius;
            m_Vertex = geometry.Center;
        }

        #endregion

        #region IConvexCollider

        public ColliderType Type => m_Header.Type;
        public CollisionType CollisionType => m_Header.CollisionType;
        public int MemorySize => UnsafeUtility.SizeOf<SphereCollider>();

        public CollisionFilter Filter { get => m_Header.Filter; set { if (!m_Header.Filter.Equals(value)) { m_Header.Version += 1; m_Header.Filter = value; } } }
        internal bool RespondsToCollision => m_Header.Material.CollisionResponse != CollisionResponsePolicy.None;
        public Material Material { get => m_Header.Material; set { if (!m_Header.Material.Equals(value)) { m_Header.Version += 1; m_Header.Material = value; } } }

        public MassProperties MassProperties => new MassProperties
        {
            MassDistribution = new MassDistribution
            {
                Transform = new RigidTransform(quaternion.identity, Center),
                InertiaTensor = new float3(sfloat.FromRaw(0x3ecccccd) * Radius * Radius)
            },
            Volume = sfloat.FromRaw(0x40860a92) * Radius * Radius * Radius,
            AngularExpansionFactor = sfloat.Zero
        };

        public Aabb CalculateAabb()
        {
            return new Aabb
            {
                Min = Center - new float3(Radius),
                Max = Center + new float3(Radius)
            };
        }

        public Aabb CalculateAabb(RigidTransform transform)
        {
            float3 centerInWorld = math.transform(transform, Center);
            return new Aabb
            {
                Min = centerInWorld - new float3(Radius),
                Max = centerInWorld + new float3(Radius)
            };
        }

        // Cast a ray against this collider.
        public bool CastRay(RaycastInput input) => QueryWrappers.RayCast(ref this, input);
        public bool CastRay(RaycastInput input, out RaycastHit closestHit) => QueryWrappers.RayCast(ref this, input, out closestHit);
        public bool CastRay(RaycastInput input, ref NativeList<RaycastHit> allHits) => QueryWrappers.RayCast(ref this, input, ref allHits);
        public unsafe bool CastRay<T>(RaycastInput input, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            fixed (SphereCollider* target = &this)
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
            fixed (SphereCollider* target = &this)
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
            fixed (SphereCollider* target = &this)
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
            fixed (SphereCollider* target = &this)
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
