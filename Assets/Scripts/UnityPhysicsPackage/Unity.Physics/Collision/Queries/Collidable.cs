using Unity.Assertions;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityS.Mathematics;

namespace UnityS.Physics
{
    // Interface for objects that can be hit by physics queries.
    public interface ICollidable    // TODO: rename to Physics.IQueryable?
    {
        // Bounding box

        // Calculate an axis aligned bounding box around the object, in local space.
        Aabb CalculateAabb();

        // Calculate an axis aligned bounding box around the object, in the given space.
        Aabb CalculateAabb(RigidTransform transform);

        // Cast ray

        // Cast a ray against the object.
        // Return true if it hits.
        bool CastRay(RaycastInput input);

        // Cast a ray against the object.
        // Return true if it hits, with details of the closest hit in "closestHit".
        bool CastRay(RaycastInput input, out RaycastHit closestHit);

        // Cast a ray against the object.
        // Return true if it hits, with details of every hit in "allHits".
        bool CastRay(RaycastInput input, ref NativeList<RaycastHit> allHits);

        // Generic ray cast.
        // Return true if it hits, with details stored in the collector implementation.
        bool CastRay<T>(RaycastInput input, ref T collector) where T : struct, ICollector<RaycastHit>;

        // Cast collider

        // Cast a collider against the object.
        // Return true if it hits.
        bool CastCollider(ColliderCastInput input);

        // Cast a collider against the object.
        // Return true if it hits, with details of the closest hit in "closestHit".
        bool CastCollider(ColliderCastInput input, out ColliderCastHit closestHit);

        // Cast a collider against the object.
        // Return true if it hits, with details of every hit in "allHits".
        bool CastCollider(ColliderCastInput input, ref NativeList<ColliderCastHit> allHits);

        // Generic collider cast.
        // Return true if it hits, with details stored in the collector implementation.
        bool CastCollider<T>(ColliderCastInput input, ref T collector) where T : struct, ICollector<ColliderCastHit>;

        // Point distance query

        // Calculate the distance from a point to the object.
        // Return true if there are any hits.
        bool CalculateDistance(PointDistanceInput input);

        // Calculate the distance from a point to the object.
        // Return true if there are any hits, with details of the closest hit in "closestHit".
        bool CalculateDistance(PointDistanceInput input, out DistanceHit closestHit);

        // Calculate the distance from a point to the object.
        // Return true if there are any hits, with details of every hit in "allHits".
        bool CalculateDistance(PointDistanceInput input, ref NativeList<DistanceHit> allHits);

        // Calculate the distance from a point to the object.
        // Return true if there are any hits, with details stored in the collector implementation.
        bool CalculateDistance<T>(PointDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>;

        // Collider distance query

        // Calculate the distance from a collider to the object.
        // Return true if there are any hits.
        bool CalculateDistance(ColliderDistanceInput input);

        // Calculate the distance from a collider to the object.
        // Return true if there are any hits, with details of the closest hit in "closestHit".
        bool CalculateDistance(ColliderDistanceInput input, out DistanceHit closestHit);

        // Calculate the distance from a collider to the object.
        // Return true if there are any hits, with details of every hit in "allHits".
        bool CalculateDistance(ColliderDistanceInput input, ref NativeList<DistanceHit> allHits);

        // Calculate the distance from a collider to the object.
        // Return true if there are any hits, with details stored in the collector implementation.
        bool CalculateDistance<T>(ColliderDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>;

        // Interfaces that look like the GameObject query interfaces.

        // Checks if the provided sphere is overlapping with an ICollidable
        // Return true if it is overlapping
        bool CheckSphere(float3 position, sfloat radius, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default);

        // Checks if the provided sphere is overlapping with an ICollidable
        // Return true if there is at least one overlap, and all overlaps will be stored in provided list
        bool OverlapSphere(float3 position, sfloat radius, ref NativeList<DistanceHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default);

        // Checks if the provided sphere is overlapping with an ICollidable
        // Return true if there is at least one overlap, the passed collector is used for custom hit filtering if needed
        bool OverlapSphereCustom<T>(float3 position, sfloat radius, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<DistanceHit>;

        // Checks if the provided capsule is overlapping with an ICollidable
        // Return true if it is overlapping
        bool CheckCapsule(float3 point1, float3 point2, sfloat radius, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default);

        // Checks if the provided capsule is overlapping with an ICollidable
        // Return true if there is at least one overlap, and all overlaps will be stored in provided list
        bool OverlapCapsule(float3 point1, float3 point2, sfloat radius, ref NativeList<DistanceHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default);

        // Checks if the provided capsule is overlapping with an ICollidable
        // Return true if there is at least one overlap, the passed collector is used for custom hit filtering if needed
        bool OverlapCapsuleCustom<T>(float3 point1, float3 point2, sfloat radius, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<DistanceHit>;

        // Checks if the provided box is overlapping with an ICollidable
        // Return true if it is overlapping
        bool CheckBox(float3 center, quaternion orientation, float3 halfExtents, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default);

        // Checks if the provided box is overlapping with an ICollidable
        // Return true if there is at least one overlap, and all overlaps will be stored in provided list
        bool OverlapBox(float3 center, quaternion orientation, float3 halfExtents, ref NativeList<DistanceHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default);

        // Checks if the provided box is overlapping with an ICollidable
        // Return true if there is at least one overlap, the passed collector is used for custom hit filtering if needed
        bool OverlapBoxCustom<T>(float3 center, quaternion orientation, float3 halfExtents, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<DistanceHit>;

        // Casts a specified sphere along a ray specified with origin, direction, and maxDistance, and checks if it hits an ICollidable.
        // Return true if there is at least one hit.
        bool SphereCast(float3 origin, sfloat radius, float3 direction, sfloat maxDistance, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default);

        // Casts a specified sphere along a ray specified with origin, direction, and maxDistance, and checks if it hits an ICollidable.
        // Return true if a hit happened, the information about closest hit will be in hitInfo.
        bool SphereCast(float3 origin, sfloat radius, float3 direction, sfloat maxDistance, out ColliderCastHit hitInfo, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default);

        // Casts a specified sphere along a ray specified with origin, direction, and maxDistance, and checks if it hits an ICollidable.
        // Return true if at least one hit happened, all hits will be stored in a provided list.
        bool SphereCastAll(float3 origin, sfloat radius, float3 direction, sfloat maxDistance, ref NativeList<ColliderCastHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default);

        // Casts a specified sphere along a ray specified with origin, direction, and maxDistance, and checks if it hits an ICollidable.
        // Return true if at least one hit happened, the passed collector is used for custom hit filtering.
        bool SphereCastCustom<T>(float3 origin, sfloat radius, float3 direction, sfloat maxDistance, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<ColliderCastHit>;

        // Casts a specified box along a ray specified with center, direction, and maxDistance, and checks if it hits an ICollidable.
        // Return true if there is at least one hit.
        bool BoxCast(float3 center, quaternion orientation, float3 halfExtents, float3 direction, sfloat maxDistance, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default);

        // Casts a specified box along a ray specified with center, direction, and maxDistance, and checks if it hits an ICollidable.
        // Return true if a hit happened, the information about closest hit will be in hitInfo.
        bool BoxCast(float3 center, quaternion orientation, float3 halfExtents, float3 direction, sfloat maxDistance, out ColliderCastHit hitInfo, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default);

        // Casts a specified box along a ray specified with center, direction, and maxDistance, and checks if it hits an ICollidable.
        // Return true if at least one hit happened, all hits will be stored in a provided list.
        bool BoxCastAll(float3 center, quaternion orientation, float3 halfExtents, float3 direction, sfloat maxDistance, ref NativeList<ColliderCastHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default);

        // Casts a specified box along a ray specified with center, direction, and maxDistance, and checks if it hits an ICollidable.
        // Return true if at least one hit happened, the passed collector is used for custom hit filtering.
        bool BoxCastCustom<T>(float3 center, quaternion orientation, float3 halfExtents, float3 direction, sfloat maxDistance, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<ColliderCastHit>;

        // Casts a capsule specified with two points along a ray specified with the center of the capsule, direction and maxDistance, and checks if it hits an ICollidable.
        // Return true if there is at least one hit.
        bool CapsuleCast(float3 point1, float3 point2, sfloat radius, float3 direction, sfloat maxDistance, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default);

        // Casts a capsule specified with two points along a ray specified with the center of the capsule, direction and maxDistance, and checks if it hits an ICollidable.
        // Return true if a hit happened, the information about closest hit will be in hitInfo.
        bool CapsuleCast(float3 point1, float3 point2, sfloat radius, float3 direction, sfloat maxDistance, out ColliderCastHit hitInfo, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default);

        // Casts a capsule specified with two points along a ray specified with the center of the capsule, direction and maxDistance, and checks if it hits an ICollidable.
        // Return true if at least one hit happened, all hits will be stored in a provided list.
        bool CapsuleCastAll(float3 point1, float3 point2, sfloat radius, float3 direction, sfloat maxDistance, ref NativeList<ColliderCastHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default);

        // Casts a capsule specified with two points along a ray specified with the center of the capsule, direction and maxDistance, and checks if it hits an ICollidable.
        // Return true if at least one hit happened, the passed collector is used for custom hit filtering.
        bool CapsuleCastCustom<T>(float3 point1, float3 point2, sfloat radius, float3 direction, sfloat maxDistance, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<ColliderCastHit>;
    }

    // Used as a way to provide queries with more filtering options, without creating collectors
    // At the moment, IgnoreTriggers is the only option that is supported
    public enum QueryInteraction : byte
    {
        Default = 0,
        IgnoreTriggers = 1 << 0
    }

    // Wrappers around generic ICollidable queries
    static class QueryWrappers
    {
        #region Ray casts

        public static bool RayCast<T>(ref T target, RaycastInput input) where T : struct, ICollidable
        {
            var collector = new AnyHitCollector<RaycastHit>(sfloat.One);
            return target.CastRay(input, ref collector);
        }

        public static bool RayCast<T>(ref T target, RaycastInput input, out RaycastHit closestHit) where T : struct, ICollidable
        {
            var collector = new ClosestHitCollector<RaycastHit>(sfloat.One);
            if (target.CastRay(input, ref collector))
            {
                closestHit = collector.ClosestHit;  // TODO: would be nice to avoid this copy
                return true;
            }

            closestHit = new RaycastHit();
            return false;
        }

        public static bool RayCast<T>(ref T target, RaycastInput input, ref NativeList<RaycastHit> allHits) where T : struct, ICollidable
        {
            var collector = new AllHitsCollector<RaycastHit>(sfloat.One, ref allHits);
            return target.CastRay(input, ref collector);
        }

        #endregion

        #region Collider casts

        public static bool ColliderCast<T>(ref T target, ColliderCastInput input) where T : struct, ICollidable
        {
            var collector = new AnyHitCollector<ColliderCastHit>(sfloat.One);
            return target.CastCollider(input, ref collector);
        }

        public static bool ColliderCast<T>(ref T target, ColliderCastInput input, out ColliderCastHit result) where T : struct, ICollidable
        {
            var collector = new ClosestHitCollector<ColliderCastHit>(sfloat.One);
            if (target.CastCollider(input, ref collector))
            {
                result = collector.ClosestHit;  // TODO: would be nice to avoid this copy
                return true;
            }

            result = new ColliderCastHit();
            return false;
        }

        public static bool ColliderCast<T>(ref T target, ColliderCastInput input, ref NativeList<ColliderCastHit> allHits) where T : struct, ICollidable
        {
            var collector = new AllHitsCollector<ColliderCastHit>(sfloat.One, ref allHits);
            return target.CastCollider(input, ref collector);
        }

        #endregion

        #region Point distance queries

        public static bool CalculateDistance<T>(ref T target, PointDistanceInput input) where T : struct, ICollidable
        {
            var collector = new AnyHitCollector<DistanceHit>(input.MaxDistance);
            return target.CalculateDistance(input, ref collector);
        }

        public static bool CalculateDistance<T>(ref T target, PointDistanceInput input, out DistanceHit result) where T : struct, ICollidable
        {
            var collector = new ClosestHitCollector<DistanceHit>(input.MaxDistance);
            if (target.CalculateDistance(input, ref collector))
            {
                result = collector.ClosestHit;  // TODO: would be nice to avoid this copy
                return true;
            }

            result = new DistanceHit();
            return false;
        }

        public static bool CalculateDistance<T>(ref T target, PointDistanceInput input, ref NativeList<DistanceHit> allHits) where T : struct, ICollidable
        {
            var collector = new AllHitsCollector<DistanceHit>(input.MaxDistance, ref allHits);
            return target.CalculateDistance(input, ref collector);
        }

        #endregion

        #region Collider distance queries

        public static bool CalculateDistance<T>(ref T target, ColliderDistanceInput input) where T : struct, ICollidable
        {
            var collector = new AnyHitCollector<DistanceHit>(input.MaxDistance);
            return target.CalculateDistance(input, ref collector);
        }

        public static bool CalculateDistance<T>(ref T target, ColliderDistanceInput input, out DistanceHit result) where T : struct, ICollidable
        {
            var collector = new ClosestHitCollector<DistanceHit>(input.MaxDistance);
            if (target.CalculateDistance(input, ref collector))
            {
                result = collector.ClosestHit;  // TODO: would be nice to avoid this copy
                return true;
            }

            result = new DistanceHit();
            return false;
        }

        public static bool CalculateDistance<T>(ref T target, ColliderDistanceInput input, ref NativeList<DistanceHit> allHits) where T : struct, ICollidable
        {
            var collector = new AllHitsCollector<DistanceHit>(input.MaxDistance, ref allHits);
            return target.CalculateDistance(input, ref collector);
        }

        #endregion

        #region Existing GO API queries

        public static bool OverlapSphereCustom<T, C>(ref T target, float3 position, sfloat radius, ref C collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            where T : struct, ICollidable
            where C : struct, ICollector<DistanceHit>
        {
            PointDistanceInput input = new PointDistanceInput
            {
                Filter = filter,
                MaxDistance = radius,
                Position = position
            };

            if (queryInteraction == QueryInteraction.Default)
            {
                return target.CalculateDistance(input, ref collector);
            }
            else
            {
                unsafe
                {
                    var interactionCollector = new QueryInteractionCollector<DistanceHit, C>
                    {
                        Collector = collector
                    };

                    bool returnValue = target.CalculateDistance(input, ref interactionCollector);

                    collector = interactionCollector.Collector;
                    return returnValue;
                }
            }
        }

        public static bool OverlapSphere<T>(ref T target, float3 position, sfloat radius, ref NativeList<DistanceHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            where T : struct, ICollidable
        {
            var collector = new AllHitsCollector<DistanceHit>(radius, ref outHits);
            return target.OverlapSphereCustom(position, radius, ref collector, filter, queryInteraction);
        }

        public static bool CheckSphere<T>(ref T target, float3 position, sfloat radius, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            where T : struct, ICollidable
        {
            var collector = new AnyHitCollector<DistanceHit>(radius);
            return target.OverlapSphereCustom(position, radius, ref collector, filter, queryInteraction);
        }

        public static bool OverlapCapsuleCustom<T, C>(ref T target, float3 point1, float3 point2, sfloat radius, ref C collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            where T : struct, ICollidable
            where C : struct, ICollector<DistanceHit>
        {
            Assert.IsTrue(collector.MaxFraction == sfloat.Zero);

            CapsuleCollider collider = default;
            float3 center = (point1 + point2) / 2;

            CapsuleGeometry geometry = new CapsuleGeometry
            {
                Radius = radius,
                Vertex0 = point1 - center,
                Vertex1 = point2 - center
            };

            collider.Initialize(geometry, filter, Material.Default);
            ColliderDistanceInput input;
            unsafe
            {
                input = new ColliderDistanceInput
                {
                    Collider = (Collider*)UnsafeUtility.AddressOf(ref collider),
                    MaxDistance = sfloat.Zero,
                    Transform = new RigidTransform
                    {
                        pos = center,
                        rot = quaternion.identity
                    }
                };
            }

            if (queryInteraction == QueryInteraction.Default)
            {
                return target.CalculateDistance(input, ref collector);
            }
            else
            {
                unsafe
                {
                    var interactionCollector = new QueryInteractionCollector<DistanceHit, C>
                    {
                        Collector = collector,
                    };

                    bool returnValue = target.CalculateDistance(input, ref interactionCollector);

                    collector = interactionCollector.Collector;
                    return returnValue;
                }
            }
        }

        public static bool OverlapCapsule<T>(ref T target, float3 point1, float3 point2, sfloat radius, ref NativeList<DistanceHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            where T : struct, ICollidable
        {
            var collector = new AllHitsCollector<DistanceHit>(sfloat.Zero, ref outHits);
            return target.OverlapCapsuleCustom(point1, point2, radius, ref collector, filter, queryInteraction);
        }

        public static bool CheckCapsule<T>(ref T target, float3 point1, float3 point2, sfloat radius, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            where T : struct, ICollidable
        {
            var collector = new AnyHitCollector<DistanceHit>(sfloat.Zero);
            return target.OverlapCapsuleCustom(point1, point2, radius, ref collector, filter, queryInteraction);
        }

        public static bool OverlapBoxCustom<T, C>(ref T target, float3 center, quaternion orientation, float3 halfExtents, ref C collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            where T : struct, ICollidable
            where C : struct, ICollector<DistanceHit>
        {
            Assert.IsTrue(collector.MaxFraction == sfloat.Zero);

            BoxCollider collider = default;
            BoxGeometry geometry = new BoxGeometry
            {
                BevelRadius = sfloat.Zero,
                Center = float3.zero,
                Size = halfExtents * 2,
                Orientation = quaternion.identity
            };

            collider.Initialize(geometry, filter, Material.Default);

            ColliderDistanceInput input;
            unsafe
            {
                input = new ColliderDistanceInput
                {
                    Collider = (Collider*)UnsafeUtility.AddressOf(ref collider),
                    MaxDistance = sfloat.Zero,
                    Transform = new RigidTransform
                    {
                        pos = center,
                        rot = orientation
                    }
                };
            }

            if (queryInteraction == QueryInteraction.Default)
            {
                return target.CalculateDistance(input, ref collector);
            }
            else
            {
                unsafe
                {
                    var interactionCollector = new QueryInteractionCollector<DistanceHit, C>
                    {
                        Collector = collector,
                    };

                    bool returnValue = target.CalculateDistance(input, ref interactionCollector);

                    collector = interactionCollector.Collector;
                    return returnValue;
                }
            }
        }

        public static bool OverlapBox<T>(ref T target, float3 center, quaternion orientation, float3 halfExtents, ref NativeList<DistanceHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            where T : struct, ICollidable
        {
            var collector = new AllHitsCollector<DistanceHit>(sfloat.Zero, ref outHits);
            return target.OverlapBoxCustom(center, orientation, halfExtents, ref collector, filter, queryInteraction);
        }

        public static bool CheckBox<T>(ref T target, float3 center, quaternion orientation, float3 halfExtents, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            where T : struct, ICollidable
        {
            var collector = new AnyHitCollector<DistanceHit>(sfloat.Zero);
            return target.OverlapBoxCustom(center, orientation, halfExtents, ref collector, filter, queryInteraction);
        }

        public static bool SphereCastCustom<T, C>(ref T target, float3 origin, sfloat radius, float3 direction, sfloat maxDistance, ref C collector, CollisionFilter filter ,  QueryInteraction queryInteraction = QueryInteraction.Default)
            where T : struct, ICollidable
            where C : struct, ICollector<ColliderCastHit>
        {
            SphereCollider collider = default;
            SphereGeometry geometry = new SphereGeometry
            {
                Center = 0,
                Radius = radius
            };

            collider.Initialize(geometry, filter, Material.Default);

            ColliderCastInput input;
            unsafe
            {
                input = new ColliderCastInput
                {
                    Collider = (Collider*)UnsafeUtility.AddressOf(ref collider),
                    Orientation = quaternion.identity,
                    Start = origin,
                    End = origin + direction * maxDistance
                };
            }

            if (queryInteraction == QueryInteraction.Default)
            {
                return target.CastCollider(input, ref collector);
            }
            else
            {
                unsafe
                {
                    var interactionCollector = new QueryInteractionCollector<ColliderCastHit, C>
                    {
                        Collector = collector
                    };

                    bool returnValue =  target.CastCollider(input, ref interactionCollector);

                    collector = interactionCollector.Collector;
                    return returnValue;
                }
            }
        }

        public static bool SphereCastAll<T>(ref T target, float3 origin, sfloat radius, float3 direction, sfloat maxDistance, ref NativeList<ColliderCastHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            where T : struct, ICollidable
        {
            var collector = new AllHitsCollector<ColliderCastHit>(sfloat.One, ref outHits);
            return target.SphereCastCustom(origin, radius, direction, maxDistance, ref collector, filter, queryInteraction);
        }

        public static bool SphereCast<T>(ref T target, float3 origin, sfloat radius, float3 direction, sfloat maxDistance, out ColliderCastHit hitInfo, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            where T : struct, ICollidable
        {
            var collector = new ClosestHitCollector<ColliderCastHit>(sfloat.One);
            bool hasHit = target.SphereCastCustom(origin, radius, direction, maxDistance, ref collector, filter, queryInteraction);
            hitInfo = collector.ClosestHit;

            return hasHit;
        }

        public static bool SphereCast<T>(ref T target, float3 origin, sfloat radius, float3 direction, sfloat maxDistance, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            where T : struct, ICollidable
        {
            var collector = new AnyHitCollector<ColliderCastHit>(sfloat.One);
            return target.SphereCastCustom(origin, radius, direction, maxDistance, ref collector, filter, queryInteraction);
        }

        public static bool BoxCastCustom<T, C>(ref T target, float3 center, quaternion orientation, float3 halfExtents, float3 direction, sfloat maxDistance, ref C collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            where T : struct, ICollidable
            where C : struct, ICollector<ColliderCastHit>
        {
            BoxCollider collider = default;
            BoxGeometry boxGeometry = new BoxGeometry
            {
                BevelRadius = sfloat.Zero,
                Center = 0,
                Orientation = quaternion.identity,
                Size = halfExtents * 2
            };

            collider.Initialize(boxGeometry, filter, Material.Default);

            ColliderCastInput input;
            unsafe
            {
                input = new ColliderCastInput
                {
                    Collider = (Collider*)UnsafeUtility.AddressOf(ref collider),
                    Orientation = orientation,
                    Start = center,
                    End = center + direction * maxDistance
                };
            }

            if (queryInteraction == QueryInteraction.Default)
            {
                return target.CastCollider(input, ref collector);
            }
            else
            {
                unsafe
                {
                    var interactionCollector = new QueryInteractionCollector<ColliderCastHit, C>
                    {
                        Collector = collector
                    };

                    bool returnValue = target.CastCollider(input, ref interactionCollector);

                    collector = interactionCollector.Collector;
                    return returnValue;
                }
            }
        }

        public static bool BoxCastAll<T>(ref T target, float3 center, quaternion orientation, float3 halfExtents, float3 direction, sfloat maxDistance, ref NativeList<ColliderCastHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            where T : struct, ICollidable
        {
            var collector = new AllHitsCollector<ColliderCastHit>(sfloat.One, ref outHits);
            return target.BoxCastCustom(center, orientation, halfExtents, direction, maxDistance, ref collector, filter, queryInteraction);
        }

        public static bool BoxCast<T>(ref T target, float3 center, quaternion orientation, float3 halfExtents, float3 direction, sfloat maxDistance, out ColliderCastHit hitInfo, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            where T : struct, ICollidable
        {
            var collector = new ClosestHitCollector<ColliderCastHit>(sfloat.One);
            bool hasHit = target.BoxCastCustom(center, orientation, halfExtents, direction, maxDistance, ref collector, filter, queryInteraction);
            hitInfo = collector.ClosestHit;
            return hasHit;
        }

        public static bool BoxCast<T>(ref T target, float3 center, quaternion orientation, float3 halfExtents, float3 direction, sfloat maxDistance, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            where T : struct, ICollidable
        {
            var collector = new AnyHitCollector<ColliderCastHit>(sfloat.One);
            return target.BoxCastCustom(center, orientation, halfExtents, direction, maxDistance, ref collector, filter, queryInteraction);
        }

        public static bool CapsuleCastCustom<T, C>(ref T target, float3 point1, float3 point2, sfloat radius, float3 direction, sfloat maxDistance, ref C collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            where T : struct, ICollidable
            where C : struct, ICollector<ColliderCastHit>
        {
            CapsuleCollider collider = default;

            float3 center = (point1 + point2) / 2;

            CapsuleGeometry geometry = new CapsuleGeometry
            {
                Radius = radius,
                Vertex0 = point1 - center,
                Vertex1 = point2 - center
            };

            collider.Initialize(geometry, filter, Material.Default);

            ColliderCastInput input;
            unsafe
            {
                input = new ColliderCastInput
                {
                    Collider = (Collider*)UnsafeUtility.AddressOf(ref collider),
                    Orientation = quaternion.identity,
                    Start = center,
                    End = center + direction * maxDistance
                };
            }

            if (queryInteraction == QueryInteraction.Default)
            {
                return target.CastCollider(input, ref collector);
            }
            else
            {
                unsafe
                {
                    var interactionCollector = new QueryInteractionCollector<ColliderCastHit, C>
                    {
                        Collector = collector
                    };

                    bool returnValue = target.CastCollider(input, ref interactionCollector);

                    collector = interactionCollector.Collector;
                    return returnValue;
                }
            }
        }

        public static bool CapsuleCastAll<T>(ref T target, float3 point1, float3 point2, sfloat radius, float3 direction, sfloat maxDistance, ref NativeList<ColliderCastHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            where T : struct, ICollidable
        {
            var collector = new AllHitsCollector<ColliderCastHit>(sfloat.One, ref outHits);
            return target.CapsuleCastCustom(point1, point2, radius, direction, maxDistance, ref collector, filter, queryInteraction);
        }

        public static bool CapsuleCast<T>(ref T target, float3 point1, float3 point2, sfloat radius, float3 direction, sfloat maxDistance, out ColliderCastHit hitInfo, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            where T : struct, ICollidable
        {
            var collector = new ClosestHitCollector<ColliderCastHit>(sfloat.One);
            bool hasHit = target.CapsuleCastCustom(point1, point2, radius, direction, maxDistance, ref collector, filter, queryInteraction);

            hitInfo = collector.ClosestHit;
            return hasHit;
        }

        public static bool CapsuleCast<T>(ref T target, float3 point1, float3 point2, sfloat radius, float3 direction, sfloat maxDistance, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            where T : struct, ICollidable
        {
            var collector = new AnyHitCollector<ColliderCastHit>(sfloat.One);
            return target.CapsuleCastCustom(point1, point2, radius, direction, maxDistance, ref collector, filter, queryInteraction);
        }

        #endregion
    }
}
