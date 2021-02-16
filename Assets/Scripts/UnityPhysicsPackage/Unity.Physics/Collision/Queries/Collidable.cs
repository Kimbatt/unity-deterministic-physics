using Unity.Collections;
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
    }
}
