using UnityEngine;

namespace UnityS.Mathematics
{
    public static class AABBExtensions
    {
        public static AABB ToAABB(this Bounds bounds)
        {
            return new AABB { Center = (float3)bounds.center, Extents = (float3)bounds.extents};
        }

        public static Bounds ToBounds(this AABB aabb)
        {
            return new Bounds { center = (Vector3)aabb.Center, extents = (Vector3)aabb.Extents};
        }
    }
}
