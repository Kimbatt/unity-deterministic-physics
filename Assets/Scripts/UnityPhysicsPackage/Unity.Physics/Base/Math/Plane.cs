using System.Runtime.CompilerServices;
using System.Diagnostics;
using UnityS.Mathematics;

namespace UnityS.Physics
{
    // A plane described by a normal and a distance from the origin
    [DebuggerDisplay("{Normal}, {Distance}")]
    public struct Plane
    {
        private float4 m_NormalAndDistance;

        public float3 Normal
        {
            get => m_NormalAndDistance.xyz;
            set => m_NormalAndDistance.xyz = value;
        }

        public sfloat Distance
        {
            get => m_NormalAndDistance.w;
            set => m_NormalAndDistance.w = value;
        }

        // Returns the distance from the point to the plane, positive if the point is on the side of
        // the plane on which the plane normal points, zero if the point is on the plane, negative otherwise.
        public sfloat SignedDistanceToPoint(float3 point)
        {
            return Math.Dotxyz1(m_NormalAndDistance, point);
        }

        // Returns the closest point on the plane to the input point.
        public float3 Projection(float3 point)
        {
            return point - Normal * SignedDistanceToPoint(point);
        }

        public Plane Flipped => new Plane { m_NormalAndDistance = -m_NormalAndDistance };

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Plane(float3 normal, sfloat distance)
        {
            m_NormalAndDistance = new float4(normal, distance);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator float4(Plane plane) => plane.m_NormalAndDistance;
    }

    // Helper functions
    public static partial class Math
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Plane PlaneFromDirection(float3 origin, float3 direction)
        {
            float3 normal = math.normalize(direction);
            return new Plane(normal, -math.dot(normal, origin));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Plane PlaneFromTwoEdges(float3 origin, float3 edgeA, float3 edgeB)
        {
            return PlaneFromDirection(origin, math.cross(edgeA, edgeB));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Plane TransformPlane(RigidTransform transform, Plane plane)
        {
            float3 normal = math.rotate(transform.rot, plane.Normal);
            return new Plane(normal, plane.Distance - math.dot(normal, transform.pos));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Plane TransformPlane(MTransform transform, Plane plane)
        {
            float3 normal = math.mul(transform.Rotation, plane.Normal);
            return new Plane(normal, plane.Distance - math.dot(normal, transform.Translation));
        }
    }
}
