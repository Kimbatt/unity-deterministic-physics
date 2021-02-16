using System;
using System.Runtime.CompilerServices;
using UnityS.Mathematics;

namespace UnityS.Physics
{
    /// <summary>
    /// A target in the space of a rigid body that will align with a corresponding target in the space of the other body to which it is constrained.
    /// </summary>
    public struct BodyFrame : IEquatable<BodyFrame>
    {
        /// <summary>
        /// The bind pose anchor or target position of the joint in the space of its rigid body.
        /// </summary>
        public float3 Position;
        /// <summary>
        /// The bind pose orientation of the joint's x-axis in the space of its rigid body.
        /// </summary>
        public float3 Axis;
        /// <summary>
        /// The bind pose orientation of the joint's y-axis in the space of its rigid body.
        /// </summary>
        public float3 PerpendicularAxis;

        public BodyFrame(RigidTransform transform)
        {
            Position = transform.pos;
            var rotation = new float3x3(transform.rot);
            Axis = rotation.c0;
            PerpendicularAxis = rotation.c1;
        }

        static float3 k_DefaultAxis => new float3(sfloat.One, sfloat.Zero, sfloat.Zero);
        static float3 k_DefaultPerpendicular = new float3(sfloat.Zero, sfloat.One, sfloat.Zero);

        public static readonly BodyFrame Identity =
            new BodyFrame { Axis = k_DefaultAxis, PerpendicularAxis = k_DefaultPerpendicular };

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public RigidTransform AsRigidTransform() => new RigidTransform(ValidateAxes(), Position);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal Math.MTransform AsMTransform() => new Math.MTransform(ValidateAxes(), Position);

        // slower than math.orthonormalize(), this method replicates UnityEngine.Vector3.OrthoNormalize()
        // it is more robust if input Axis is not pre-normalized or frame is degenerate
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static float3x3 OrthoNormalize(float3 u, float3 v)
        {
            var mag = math.length(u);
            u = math.select(k_DefaultAxis, u / mag, mag > Math.Constants.UnityEpsilon);

            v -= math.dot(u, v) * u;
            mag = math.length(v);
            v = math.select(OrthoNormalVectorFast(u), v / mag, mag > Math.Constants.UnityEpsilon);

            return new float3x3(u, v, math.cross(u, v));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static float3 OrthoNormalVectorFast(float3 n)
        {
            sfloat kRcpSqrt2 = sfloat.FromRaw(0x3f3504f3);
            var usePlaneYZ = math.abs(n.z) > kRcpSqrt2;
            var a = math.select(math.dot(n.xy, n.xy), math.dot(n.yz, n.yz), usePlaneYZ);
            var k = math.rcp(math.sqrt(a));
            return math.select(new float3(-n.y * k, n.x * k, sfloat.Zero), new float3(sfloat.Zero, -n.z * k, n.y * k), usePlaneYZ);
        }

        internal float3x3 ValidateAxes()
        {
            // TODO: math.orthonormalize() does not guarantee an ortho-normalized result when Axis is non-normalized
            var sqrMag = math.lengthsq(Axis);
            sfloat kEpsilon = Math.Constants.UnityEpsilon;
            return sqrMag >= sfloat.One - kEpsilon && sqrMag <= sfloat.One + kEpsilon
                ? math.orthonormalize(new float3x3(Axis, PerpendicularAxis, default))
                : OrthoNormalize(Axis, PerpendicularAxis);
        }

        public bool Equals(BodyFrame other) =>
            Position.Equals(other.Position)
            && Axis.Equals(other.Axis)
            && PerpendicularAxis.Equals(other.PerpendicularAxis);

        public override bool Equals(object obj) => obj is BodyFrame other && Equals(other);

        public override int GetHashCode() => unchecked((int)math.hash(new float3x3(Position, Axis, PerpendicularAxis)));

        public override string ToString() =>
            $"BodyFrame {{ Axis = {Axis}, PerpendicularAxis = {PerpendicularAxis}, Position = {Position} }}";

        public static implicit operator BodyFrame (RigidTransform transform) => new BodyFrame(transform);
    }
}
