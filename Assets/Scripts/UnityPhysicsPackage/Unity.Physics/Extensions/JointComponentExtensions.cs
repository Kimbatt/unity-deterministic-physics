using UnityS.Mathematics;
using FloatRange = UnityS.Physics.Math.FloatRange;

namespace UnityS.Physics.Extensions
{
    /// <summary>
    /// Extension methods for working with <see cref="PhysicsJoint"/>.
    /// </summary>
    public static class JointComponentExtensions
    {
        #region LimitedDistance

        /// <summary>
        /// Gets the range of motion for a <see cref="PhysicsJoint"/> created using <see cref="PhysicsJoint.CreateLimitedDistance"/>.
        /// </summary>
        /// <returns>The minimum required distance and maximum possible distance between the two anchor points.</returns>
        public static FloatRange GetLimitedDistanceRange(in this PhysicsJoint joint) =>
            new FloatRange
            {
                Min = joint[PhysicsJoint.k_LimitedDistanceRangeIndex].Min,
                Max = joint[PhysicsJoint.k_LimitedDistanceRangeIndex].Max
            };

        /// <summary>
        /// Applies the specified range of motion to a <see cref="PhysicsJoint"/> created using <see cref="PhysicsJoint.CreateLimitedDistance"/>.
        /// </summary>
        /// <param name="distanceRange">The minimum required distance and maximum possible distance between the two anchor points.</param>
        public static void SetLimitedDistanceRange(ref this PhysicsJoint joint, FloatRange distanceRange)
        {
            var constraints = joint.GetConstraints();
            constraints.ElementAt(PhysicsJoint.k_LimitedDistanceRangeIndex).Min = distanceRange.Min;
            constraints.ElementAt(PhysicsJoint.k_LimitedDistanceRangeIndex).Max = distanceRange.Max;
            joint.SetConstraints(constraints);
        }

        #endregion

        #region LimitedHinge

        /// <summary>
        /// Gets the range of motion for a <see cref="PhysicsJoint"/> created using <see cref="PhysicsJoint.CreateLimitedHinge"/>.
        /// </summary>
        /// <returns>The minimum required and maximum possible angle of rotation about the aligned axes.</returns>
        public static FloatRange GetLimitedHingeRange(in this PhysicsJoint joint) =>
            new FloatRange
            {
                Min = joint[PhysicsJoint.k_LimitedHingeRangeIndex].Min,
                Max = joint[PhysicsJoint.k_LimitedHingeRangeIndex].Max
            };

        /// <summary>
        /// Applies the specified range of motion to a <see cref="PhysicsJoint"/> created using <see cref="PhysicsJoint.CreateLimitedHinge"/>.
        /// </summary>
        /// <param name="angularRange">The minimum required and maximum possible angle of rotation about the aligned axes.</param>
        public static void SetLimitedHingeRange(ref this PhysicsJoint joint, FloatRange angularRange)
        {
            var constraints = joint.GetConstraints();
            constraints.ElementAt(PhysicsJoint.k_LimitedHingeRangeIndex).Min = angularRange.Min;
            constraints.ElementAt(PhysicsJoint.k_LimitedHingeRangeIndex).Max = angularRange.Max;
            joint.SetConstraints(constraints);
        }

        #endregion

        #region Prismatic

        /// <summary>
        /// Gets the range of motion for a <see cref="PhysicsJoint"/> created using <see cref="PhysicsJoint.CreatePrismatic"/>.
        /// </summary>
        /// <returns>The minimum required and maximum possible distance between the two anchor points along their aligned axes.</returns>
        public static FloatRange GetPrismaticRange(in this PhysicsJoint joint) =>
            new FloatRange
            {
                Min = joint[PhysicsJoint.k_PrismaticDistanceOnAxisIndex].Min,
                Max = joint[PhysicsJoint.k_PrismaticDistanceOnAxisIndex].Max
            };

        /// <summary>
        /// Applies the specified range of motion to a <see cref="PhysicsJoint"/> created using <see cref="PhysicsJoint.CreatePrismatic"/>.
        /// </summary>
        /// <param name="distanceOnAxis">The minimum required and maximum possible distance between the two anchor points along their aligned axes.</param>
        public static void SetPrismaticRange(ref this PhysicsJoint joint, FloatRange distanceOnAxis)
        {
            var constraints = joint.GetConstraints();
            constraints.ElementAt(PhysicsJoint.k_PrismaticDistanceOnAxisIndex).Min = distanceOnAxis.Min;
            constraints.ElementAt(PhysicsJoint.k_PrismaticDistanceOnAxisIndex).Max = distanceOnAxis.Max;
            joint.SetConstraints(constraints);
        }

        #endregion

        #region Ragdoll

        /// <summary>
        /// Gets the range of motion for a ragdoll primary cone <see cref="PhysicsJoint"/> created using <see cref="PhysicsJoint.CreateRagdoll"/>.
        /// </summary>
        /// <param name="maxConeAngle">Half angle of the primary cone, which defines the maximum possible range of motion in which the primary axis is restricted.</param>
        /// <param name="angularTwistRange">The range of angular motion for twisting around the primary axis within the region defined by the primary and perpendicular cones. This range is usually symmetrical.</param>
        public static void GetRagdollPrimaryConeAndTwistRange(in this PhysicsJoint joint, out sfloat maxConeAngle, out FloatRange angularTwistRange)
        {
            maxConeAngle = joint[PhysicsJoint.k_RagdollPrimaryMaxConeIndex].Max;
            angularTwistRange = new FloatRange
            {
                Min = joint[PhysicsJoint.k_RagdollPrimaryTwistRangeIndex].Min,
                Max = joint[PhysicsJoint.k_RagdollPrimaryTwistRangeIndex].Max
            };
        }

        /// <summary>
        /// Applies the specified range of motion to a ragdoll primary cone <see cref="PhysicsJoint"/> created using <see cref="PhysicsJoint.CreateRagdoll"/>.
        /// </summary>
        /// <param name="maxConeAngle">Half angle of the primary cone, which defines the maximum possible range of motion in which the primary axis is restricted. This value is clamped to the range (-pi, pi).</param>
        /// <param name="angularTwistRange">The range of angular motion for twisting around the primary axis within the region defined by the primary and perpendicular cones. This range is usually symmetrical, and is clamped to the range (-pi, pi).</param>
        public static void SetRagdollPrimaryConeAndTwistRange(ref this PhysicsJoint joint, sfloat maxConeAngle, FloatRange angularTwistRange)
        {
            angularTwistRange = math.clamp(angularTwistRange, new float2(-math.PI), new float2(math.PI));
            var constraints = joint.GetConstraints();
            constraints.ElementAt(PhysicsJoint.k_RagdollPrimaryMaxConeIndex).Min = sfloat.Zero;
            constraints.ElementAt(PhysicsJoint.k_RagdollPrimaryMaxConeIndex).Max = math.min(math.abs(maxConeAngle), math.PI);
            constraints.ElementAt(PhysicsJoint.k_RagdollPrimaryTwistRangeIndex).Min = angularTwistRange.Min;
            constraints.ElementAt(PhysicsJoint.k_RagdollPrimaryTwistRangeIndex).Max = angularTwistRange.Max;
            joint.SetConstraints(constraints);
        }

        /// <summary>
        /// Gets the range of motion for a ragdoll perpendicular cone <see cref="PhysicsJoint"/> created using <see cref="PhysicsJoint.CreateRagdoll"/>.
        /// </summary>
        /// <returns>The range of angular motion defining the cones perpendicular to the primary cone, between which the primary axis may swing. This range may be asymmetrical.</returns>
        public static FloatRange GetRagdollPerpendicularConeRange(in this PhysicsJoint joint) =>
            new FloatRange
            {
                Min = joint[PhysicsJoint.k_RagdollPerpendicularRangeIndex].Min,
                Max = joint[PhysicsJoint.k_RagdollPerpendicularRangeIndex].Max
            } - new float2(sfloat.FromRaw(0x3fc90fdb));

        /// <summary>
        /// Applies the specified range of motion to a ragdoll perpendicular cone <see cref="PhysicsJoint"/> created using <see cref="PhysicsJoint.CreateRagdoll"/>.
        /// </summary>
        /// <param name="angularPlaneRange">The range of angular motion defining the cones perpendicular to the primary cone, between which the primary cone's axis may swing. This range may be asymmetrical, and is clamped to the range (-pi/2, pi/2).</param>
        public static void SetRagdollPerpendicularConeRange(ref this PhysicsJoint joint, FloatRange angularPlaneRange)
        {
            angularPlaneRange = math.clamp(angularPlaneRange + new float2(sfloat.FromRaw(0x3fc90fdb)), new float2(sfloat.Zero), new float2(math.PI));
            var constraints = joint.GetConstraints();
            constraints.ElementAt(PhysicsJoint.k_RagdollPerpendicularRangeIndex).Min = angularPlaneRange.Min;
            constraints.ElementAt(PhysicsJoint.k_RagdollPerpendicularRangeIndex).Max = angularPlaneRange.Max;
            joint.SetConstraints(constraints);
        }

        /// <summary>
        /// Gets the constrained degrees of freedom of a <see cref="PhysicsJoint"/> created using <see cref="PhysicsJoint.CreateLimitedDOF"/>.
        /// </summary>
        /// <param name="linearAxes">The linear axes constrained by the joint.</param>
        /// <param name="angularAxes">The angular axes constrained by the joint.</param>
        public static void GetLimitedDOFAxes(in this PhysicsJoint joint, out bool3 linearAxes, out bool3 angularAxes)
        {
            linearAxes = joint[PhysicsJoint.k_LimitedDOFLinearIndex].ConstrainedAxes;
            angularAxes = joint[PhysicsJoint.k_LimitedDOFAngularIndex].ConstrainedAxes;
        }

        /// <summary>
        /// Applies the constrained degrees of freedom to a <see cref="PhysicsJoint"/> created using <see cref="PhysicsJoint.CreateLimitedDOF"/>.
        /// </summary>
        /// <param name="linearAxes">The linear axes constrained by the joint.</param>
        /// <param name="angularAxes">The angular axes constrained by the joint.</param>
        public static void setLimitedDOFAxes(ref this PhysicsJoint joint, bool3 linearAxes, bool3 angularAxes)
        {
            var constraints = joint.GetConstraints();
            constraints.ElementAt(PhysicsJoint.k_LimitedDOFLinearIndex).ConstrainedAxes = linearAxes;
            constraints.ElementAt(PhysicsJoint.k_LimitedDOFAngularIndex).ConstrainedAxes = angularAxes;
            joint.SetConstraints(constraints);
        }

        #endregion
    }
}
