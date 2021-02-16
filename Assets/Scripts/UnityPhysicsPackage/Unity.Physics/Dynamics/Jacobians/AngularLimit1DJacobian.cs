using Unity.Burst;
using UnityS.Mathematics;
using static UnityS.Physics.Math;

namespace UnityS.Physics
{
    // Solve data for a constraint that limits one degree of angular freedom
    [NoAlias]
    struct AngularLimit1DJacobian
    {
        // Limited axis in motion A space
        // TODO could calculate this from AxisIndex and MotionAFromJoint
        public float3 AxisInMotionA;

        // Index of the limited axis
        public int AxisIndex;

        // Relative angle limits
        public sfloat MinAngle;
        public sfloat MaxAngle;

        // Relative orientation of the motions before solving
        public quaternion MotionBFromA;

        // Rotation to joint space from motion space
        public quaternion MotionAFromJoint;
        public quaternion MotionBFromJoint;

        // Error before solving
        public sfloat InitialError;

        // Fraction of the position error to correct per step
        public sfloat Tau;

        // Fraction of the velocity error to correct per step
        public sfloat Damping;

        // Build the Jacobian
        public void Build(
            MTransform aFromConstraint, MTransform bFromConstraint,
            MotionVelocity velocityA, MotionVelocity velocityB,
            MotionData motionA, MotionData motionB,
            Constraint constraint, sfloat tau, sfloat damping)
        {
            // Copy the constraint into the jacobian
            AxisIndex = constraint.ConstrainedAxis1D;
            AxisInMotionA = aFromConstraint.Rotation[AxisIndex];
            MinAngle = constraint.Min;
            MaxAngle = constraint.Max;
            Tau = tau;
            Damping = damping;
            MotionBFromA = math.mul(math.inverse(motionB.WorldFromMotion.rot), motionA.WorldFromMotion.rot);
            MotionAFromJoint = new quaternion(aFromConstraint.Rotation);
            MotionBFromJoint = new quaternion(bFromConstraint.Rotation);

            // Calculate the current error
            InitialError = CalculateError(MotionBFromA);
        }

        // Solve the Jacobian
        public void Solve(ref MotionVelocity velocityA, ref MotionVelocity velocityB, sfloat timestep, sfloat invTimestep)
        {
            // Predict the relative orientation at the end of the step
            quaternion futureMotionBFromA = JacobianUtilities.IntegrateOrientationBFromA(MotionBFromA, velocityA.AngularVelocity, velocityB.AngularVelocity, timestep);

            // Calculate the effective mass
            float3 axisInMotionB = math.mul(futureMotionBFromA, -AxisInMotionA);
            sfloat effectiveMass;
            {
                sfloat invEffectiveMass = math.csum(AxisInMotionA * AxisInMotionA * velocityA.InverseInertia +
                    axisInMotionB * axisInMotionB * velocityB.InverseInertia);
                effectiveMass = math.select(sfloat.One / invEffectiveMass, sfloat.Zero, invEffectiveMass.IsZero());
            }

            // Calculate the error, adjust by tau and damping, and apply an impulse to correct it
            sfloat futureError = CalculateError(futureMotionBFromA);
            sfloat solveError = JacobianUtilities.CalculateCorrection(futureError, InitialError, Tau, Damping);
            sfloat impulse = math.mul(effectiveMass, -solveError) * invTimestep;
            velocityA.ApplyAngularImpulse(impulse * AxisInMotionA);
            velocityB.ApplyAngularImpulse(impulse * axisInMotionB);
        }

        // Helper function
        private sfloat CalculateError(quaternion motionBFromA)
        {
            // Calculate the relative body rotation
            quaternion jointBFromA = math.mul(math.mul(math.inverse(MotionBFromJoint), motionBFromA), MotionAFromJoint);

            // Find the twist angle of the rotation.
            //
            // There is no one correct solution for the twist angle. Suppose the joint models a pair of bodies connected by
            // three gimbals, one of which is limited by this jacobian. There are multiple configurations of the gimbals that
            // give the bodies the same relative orientation, so it is impossible to determine the configuration from the
            // bodies' orientations alone, nor therefore the orientation of the limited gimbal.
            //
            // This code instead makes a reasonable guess, the twist angle of the swing-twist decomposition of the bodies'
            // relative orientation. It always works when the limited axis itself is unable to rotate freely, as in a limited
            // hinge. It works fairly well when the limited axis can only rotate a small amount, preferably less than 90
            // degrees. It works poorly at higher angles, especially near 180 degrees where it is not continuous. For systems
            // that require that kind of flexibility, the gimbals should be modeled as separate bodies.
            sfloat angle = CalculateTwistAngle(jointBFromA, AxisIndex);

            // Angle is in [-2pi, 2pi].
            // For comparison against the limits, find k so that angle + 2k * pi is as close to [min, max] as possible.
            sfloat centerAngle = (MinAngle + MaxAngle) / (sfloat)2.0f;
            bool above = angle > (centerAngle + math.PI);
            bool below = angle < (centerAngle - math.PI);
            angle = math.select(angle, angle - (sfloat)2.0f * math.PI, above);
            angle = math.select(angle, angle + (sfloat)2.0f * math.PI, below);

            // Calculate the relative angle about the twist axis
            return JacobianUtilities.CalculateError(angle, MinAngle, MaxAngle);
        }
    }
}
