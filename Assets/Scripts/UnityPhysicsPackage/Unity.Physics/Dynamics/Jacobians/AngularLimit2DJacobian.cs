using Unity.Burst;
using UnityS.Mathematics;
using static UnityS.Physics.Math;

namespace UnityS.Physics
{
    // Solve data for a constraint that limits two degrees of angular freedom
    [NoAlias]
    struct AngularLimit2DJacobian
    {
        // Free axes in motion space
        public float3 AxisAinA;
        public float3 AxisBinB;

        // Relative angle limits
        public sfloat MinAngle;
        public sfloat MaxAngle;

        // Relative orientation before solving
        public quaternion BFromA;

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
            // Copy the constraint data
            int freeIndex = constraint.FreeAxis2D;
            AxisAinA = aFromConstraint.Rotation[freeIndex];
            AxisBinB = bFromConstraint.Rotation[freeIndex];
            MinAngle = constraint.Min;
            MaxAngle = constraint.Max;
            Tau = tau;
            Damping = damping;
            BFromA = math.mul(math.inverse(motionB.WorldFromMotion.rot), motionA.WorldFromMotion.rot);

            // Calculate the initial error
            {
                float3 axisAinB = math.mul(BFromA, AxisAinA);
                sfloat sinAngle = math.length(math.cross(axisAinB, AxisBinB));
                sfloat cosAngle = math.dot(axisAinB, AxisBinB);
                sfloat angle = math.atan2(sinAngle, cosAngle);
                InitialError = JacobianUtilities.CalculateError(angle, MinAngle, MaxAngle);
            }
        }

        // Solve the Jacobian
        public void Solve(ref MotionVelocity velocityA, ref MotionVelocity velocityB, sfloat timestep, sfloat invTimestep)
        {
            // Predict the relative orientation at the end of the step
            quaternion futureBFromA = JacobianUtilities.IntegrateOrientationBFromA(BFromA, velocityA.AngularVelocity, velocityB.AngularVelocity, timestep);

            // Calculate the jacobian axis and angle
            float3 axisAinB = math.mul(futureBFromA, AxisAinA);
            float3 jacB0 = math.cross(axisAinB, AxisBinB);
            float3 jacA0 = math.mul(math.inverse(futureBFromA), -jacB0);
            sfloat jacLengthSq = math.lengthsq(jacB0);
            sfloat invJacLength = Math.RSqrtSafe(jacLengthSq);
            sfloat futureAngle;
            {
                sfloat sinAngle = jacLengthSq * invJacLength;
                sfloat cosAngle = math.dot(axisAinB, AxisBinB);
                futureAngle = math.atan2(sinAngle, cosAngle);
            }

            // Choose a second jacobian axis perpendicular to A
            float3 jacB1 = math.cross(jacB0, axisAinB);
            float3 jacA1 = math.mul(math.inverse(futureBFromA), -jacB1);

            // Calculate effective mass
            float2 effectiveMass; // First column of the 2x2 matrix, we don't need the second column because the second component of error is zero
            {
                // Calculate the inverse effective mass matrix, then invert it
                sfloat invEffMassDiag0 = math.csum(jacA0 * jacA0 * velocityA.InverseInertia + jacB0 * jacB0 * velocityB.InverseInertia);
                sfloat invEffMassDiag1 = math.csum(jacA1 * jacA1 * velocityA.InverseInertia + jacB1 * jacB1 * velocityB.InverseInertia);
                sfloat invEffMassOffDiag = math.csum(jacA0 * jacA1 * velocityA.InverseInertia + jacB0 * jacB1 * velocityB.InverseInertia);
                sfloat det = invEffMassDiag0 * invEffMassDiag1 - invEffMassOffDiag * invEffMassOffDiag;
                sfloat invDet = math.select(jacLengthSq / det, sfloat.Zero, det.IsZero()); // scale by jacLengthSq because the jacs were not normalized
                effectiveMass = invDet * new float2(invEffMassDiag1, -invEffMassOffDiag);
            }

            // Normalize the jacobians
            jacA0 *= invJacLength;
            jacB0 *= invJacLength;
            jacA1 *= invJacLength;
            jacB1 *= invJacLength;

            // Calculate the error, adjust by tau and damping, and apply an impulse to correct it
            sfloat futureError = JacobianUtilities.CalculateError(futureAngle, MinAngle, MaxAngle);
            sfloat solveError = JacobianUtilities.CalculateCorrection(futureError, InitialError, Tau, Damping);
            float2 impulse = -effectiveMass * solveError * invTimestep;
            velocityA.ApplyAngularImpulse(impulse.x * jacA0 + impulse.y * jacA1);
            velocityB.ApplyAngularImpulse(impulse.x * jacB0 + impulse.y * jacB1);
        }
    }
}
