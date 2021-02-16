using Unity.Burst;
using UnityS.Mathematics;
using static UnityS.Physics.Math;

namespace UnityS.Physics
{
    // Solve data for a constraint that limits three degrees of angular freedom
    [NoAlias]
    struct AngularLimit3DJacobian
    {
        // Relative angle limits
        public sfloat MinAngle;
        public sfloat MaxAngle;

        // Relative orientation of motions before solving
        public quaternion BFromA;

        // Angle is zero when BFromA = RefBFromA
        public quaternion RefBFromA;

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
            BFromA = math.mul(math.inverse(motionB.WorldFromMotion.rot), motionA.WorldFromMotion.rot);
            RefBFromA = new quaternion(math.mul(bFromConstraint.Rotation, aFromConstraint.InverseRotation));
            MinAngle = constraint.Min;
            MaxAngle = constraint.Max;
            Tau = tau;
            Damping = damping;

            quaternion jointOrientation = math.mul(math.inverse(RefBFromA), BFromA);
            sfloat initialAngle = math.asin(math.length(jointOrientation.value.xyz)) * (sfloat)2.0f;
            InitialError = JacobianUtilities.CalculateError(initialAngle, MinAngle, MaxAngle);
        }

        public void Solve(ref MotionVelocity velocityA, ref MotionVelocity velocityB, sfloat timestep, sfloat invTimestep)
        {
            // Predict the relative orientation at the end of the step
            quaternion futureBFromA = JacobianUtilities.IntegrateOrientationBFromA(BFromA, velocityA.AngularVelocity, velocityB.AngularVelocity, timestep);

            // Find the future axis and angle of rotation between the free axes
            float3 jacA0, jacA1, jacA2, jacB0, jacB1, jacB2;
            float3 effectiveMass; // first column of 3x3 effective mass matrix, don't need the others because only jac0 can have nonzero error
            sfloat futureAngle;
            {
                // Calculate the relative rotation between joint spaces
                quaternion jointOrientation = math.mul(math.inverse(RefBFromA), futureBFromA);

                // Find the axis and angle of rotation
                jacA0 = jointOrientation.value.xyz;
                sfloat sinHalfAngleSq = math.lengthsq(jacA0);
                sfloat invSinHalfAngle = Math.RSqrtSafe(sinHalfAngleSq);
                sfloat sinHalfAngle = sinHalfAngleSq * invSinHalfAngle;
                futureAngle = math.asin(sinHalfAngle) * (sfloat)2.0f;

                jacA0 = math.select(jacA0 * invSinHalfAngle, new float3(sfloat.One, sfloat.Zero, sfloat.Zero), invSinHalfAngle.IsZero());
                jacA0 = math.select(jacA0, -jacA0, jointOrientation.value.w < sfloat.Zero);
                Math.CalculatePerpendicularNormalized(jacA0, out jacA1, out jacA2);

                jacB0 = math.mul(futureBFromA, -jacA0);
                jacB1 = math.mul(futureBFromA, -jacA1);
                jacB2 = math.mul(futureBFromA, -jacA2);

                // Calculate the effective mass
                float3 invEffectiveMassDiag = new float3(
                    math.csum(jacA0 * jacA0 * velocityA.InverseInertia + jacB0 * jacB0 * velocityB.InverseInertia),
                    math.csum(jacA1 * jacA1 * velocityA.InverseInertia + jacB1 * jacB1 * velocityB.InverseInertia),
                    math.csum(jacA2 * jacA2 * velocityA.InverseInertia + jacB2 * jacB2 * velocityB.InverseInertia));
                float3 invEffectiveMassOffDiag = new float3(
                    math.csum(jacA0 * jacA1 * velocityA.InverseInertia + jacB0 * jacB1 * velocityB.InverseInertia),
                    math.csum(jacA0 * jacA2 * velocityA.InverseInertia + jacB0 * jacB2 * velocityB.InverseInertia),
                    math.csum(jacA1 * jacA2 * velocityA.InverseInertia + jacB1 * jacB2 * velocityB.InverseInertia));
                JacobianUtilities.InvertSymmetricMatrix(invEffectiveMassDiag, invEffectiveMassOffDiag, out float3 effectiveMassDiag, out float3 effectiveMassOffDiag);
                effectiveMass = JacobianUtilities.BuildSymmetricMatrix(effectiveMassDiag, effectiveMassOffDiag).c0;
            }

            // Calculate the error, adjust by tau and damping, and apply an impulse to correct it
            sfloat futureError = JacobianUtilities.CalculateError(futureAngle, MinAngle, MaxAngle);
            sfloat solveError = JacobianUtilities.CalculateCorrection(futureError, InitialError, Tau, Damping);
            sfloat solveVelocity = -solveError * invTimestep;
            float3 impulseA = solveVelocity * (jacA0 * effectiveMass.x + jacA1 * effectiveMass.y + jacA2 * effectiveMass.z);
            float3 impulseB = solveVelocity * (jacB0 * effectiveMass.x + jacB1 * effectiveMass.y + jacB2 * effectiveMass.z);
            velocityA.ApplyAngularImpulse(impulseA);
            velocityB.ApplyAngularImpulse(impulseB);
        }
    }
}
