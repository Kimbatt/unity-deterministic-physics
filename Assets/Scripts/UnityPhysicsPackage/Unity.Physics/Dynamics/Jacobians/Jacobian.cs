using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityS.Mathematics;
using UnityEngine.Assertions;

namespace UnityS.Physics
{
    public enum JacobianType : byte
    {
        // Contact Jacobians
        Contact,
        Trigger,

        // Joint Jacobians
        LinearLimit,
        AngularLimit1D,
        AngularLimit2D,
        AngularLimit3D
    }

    // Flags which enable optional Jacobian behaviors
    [Flags]
    public enum JacobianFlags : byte
    {
        // These flags apply to all Jacobians
        Disabled = 1 << 0,
        EnableMassFactors = 1 << 1,
        UserFlag0 = 1 << 2,
        UserFlag1 = 1 << 3,
        UserFlag2 = 1 << 4,

        // These flags apply only to contact Jacobians
        IsTrigger = 1 << 5,
        EnableCollisionEvents = 1 << 6,
        EnableSurfaceVelocity = 1 << 7
    }

    // Jacobian header, first part of each Jacobian in the stream
    struct JacobianHeader
    {
        public BodyIndexPair BodyPair { get; internal set; }
        public JacobianType Type { get; internal set; }
        public JacobianFlags Flags { get; internal set; }

        // Whether the Jacobian should be solved or not
        public bool Enabled
        {
            get => ((Flags & JacobianFlags.Disabled) == 0);
            set => Flags = value ? (Flags & ~JacobianFlags.Disabled) : (Flags | JacobianFlags.Disabled);
        }

        // Whether the Jacobian contains manifold data for collision events or not
        public bool HasContactManifold => (Flags & JacobianFlags.EnableCollisionEvents) != 0;

        // Collider keys for the collision events
        public ColliderKeyPair ColliderKeys
        {
            get => HasContactManifold ? AccessColliderKeys() : ColliderKeyPair.Empty;
            set
            {
                if (HasContactManifold)
                    AccessColliderKeys() = value;
                else
                    SafetyChecks.ThrowNotSupportedException("Jacobian does not have collision events enabled");
            }
        }

        // Overrides for the mass properties of the pair of bodies
        public bool HasMassFactors => (Flags & JacobianFlags.EnableMassFactors) != 0;
        public MassFactors MassFactors
        {
            get => HasMassFactors ? AccessMassFactors() : MassFactors.Default;
            set
            {
                if (HasMassFactors)
                    AccessMassFactors() = value;
                else
                    SafetyChecks.ThrowNotSupportedException("Jacobian does not have mass factors enabled");
            }
        }

        // The surface velocity to apply to contact points
        public bool HasSurfaceVelocity => (Flags & JacobianFlags.EnableSurfaceVelocity) != 0;
        public SurfaceVelocity SurfaceVelocity
        {
            get => HasSurfaceVelocity ? AccessSurfaceVelocity() : new SurfaceVelocity();
            set
            {
                if (HasSurfaceVelocity)
                    AccessSurfaceVelocity() = value;
                else
                    SafetyChecks.ThrowNotSupportedException("Jacobian does not have surface velocity enabled");
            }
        }

        // Solve the Jacobian
        public void Solve([NoAlias] ref MotionVelocity velocityA, [NoAlias] ref MotionVelocity velocityB, Solver.StepInput stepInput,
            [NoAlias] ref NativeStream.Writer collisionEventsWriter, [NoAlias] ref NativeStream.Writer triggerEventsWriter, bool enableFrictionVelocitiesHeuristic,
            Solver.MotionStabilizationInput motionStabilizationSolverInputA, Solver.MotionStabilizationInput motionStabilizationSolverInputB)
        {
            if (Enabled)
            {
                switch (Type)
                {
                    case JacobianType.Contact:
                        AccessBaseJacobian<ContactJacobian>().Solve(ref this, ref velocityA, ref velocityB, stepInput, ref collisionEventsWriter,
                            enableFrictionVelocitiesHeuristic, motionStabilizationSolverInputA, motionStabilizationSolverInputB);
                        break;
                    case JacobianType.Trigger:
                        AccessBaseJacobian<TriggerJacobian>().Solve(ref this, ref velocityA, ref velocityB, stepInput, ref triggerEventsWriter);
                        break;
                    case JacobianType.LinearLimit:
                        AccessBaseJacobian<LinearLimitJacobian>().Solve(ref velocityA, ref velocityB, stepInput.Timestep, stepInput.InvTimestep);
                        break;
                    case JacobianType.AngularLimit1D:
                        AccessBaseJacobian<AngularLimit1DJacobian>().Solve(ref velocityA, ref velocityB, stepInput.Timestep, stepInput.InvTimestep);
                        break;
                    case JacobianType.AngularLimit2D:
                        AccessBaseJacobian<AngularLimit2DJacobian>().Solve(ref velocityA, ref velocityB, stepInput.Timestep, stepInput.InvTimestep);
                        break;
                    case JacobianType.AngularLimit3D:
                        AccessBaseJacobian<AngularLimit3DJacobian>().Solve(ref velocityA, ref velocityB, stepInput.Timestep, stepInput.InvTimestep);
                        break;
                    default:
                        SafetyChecks.ThrowNotImplementedException();
                        return;
                }
            }
        }

        #region Helpers

        public static int CalculateSize(JacobianType type, JacobianFlags flags, int numContactPoints = 0)
        {
            return UnsafeUtility.SizeOf<JacobianHeader>() +
                SizeOfBaseJacobian(type) + SizeOfModifierData(type, flags) +
                numContactPoints * UnsafeUtility.SizeOf<ContactJacAngAndVelToReachCp>() +
                SizeOfContactPointData(type, flags, numContactPoints);
        }

        private static int SizeOfColliderKeys(JacobianType type, JacobianFlags flags)
        {
            return (type == JacobianType.Contact && (flags & JacobianFlags.EnableCollisionEvents) != 0) ?
                UnsafeUtility.SizeOf<ColliderKeyPair>() : 0;
        }

        private static int SizeOfEntityPair(JacobianType type, JacobianFlags flags)
        {
            return (type == JacobianType.Contact && (flags & JacobianFlags.EnableCollisionEvents) != 0) ?
                UnsafeUtility.SizeOf<EntityPair>() : 0;
        }

        private static int SizeOfSurfaceVelocity(JacobianType type, JacobianFlags flags)
        {
            return (type == JacobianType.Contact && (flags & JacobianFlags.EnableSurfaceVelocity) != 0) ?
                UnsafeUtility.SizeOf<SurfaceVelocity>() : 0;
        }

        private static int SizeOfMassFactors(JacobianType type, JacobianFlags flags)
        {
            return (type == JacobianType.Contact && (flags & JacobianFlags.EnableMassFactors) != 0) ?
                UnsafeUtility.SizeOf<MassFactors>() : 0;
        }

        private static int SizeOfModifierData(JacobianType type, JacobianFlags flags)
        {
            return SizeOfColliderKeys(type, flags) + SizeOfEntityPair(type, flags) + SizeOfSurfaceVelocity(type, flags) +
                SizeOfMassFactors(type, flags);
        }

        private static int SizeOfContactPointData(JacobianType type, JacobianFlags flags, int numContactPoints = 0)
        {
            return (type == JacobianType.Contact && (flags & JacobianFlags.EnableCollisionEvents) != 0) ?
                numContactPoints * UnsafeUtility.SizeOf<ContactPoint>() : 0;
        }

        private static int SizeOfBaseJacobian(JacobianType type)
        {
            switch (type)
            {
                case JacobianType.Contact:
                    return UnsafeUtility.SizeOf<ContactJacobian>();
                case JacobianType.Trigger:
                    return UnsafeUtility.SizeOf<TriggerJacobian>();
                case JacobianType.LinearLimit:
                    return UnsafeUtility.SizeOf<LinearLimitJacobian>();
                case JacobianType.AngularLimit1D:
                    return UnsafeUtility.SizeOf<AngularLimit1DJacobian>();
                case JacobianType.AngularLimit2D:
                    return UnsafeUtility.SizeOf<AngularLimit2DJacobian>();
                case JacobianType.AngularLimit3D:
                    return UnsafeUtility.SizeOf<AngularLimit3DJacobian>();
                default:
                    SafetyChecks.ThrowNotImplementedException();
                    return default;
            }
        }

        // Access to "base" jacobian - a jacobian that comes after the header
        public unsafe ref T AccessBaseJacobian<T>() where T : struct
        {
            byte* ptr = (byte*)UnsafeUtility.AddressOf(ref this);
            ptr += UnsafeUtility.SizeOf<JacobianHeader>();
            return ref UnsafeUtility.AsRef<T>(ptr);
        }

        public unsafe ref ColliderKeyPair AccessColliderKeys()
        {
            Assert.IsTrue((Flags & JacobianFlags.EnableCollisionEvents) != 0);
            byte* ptr = (byte*)UnsafeUtility.AddressOf(ref this);
            ptr += UnsafeUtility.SizeOf<JacobianHeader>() + SizeOfBaseJacobian(Type);
            return ref UnsafeUtility.AsRef<ColliderKeyPair>(ptr);
        }

        public unsafe ref EntityPair AccessEntities()
        {
            Assert.IsTrue((Flags & JacobianFlags.EnableCollisionEvents) != 0);
            byte* ptr = (byte*)UnsafeUtility.AddressOf(ref this);
            ptr += UnsafeUtility.SizeOf<JacobianHeader>() + SizeOfBaseJacobian(Type) + SizeOfColliderKeys(Type, Flags);
            return ref UnsafeUtility.AsRef<EntityPair>(ptr);
        }

        public unsafe ref SurfaceVelocity AccessSurfaceVelocity()
        {
            Assert.IsTrue((Flags & JacobianFlags.EnableSurfaceVelocity) != 0);
            byte* ptr = (byte*)UnsafeUtility.AddressOf(ref this);
            ptr += UnsafeUtility.SizeOf<JacobianHeader>() + SizeOfBaseJacobian(Type) +
                SizeOfColliderKeys(Type, Flags) + SizeOfEntityPair(Type, Flags);
            return ref UnsafeUtility.AsRef<SurfaceVelocity>(ptr);
        }

        public unsafe ref MassFactors AccessMassFactors()
        {
            Assert.IsTrue((Flags & JacobianFlags.EnableMassFactors) != 0);
            byte* ptr = (byte*)UnsafeUtility.AddressOf(ref this);
            ptr += UnsafeUtility.SizeOf<JacobianHeader>() + SizeOfBaseJacobian(Type) +
                SizeOfColliderKeys(Type, Flags) + SizeOfEntityPair(Type, Flags) + SizeOfSurfaceVelocity(Type, Flags);
            return ref UnsafeUtility.AsRef<MassFactors>(ptr);
        }

        public unsafe ref ContactJacAngAndVelToReachCp AccessAngularJacobian(int pointIndex)
        {
            Assert.IsTrue(Type == JacobianType.Contact || Type == JacobianType.Trigger);
            byte* ptr = (byte*)UnsafeUtility.AddressOf(ref this);
            ptr += UnsafeUtility.SizeOf<JacobianHeader>() + SizeOfBaseJacobian(Type) + SizeOfModifierData(Type, Flags) +
                pointIndex * UnsafeUtility.SizeOf<ContactJacAngAndVelToReachCp>();
            return ref UnsafeUtility.AsRef<ContactJacAngAndVelToReachCp>(ptr);
        }

        public unsafe ref ContactPoint AccessContactPoint(int pointIndex)
        {
            Assert.IsTrue(Type == JacobianType.Contact);

            var baseJac = AccessBaseJacobian<ContactJacobian>();
            byte* ptr = (byte*)UnsafeUtility.AddressOf(ref this);
            ptr += UnsafeUtility.SizeOf<JacobianHeader>() + SizeOfBaseJacobian(Type) + SizeOfModifierData(Type, Flags) +
                baseJac.BaseJacobian.NumContacts * UnsafeUtility.SizeOf<ContactJacAngAndVelToReachCp>() +
                pointIndex * UnsafeUtility.SizeOf<ContactPoint>();
            return ref UnsafeUtility.AsRef<ContactPoint>(ptr);
        }

        #endregion
    }

    // Helper functions for working with Jacobians
    static class JacobianUtilities
    {
        public static void CalculateTauAndDamping(sfloat springFrequency, sfloat springDampingRatio, sfloat timestep, int iterations, out sfloat tau, out sfloat damping)
        {
            // TODO
            // - it's a significant amount of work to calculate tau and damping.  They depend on step length, so they have to be calculated each step.
            //   probably worth caching tau and damping for the default spring constants on the world and branching.
            // - you always get a higher effective damping ratio than you ask for because of the error from to discrete integration. The error varies
            //   with step length.  Can we estimate or bound that error and compensate for it?

            /*
            
            How to derive these formulas for tau and damping:

            1) implicit euler integration of a damped spring

               damped spring equation: x'' = -kx - cx'
               h = step length

               x2 = x1 + hv2
               v2 = v1 + h(-kx2 - cv2)/m
                  = v1 + h(-kx1 - hkv2 - cv2)/m
                  = v1 / (1 + h^2k/m + hc/m) - hkx1 / (m + h^2k + hc)

            2) gauss-seidel iterations of a stiff constraint.  Example for four iterations:

               t = tau, d = damping, a = 1 - d
               v2 = av1 - (t / h)x1
               v3 = av2 - (t / h)x1
               v4 = av3 - (t / h)x1
               v5 = av4 - (t / h)x1
                  = a^4v1 - (a^3 + a^2 + a + 1)(t / h)x1

            3) by matching coefficients of v1 and x1 in the formulas for v2 in step (1) and v5 in step (2), we see that if:

               (1 - damping)^4 = 1 / (1 + h^2k / m + hc / m)
               ((1 - damping)^3 + (1 - damping)^2 + (1 - damping) + 1)(tau / h) = hk / (m + h^2k + hc)

               then our constraint is equivalent to the implicit euler integration of a spring.
               solve the first equation for damping, then solve the second equation for tau.
               then substitute in k = mw^2, c = 2mzw.

            */

            sfloat h = timestep;
            sfloat w = springFrequency * (sfloat)2.0f * math.PI; // convert oscillations/sec to radians/sec
            sfloat z = springDampingRatio;
            sfloat hw = h * w;
            sfloat hhww = hw * hw;

            // a = 1-d, aExp = a^iterations, aSum = aExp / sum(i in [0, iterations), a^i)
            sfloat aExp = sfloat.One / (sfloat.One + hhww + (sfloat)2.0f * hw * z);
            sfloat a, aSum;
            if (iterations == 4)
            {
                // special case expected iterations = 4
                sfloat invA2 = math.rsqrt(aExp);
                sfloat a2 = invA2 * aExp;
                a = math.rsqrt(invA2);
                aSum = (sfloat.One + a2 + a * (sfloat.One + a2));
            }
            else
            {
                a = math.pow(aExp, sfloat.One / (sfloat)iterations);
                aSum = sfloat.One;
                for (int i = 1; i < iterations; i++)
                {
                    aSum = a * aSum + sfloat.One;
                }
            }

            damping = sfloat.One - a;
            tau = hhww * aExp / aSum;
        }

        public static void CalculateTauAndDamping(Constraint constraint, sfloat timestep, int iterations, out sfloat tau, out sfloat damping)
        {
            CalculateTauAndDamping(constraint.SpringFrequency, constraint.SpringDamping, timestep, iterations, out tau, out damping);
        }

        // Returns x - clamp(x, min, max)
        public static sfloat CalculateError(sfloat x, sfloat min, sfloat max)
        {
            sfloat error = math.max(x - max, sfloat.Zero);
            error = math.min(x - min, error);
            return error;
        }

        // Returns the amount of error for the solver to correct, where initialError is the pre-integration error and predictedError is the expected post-integration error
        public static sfloat CalculateCorrection(sfloat predictedError, sfloat initialError, sfloat tau, sfloat damping)
        {
            return math.max(predictedError - initialError, sfloat.Zero) * damping + math.min(predictedError, initialError) * tau;
        }

        // Integrate the relative orientation of a pair of bodies, faster and less memory than storing both bodies' orientations and integrating them separately
        public static quaternion IntegrateOrientationBFromA(quaternion bFromA, float3 angularVelocityA, float3 angularVelocityB, sfloat timestep)
        {
            quaternion dqA = Integrator.IntegrateAngularVelocity(angularVelocityA, timestep);
            quaternion dqB = Integrator.IntegrateAngularVelocity(angularVelocityB, timestep);
            return math.normalize(math.mul(math.mul(math.inverse(dqB), bFromA), dqA));
        }

        // Calculate the inverse effective mass of a linear jacobian
        public static sfloat CalculateInvEffectiveMassDiag(
            float3 angA, float3 invInertiaA, sfloat invMassA,
            float3 angB, float3 invInertiaB, sfloat invMassB)
        {
            float3 angularPart = angA * angA * invInertiaA + angB * angB * invInertiaB;
            sfloat linearPart = invMassA + invMassB;
            return (angularPart.x + angularPart.y) + (angularPart.z + linearPart);
        }

        // Calculate the inverse effective mass for a pair of jacobians with perpendicular linear parts
        public static sfloat CalculateInvEffectiveMassOffDiag(
            float3 angA0, float3 angA1, float3 invInertiaA,
            float3 angB0, float3 angB1, float3 invInertiaB)
        {
            return math.csum(angA0 * angA1 * invInertiaA + angB0 * angB1 * invInertiaB);
        }

        // Inverts a symmetrix 3x3 matrix with diag = (0, 0), (1, 1), (2, 2), offDiag = (0, 1), (0, 2), (1, 2) = (1, 0), (2, 0), (2, 1)
        public static bool InvertSymmetricMatrix(float3 diag, float3 offDiag, out float3 invDiag, out float3 invOffDiag)
        {
            float3 offDiagSq = offDiag.zyx * offDiag.zyx;
            sfloat determinant = (Math.HorizontalMul(diag) + (sfloat)2.0f * Math.HorizontalMul(offDiag) - math.csum(offDiagSq * diag));
            bool determinantOk = !determinant.IsZero();
            sfloat invDeterminant = math.select(sfloat.Zero, sfloat.One / determinant, determinantOk);
            invDiag = (diag.yxx * diag.zzy - offDiagSq) * invDeterminant;
            invOffDiag = (offDiag.yxx * offDiag.zzy - diag.zyx * offDiag) * invDeterminant;
            return determinantOk;
        }

        // Builds a symmetric 3x3 matrix from diag = (0, 0), (1, 1), (2, 2), offDiag = (0, 1), (0, 2), (1, 2) = (1, 0), (2, 0), (2, 1)
        public static float3x3 BuildSymmetricMatrix(float3 diag, float3 offDiag)
        {
            return new float3x3(
                new float3(diag.x, offDiag.x, offDiag.y),
                new float3(offDiag.x, diag.y, offDiag.z),
                new float3(offDiag.y, offDiag.z, diag.z)
            );
        }
    }

    // Iterator (and modifier) for jacobians
    unsafe struct JacobianIterator
    {
        NativeStream.Reader m_Reader;

        public JacobianIterator(NativeStream.Reader jacobianStreamReader, int workItemIndex)
        {
            m_Reader = jacobianStreamReader;
            m_Reader.BeginForEachIndex(workItemIndex);
        }

        public bool HasJacobiansLeft()
        {
            return m_Reader.RemainingItemCount > 0;
        }

        public ref JacobianHeader ReadJacobianHeader()
        {
            int readSize = Read<int>();
            return ref UnsafeUtility.AsRef<JacobianHeader>(Read(readSize));
        }

        private byte* Read(int size)
        {
            byte* dataPtr = m_Reader.ReadUnsafePtr(size);

            return dataPtr;
        }

        private ref T Read<T>() where T : struct
        {
            int size = UnsafeUtility.SizeOf<T>();
            return ref UnsafeUtility.AsRef<T>(Read(size));
        }
    }
}
