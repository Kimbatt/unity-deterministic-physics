using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityS.Mathematics;

namespace UnityS.Physics
{
    public struct ContactJacobianAngular
    {
        public float3 AngularA;
        public float3 AngularB;
        public sfloat EffectiveMass;
        public sfloat Impulse; // Accumulated impulse
    }

    public struct ContactJacAngAndVelToReachCp  // TODO: better name
    {
        public ContactJacobianAngular Jac;

        // Velocity needed to reach the contact plane in one frame,
        // both if approaching (negative) and depenetrating (positive)
        public sfloat VelToReachCp;
    }

    public struct SurfaceVelocity
    {
        // Velocities between the two contacting objects
        public float3 LinearVelocity;
        public float3 AngularVelocity;
    }

    public struct MassFactors
    {
        public float3 InverseInertiaFactorA;
        public sfloat InverseMassFactorA;
        public float3 InverseInertiaFactorB;
        public sfloat InverseMassFactorB;

        public static MassFactors Default => new MassFactors
        {
            InverseInertiaFactorA = new float3(sfloat.One),
            InverseMassFactorA = sfloat.One,
            InverseInertiaFactorB = new float3(sfloat.One),
            InverseMassFactorB = sfloat.One
        };
    }

    struct BaseContactJacobian
    {
        public int NumContacts;
        public float3 Normal;

        internal static sfloat GetJacVelocity(float3 linear, ContactJacobianAngular jacAngular,
            float3 linVelA, float3 angVelA, float3 linVelB, float3 angVelB)
        {
            float3 temp = (linVelA - linVelB) * linear;
            temp += angVelA * jacAngular.AngularA;
            temp += angVelB * jacAngular.AngularB;
            return math.csum(temp);
        }
    }

    // A Jacobian representing a set of contact points that apply impulses
    [NoAlias]
    struct ContactJacobian
    {
        public BaseContactJacobian BaseJacobian;

        // Linear friction jacobians.  Only store the angular part, linear part can be recalculated from BaseJacobian.Normal
        public ContactJacobianAngular Friction0; // EffectiveMass stores friction effective mass matrix element (0, 0)
        public ContactJacobianAngular Friction1; // EffectiveMass stores friction effective mass matrix element (1, 1)

        // Angular friction about the contact normal, no linear part
        public ContactJacobianAngular AngularFriction; // EffectiveMass stores friction effective mass matrix element (2, 2)
        public float3 FrictionEffectiveMassOffDiag; // Effective mass matrix (0, 1), (0, 2), (1, 2) == (1, 0), (2, 0), (2, 1)

        public sfloat CoefficientOfFriction;

        // Generic solve method that dispatches to specific ones
        public void Solve(
            ref JacobianHeader jacHeader, ref MotionVelocity velocityA, ref MotionVelocity velocityB,
            Solver.StepInput stepInput, ref NativeStream.Writer collisionEventsWriter, bool enableFrictionVelocitiesHeuristic,
            Solver.MotionStabilizationInput motionStabilizationSolverInputA, Solver.MotionStabilizationInput motionStabilizationSolverInputB)
        {
            bool bothBodiesWithInfInertiaAndMass = velocityA.HasInfiniteInertiaAndMass && velocityB.HasInfiniteInertiaAndMass;
            if (bothBodiesWithInfInertiaAndMass)
            {
                SolveInfMassPair(ref jacHeader, velocityA, velocityB, stepInput, ref collisionEventsWriter);
            }
            else
            {
                SolveContact(ref jacHeader, ref velocityA, ref velocityB, stepInput, ref collisionEventsWriter,
                    enableFrictionVelocitiesHeuristic, motionStabilizationSolverInputA, motionStabilizationSolverInputB);
            }
        }

        // Solve the Jacobian
        public void SolveContact(
            ref JacobianHeader jacHeader, ref MotionVelocity velocityA, ref MotionVelocity velocityB,
            Solver.StepInput stepInput, ref NativeStream.Writer collisionEventsWriter, bool enableFrictionVelocitiesHeuristic,
            Solver.MotionStabilizationInput motionStabilizationSolverInputA, Solver.MotionStabilizationInput motionStabilizationSolverInputB)
        {
            // Copy velocity data
            MotionVelocity tempVelocityA = velocityA;
            MotionVelocity tempVelocityB = velocityB;
            if (jacHeader.HasMassFactors)
            {
                MassFactors jacMod = jacHeader.AccessMassFactors();
                tempVelocityA.InverseInertia *= jacMod.InverseInertiaFactorA;
                tempVelocityA.InverseMass *= jacMod.InverseMassFactorA;
                tempVelocityB.InverseInertia *= jacMod.InverseInertiaFactorB;
                tempVelocityB.InverseMass *= jacMod.InverseMassFactorB;
            }

            // Solve normal impulses
            sfloat sumImpulses = sfloat.Zero;
            sfloat totalAccumulatedImpulse = sfloat.Zero;
            bool forceCollisionEvent = false;
            for (int j = 0; j < BaseJacobian.NumContacts; j++)
            {
                ref ContactJacAngAndVelToReachCp jacAngular = ref jacHeader.AccessAngularJacobian(j);

                // Solve velocity so that predicted contact distance is greater than or equal to zero
                sfloat relativeVelocity = BaseContactJacobian.GetJacVelocity(BaseJacobian.Normal, jacAngular.Jac,
                    tempVelocityA.LinearVelocity, tempVelocityA.AngularVelocity, tempVelocityB.LinearVelocity, tempVelocityB.AngularVelocity);
                sfloat dv = jacAngular.VelToReachCp - relativeVelocity;

                sfloat impulse = dv * jacAngular.Jac.EffectiveMass;
                sfloat accumulatedImpulse = math.max(jacAngular.Jac.Impulse + impulse, sfloat.Zero);
                if (accumulatedImpulse != jacAngular.Jac.Impulse)
                {
                    sfloat deltaImpulse = accumulatedImpulse - jacAngular.Jac.Impulse;
                    ApplyImpulse(deltaImpulse, BaseJacobian.Normal, jacAngular.Jac, ref tempVelocityA, ref tempVelocityB,
                        motionStabilizationSolverInputA.InverseInertiaScale, motionStabilizationSolverInputB.InverseInertiaScale);
                }

                jacAngular.Jac.Impulse = accumulatedImpulse;
                sumImpulses += accumulatedImpulse;
                totalAccumulatedImpulse += jacAngular.Jac.Impulse;

                // Force contact event even when no impulse is applied, but there is penetration.
                forceCollisionEvent |= jacAngular.VelToReachCp > sfloat.Zero;
            }

            // Export collision event
            if (stepInput.IsLastIteration && (totalAccumulatedImpulse > sfloat.Zero || forceCollisionEvent) && jacHeader.HasContactManifold)
            {
                ExportCollisionEvent(totalAccumulatedImpulse, ref jacHeader, ref collisionEventsWriter);
            }

            // Solve friction
            if (sumImpulses > sfloat.Zero)
            {
                // Choose friction axes
                Math.CalculatePerpendicularNormalized(BaseJacobian.Normal, out float3 frictionDir0, out float3 frictionDir1);

                // Calculate impulses for full stop
                float3 imp;
                {
                    // Take velocities that produce minimum energy (between input and solver velocity) as friction input
                    float3 frictionLinVelA = tempVelocityA.LinearVelocity;
                    float3 frictionAngVelA = tempVelocityA.AngularVelocity;
                    float3 frictionLinVelB = tempVelocityB.LinearVelocity;
                    float3 frictionAngVelB = tempVelocityB.AngularVelocity;
                    if (enableFrictionVelocitiesHeuristic)
                    {
                        GetFrictionVelocities(motionStabilizationSolverInputA.InputVelocity.Linear, motionStabilizationSolverInputA.InputVelocity.Angular,
                            tempVelocityA.LinearVelocity, tempVelocityA.AngularVelocity,
                            math.rcp(tempVelocityA.InverseInertia), math.rcp(tempVelocityA.InverseMass),
                            out frictionLinVelA, out frictionAngVelA);
                        GetFrictionVelocities(motionStabilizationSolverInputB.InputVelocity.Linear, motionStabilizationSolverInputB.InputVelocity.Angular,
                            tempVelocityB.LinearVelocity, tempVelocityB.AngularVelocity,
                            math.rcp(tempVelocityB.InverseInertia), math.rcp(tempVelocityB.InverseMass),
                            out frictionLinVelB, out frictionAngVelB);
                    }

                    float3 extraFrictionDv = float3.zero;
                    if (jacHeader.HasSurfaceVelocity)
                    {
                        var surfVel = jacHeader.AccessSurfaceVelocity();

                        Math.CalculatePerpendicularNormalized(BaseJacobian.Normal, out float3 dir0, out float3 dir1);
                        sfloat linVel0 = math.dot(surfVel.LinearVelocity, dir0);
                        sfloat linVel1 = math.dot(surfVel.LinearVelocity, dir1);

                        sfloat angVelProj = math.dot(surfVel.AngularVelocity, BaseJacobian.Normal);
                        extraFrictionDv = new float3(linVel0, linVel1, angVelProj);
                    }

                    // Calculate the jacobian dot velocity for each of the friction jacobians
                    sfloat dv0 = extraFrictionDv.x - BaseContactJacobian.GetJacVelocity(frictionDir0, Friction0, frictionLinVelA, frictionAngVelA, frictionLinVelB, frictionAngVelB);
                    sfloat dv1 = extraFrictionDv.y - BaseContactJacobian.GetJacVelocity(frictionDir1, Friction1, frictionLinVelA, frictionAngVelA, frictionLinVelB, frictionAngVelB);
                    sfloat dva = extraFrictionDv.z - math.csum(AngularFriction.AngularA * frictionAngVelA + AngularFriction.AngularB * frictionAngVelB);

                    // Reassemble the effective mass matrix
                    float3 effectiveMassDiag = new float3(Friction0.EffectiveMass, Friction1.EffectiveMass, AngularFriction.EffectiveMass);
                    float3x3 effectiveMass = JacobianUtilities.BuildSymmetricMatrix(effectiveMassDiag, FrictionEffectiveMassOffDiag);

                    // Calculate the impulse
                    imp = math.mul(effectiveMass, new float3(dv0, dv1, dva));
                }

                // Clip TODO.ma calculate some contact radius and use it to influence balance between linear and angular friction
                sfloat maxImpulse = sumImpulses * CoefficientOfFriction * stepInput.InvNumSolverIterations;
                sfloat frictionImpulseSquared = math.lengthsq(imp);
                imp *= math.min(sfloat.One, maxImpulse * math.rsqrt(frictionImpulseSquared));

                // Apply impulses
                ApplyImpulse(imp.x, frictionDir0, Friction0, ref tempVelocityA, ref tempVelocityB,
                    motionStabilizationSolverInputA.InverseInertiaScale, motionStabilizationSolverInputB.InverseInertiaScale);
                ApplyImpulse(imp.y, frictionDir1, Friction1, ref tempVelocityA, ref tempVelocityB,
                    motionStabilizationSolverInputA.InverseInertiaScale, motionStabilizationSolverInputB.InverseInertiaScale);

                tempVelocityA.ApplyAngularImpulse(imp.z * AngularFriction.AngularA * motionStabilizationSolverInputA.InverseInertiaScale);
                tempVelocityB.ApplyAngularImpulse(imp.z * AngularFriction.AngularB * motionStabilizationSolverInputB.InverseInertiaScale);

                // Accumulate them
                Friction0.Impulse += imp.x;
                Friction1.Impulse += imp.y;
                AngularFriction.Impulse += imp.z;
            }

            // Write back linear and angular velocities. Changes to other properties, like InverseMass, should not be persisted.
            velocityA.LinearVelocity = tempVelocityA.LinearVelocity;
            velocityA.AngularVelocity = tempVelocityA.AngularVelocity;
            velocityB.LinearVelocity = tempVelocityB.LinearVelocity;
            velocityB.AngularVelocity = tempVelocityB.AngularVelocity;
        }

        // Solve the infinite mass pair Jacobian
        public void SolveInfMassPair(
            ref JacobianHeader jacHeader, MotionVelocity velocityA, MotionVelocity velocityB,
            Solver.StepInput stepInput, ref NativeStream.Writer collisionEventsWriter)
        {
            // Infinite mass pairs are only interested in collision events,
            // so only last iteration is performed in that case
            if (!stepInput.IsLastIteration)
            {
                return;
            }

            // Calculate normal impulses and fire collision event
            // if at least one contact point would have an impulse applied
            for (int j = 0; j < BaseJacobian.NumContacts; j++)
            {
                ref ContactJacAngAndVelToReachCp jacAngular = ref jacHeader.AccessAngularJacobian(j);

                sfloat relativeVelocity = BaseContactJacobian.GetJacVelocity(BaseJacobian.Normal, jacAngular.Jac,
                    velocityA.LinearVelocity, velocityA.AngularVelocity, velocityB.LinearVelocity, velocityB.AngularVelocity);
                sfloat dv = jacAngular.VelToReachCp - relativeVelocity;
                if (jacAngular.VelToReachCp > sfloat.Zero || dv > sfloat.Zero)
                {
                    // Export collision event only if impulse would be applied, or objects are penetrating
                    ExportCollisionEvent(sfloat.Zero, ref jacHeader, ref collisionEventsWriter);

                    return;
                }
            }
        }

        // Helper functions
        void GetFrictionVelocities(
            float3 inputLinearVelocity, float3 inputAngularVelocity,
            float3 intermediateLinearVelocity, float3 intermediateAngularVelocity,
            float3 inertia, sfloat mass,
            out float3 frictionLinearVelocityOut, out float3 frictionAngularVelocityOut)
        {
            sfloat inputEnergy;
            {
                sfloat linearEnergySq = mass * math.lengthsq(inputLinearVelocity);
                sfloat angularEnergySq = math.dot(inertia * inputAngularVelocity, inputAngularVelocity);
                inputEnergy = linearEnergySq + angularEnergySq;
            }

            sfloat intermediateEnergy;
            {
                sfloat linearEnergySq = mass * math.lengthsq(intermediateLinearVelocity);
                sfloat angularEnergySq = math.dot(inertia * intermediateAngularVelocity, intermediateAngularVelocity);
                intermediateEnergy = linearEnergySq + angularEnergySq;
            }

            if (inputEnergy < intermediateEnergy)
            {
                // Make sure we don't change the sign of intermediate velocity when using the input one.
                // If sign was to be changed, zero it out since it produces less energy.
                bool3 changedSignLin = inputLinearVelocity * intermediateLinearVelocity < float3.zero;
                bool3 changedSignAng = inputAngularVelocity * intermediateAngularVelocity < float3.zero;
                frictionLinearVelocityOut = math.select(inputLinearVelocity, float3.zero, changedSignLin);
                frictionAngularVelocityOut = math.select(inputAngularVelocity, float3.zero, changedSignAng);
            }
            else
            {
                frictionLinearVelocityOut = intermediateLinearVelocity;
                frictionAngularVelocityOut = intermediateAngularVelocity;
            }
        }

        private static void ApplyImpulse(
            sfloat impulse, float3 linear, ContactJacobianAngular jacAngular,
            ref MotionVelocity velocityA, ref MotionVelocity velocityB,
            sfloat inverseInertiaScaleA /* = 1.0f */, sfloat inverseInertiaScaleB /* = 1.0f */)
        {
            velocityA.ApplyLinearImpulse(impulse * linear);
            velocityB.ApplyLinearImpulse(-impulse * linear);

            // Scale the impulse with inverseInertiaScale
            velocityA.ApplyAngularImpulse(impulse * jacAngular.AngularA * inverseInertiaScaleA);
            velocityB.ApplyAngularImpulse(impulse * jacAngular.AngularB * inverseInertiaScaleB);
        }

        private unsafe void ExportCollisionEvent(sfloat totalAccumulatedImpulse, [NoAlias] ref JacobianHeader jacHeader,
            [NoAlias] ref NativeStream.Writer collisionEventsWriter)
        {
            // Write size before every event
            int collisionEventSize = CollisionEventData.CalculateSize(BaseJacobian.NumContacts);
            collisionEventsWriter.Write(collisionEventSize);

            // Allocate all necessary data for this event
            byte* eventPtr = collisionEventsWriter.Allocate(collisionEventSize);

            // Fill up event data
            ref CollisionEventData collisionEvent = ref UnsafeUtility.AsRef<CollisionEventData>(eventPtr);
            collisionEvent.BodyIndices = jacHeader.BodyPair;
            collisionEvent.ColliderKeys = jacHeader.AccessColliderKeys();
            collisionEvent.Entities = jacHeader.AccessEntities();
            collisionEvent.Normal = BaseJacobian.Normal;
            collisionEvent.SolverImpulse = totalAccumulatedImpulse;
            collisionEvent.NumNarrowPhaseContactPoints = BaseJacobian.NumContacts;
            for (int i = 0; i < BaseJacobian.NumContacts; i++)
            {
                collisionEvent.AccessContactPoint(i) = jacHeader.AccessContactPoint(i);
            }
        }
    }

    // A Jacobian representing a set of contact points that export trigger events
    [NoAlias]
    struct TriggerJacobian
    {
        public BaseContactJacobian BaseJacobian;
        public ColliderKeyPair ColliderKeys;
        public EntityPair Entities;

        // Solve the Jacobian
        public void Solve(
            ref JacobianHeader jacHeader, ref MotionVelocity velocityA, ref MotionVelocity velocityB, Solver.StepInput stepInput,
            ref NativeStream.Writer triggerEventsWriter)
        {
            // Export trigger events only in last iteration
            if (!stepInput.IsLastIteration)
            {
                return;
            }

            for (int j = 0; j < BaseJacobian.NumContacts; j++)
            {
                ref ContactJacAngAndVelToReachCp jacAngular = ref jacHeader.AccessAngularJacobian(j);

                // Solve velocity so that predicted contact distance is greater than or equal to zero
                sfloat relativeVelocity = BaseContactJacobian.GetJacVelocity(BaseJacobian.Normal, jacAngular.Jac,
                    velocityA.LinearVelocity, velocityA.AngularVelocity, velocityB.LinearVelocity, velocityB.AngularVelocity);
                sfloat dv = jacAngular.VelToReachCp - relativeVelocity;
                if (jacAngular.VelToReachCp > sfloat.Zero || dv > sfloat.Zero)
                {
                    // Export trigger event only if impulse would be applied, or objects are penetrating
                    triggerEventsWriter.Write(new TriggerEventData
                    {
                        BodyIndices = jacHeader.BodyPair,
                        ColliderKeys = ColliderKeys,
                        Entities = Entities
                    });

                    return;
                }
            }
        }
    }
}
