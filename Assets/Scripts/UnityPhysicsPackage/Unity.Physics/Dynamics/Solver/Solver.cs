using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using UnityS.Mathematics;
using UnityEngine.Assertions;
using static UnityS.Physics.Math;

namespace UnityS.Physics
{
    public static class Solver
    {
        // Settings for controlling the solver stabilization heuristic.
        public struct StabilizationHeuristicSettings
        {
            private byte m_EnableSolverStabilization;

            // Global switch to enable/disable the whole heuristic (false by default)
            public bool EnableSolverStabilization
            {
                get => m_EnableSolverStabilization > 0;
                set => m_EnableSolverStabilization = (byte)(value ? 1 : 0);
            }

            // Individual features control (only valid when EnableSolverStabilizationHeuristic is true)

            private byte m_EnableFrictionVelocities;

            // Switch to enable/disable heuristic when calculating friction velocities.
            // Should be disabled only if it is causing behavior issues.
            public bool EnableFrictionVelocities
            {
                get => m_EnableFrictionVelocities > 0;
                set => m_EnableFrictionVelocities = (byte)(value ? 1 : 0);
            }

            // Controls the intensity of the velocity clipping.
            // Defaults to 1.0f, while other values will scale the intensity up/down.
            // Shouldn't go higher than 5.0f, as it will result in bad behavior (too aggressive velocity clipping).
            // Set it to 0.0f to disable the feature.
            public sfloat VelocityClippingFactor;

            // Controls the intensity of inertia scaling.
            // Defaults to 1.0f, while other values will scale the intensity up/down.
            // Shouldn't go higher than 5.0f, as it will result in bad behavior (too high inertia of bodies).
            // Set it to 0.0f to disable the feature.
            public sfloat InertiaScalingFactor;

            public static readonly StabilizationHeuristicSettings Default = new StabilizationHeuristicSettings
            {
                m_EnableSolverStabilization = 0,
                m_EnableFrictionVelocities = 1,
                VelocityClippingFactor = sfloat.One,
                InertiaScalingFactor = sfloat.One
            };
        }

        // Data used for solver stabilization
        public struct StabilizationData
        {
            public StabilizationData(SimulationStepInput stepInput, SimulationContext context)
            {
                StabilizationHeuristicSettings = stepInput.SolverStabilizationHeuristicSettings;
                Gravity = stepInput.Gravity;
                if (stepInput.SolverStabilizationHeuristicSettings.EnableSolverStabilization)
                {
                    InputVelocities = context.InputVelocities;
                    MotionData = context.SolverStabilizationMotionData;
                }
                else
                {
                    InputVelocities = default;
                    MotionData = default;
                }
            }

            // Settings for stabilization heuristics
            internal StabilizationHeuristicSettings StabilizationHeuristicSettings;

            // Disable container safety restriction because it will complain about aliasing
            // with SimulationContext buffers, and it is aliasing, but completely safe.
            // Also, we need the ability to have these not allocated when the feature is not used.

            [NativeDisableParallelForRestriction]
            [NativeDisableContainerSafetyRestriction]
            internal NativeArray<Velocity> InputVelocities;

            [NativeDisableParallelForRestriction]
            [NativeDisableContainerSafetyRestriction]
            internal NativeArray<StabilizationMotionData> MotionData;

            // Gravity is used to define thresholds for stabilization,
            // and it's not needed in the solver unless stabilization is required.
            internal float3 Gravity;
        }

        // Per motion data for solver stabilization
        internal struct StabilizationMotionData
        {
            public sfloat InverseInertiaScale;
            public byte NumPairs;
        }

        // Internal motion data input for the solver stabilization
        internal struct MotionStabilizationInput
        {
            public Velocity InputVelocity;
            public sfloat InverseInertiaScale;

            public static readonly MotionStabilizationInput Default = new MotionStabilizationInput
            {
                InputVelocity = Velocity.Zero,
                InverseInertiaScale = sfloat.One
            };
        }

        internal struct StepInput
        {
            public bool IsFirstIteration;
            public bool IsLastIteration;
            public sfloat InvNumSolverIterations;
            public sfloat Timestep;
            public sfloat InvTimestep;
        }

        // Apply gravity to all dynamic bodies and copy input velocities
        public static void ApplyGravityAndCopyInputVelocities(NativeArray<MotionVelocity> motionVelocities,
            NativeArray<Velocity> inputVelocities, float3 gravityAcceleration)
        {
            for (int i = 0; i < motionVelocities.Length; i++)
            {
                ParallelApplyGravityAndCopyInputVelocitiesJob.ExecuteImpl(i, gravityAcceleration, motionVelocities, inputVelocities);
            }
        }

        // Schedule the job to apply gravity to all dynamic bodies and copy input velocities
        internal static JobHandle ScheduleApplyGravityAndCopyInputVelocitiesJob(
            NativeArray<MotionVelocity> motionVelocities, NativeArray<Velocity> inputVelocities,
            float3 gravityAcceleration, JobHandle inputDeps, bool multiThreaded = true)
        {
            if (!multiThreaded)
            {
                var job = new ApplyGravityAndCopyInputVelocitiesJob
                {
                    MotionVelocities = motionVelocities,
                    InputVelocities = inputVelocities,
                    GravityAcceleration = gravityAcceleration
                };
                return job.Schedule(inputDeps);
            }
            else
            {
                var job = new ParallelApplyGravityAndCopyInputVelocitiesJob
                {
                    MotionVelocities = motionVelocities,
                    InputVelocities = inputVelocities,
                    GravityAcceleration = gravityAcceleration
                };
                return job.Schedule(motionVelocities.Length, 64, inputDeps);
            }
        }

        // Build Jacobians from the contacts and joints stored in the simulation context
        public static void BuildJacobians(ref PhysicsWorld world,
            sfloat timeStep, float3 gravity, int numSolverIterations,
            NativeArray<DispatchPairSequencer.DispatchPair> dispatchPairs,
            ref NativeStream.Reader contactsReader, ref NativeStream.Writer jacobiansWriter)
        {
            contactsReader.BeginForEachIndex(0);
            jacobiansWriter.BeginForEachIndex(0);
            sfloat invTimeStep = timeStep > sfloat.Zero ? sfloat.One / timeStep : sfloat.Zero;
            sfloat gravityAcceleration = math.length(gravity);
            BuildJacobians(ref world, timeStep, invTimeStep, gravityAcceleration, numSolverIterations,
                dispatchPairs, 0, dispatchPairs.Length, ref contactsReader, ref jacobiansWriter);
        }

        // Schedule jobs to build Jacobians from the contacts stored in the simulation context
        internal static SimulationJobHandles ScheduleBuildJacobiansJobs(ref PhysicsWorld world, sfloat timeStep, float3 gravity,
            int numSolverIterations, JobHandle inputDeps, ref NativeList<DispatchPairSequencer.DispatchPair> dispatchPairs,
            ref DispatchPairSequencer.SolverSchedulerInfo solverSchedulerInfo,
            ref NativeStream contacts, ref NativeStream jacobians, bool multiThreaded = true)
        {
            SimulationJobHandles returnHandles = default;

            if (!multiThreaded)
            {
                returnHandles.FinalExecutionHandle = new BuildJacobiansJob
                {
                    ContactsReader = contacts.AsReader(),
                    JacobiansWriter = jacobians.AsWriter(),
                    TimeStep = timeStep,
                    Gravity = gravity,
                    NumSolverIterations = numSolverIterations,
                    World = world,
                    DispatchPairs = dispatchPairs.AsDeferredJobArray()
                }.Schedule(inputDeps);
            }
            else
            {
                var buildJob = new ParallelBuildJacobiansJob
                {
                    ContactsReader = contacts.AsReader(),
                    JacobiansWriter = jacobians.AsWriter(),
                    TimeStep = timeStep,
                    InvTimeStep = timeStep > sfloat.Zero ? sfloat.One / timeStep : sfloat.Zero,
                    GravityAcceleration = math.length(gravity),
                    NumSolverIterations = numSolverIterations,
                    World = world,
                    DispatchPairs = dispatchPairs.AsDeferredJobArray(),
                    SolverSchedulerInfo = solverSchedulerInfo
                };

                JobHandle handle = buildJob.ScheduleUnsafeIndex0(solverSchedulerInfo.NumWorkItems, 1, inputDeps);

                returnHandles.FinalDisposeHandle = JobHandle.CombineDependencies(
                    dispatchPairs.Dispose(handle),
                    contacts.Dispose(handle));
                returnHandles.FinalExecutionHandle = handle;
            }

            return returnHandles;
        }

        // Solve the Jacobians stored in the simulation context
        public static void SolveJacobians(ref NativeStream.Reader jacobiansReader, NativeArray<MotionVelocity> motionVelocities,
            sfloat timeStep, int numIterations, ref NativeStream.Writer collisionEventsWriter, ref NativeStream.Writer triggerEventsWriter,
            StabilizationData solverStabilizationData)
        {
            sfloat invNumIterations = math.rcp((sfloat)numIterations);
            sfloat invTimeStep = timeStep > sfloat.Zero ? sfloat.One / timeStep : sfloat.Zero;
            for (int solverIterationId = 0; solverIterationId < numIterations; solverIterationId++)
            {
                var stepInput = new StepInput
                {
                    InvNumSolverIterations = invNumIterations,
                    IsFirstIteration = solverIterationId == 0,
                    IsLastIteration = solverIterationId == numIterations - 1,
                    Timestep = timeStep,
                    InvTimestep = invTimeStep
                };

                Solve(motionVelocities, ref jacobiansReader, ref collisionEventsWriter, ref triggerEventsWriter, 0, stepInput, solverStabilizationData);

                if (solverStabilizationData.StabilizationHeuristicSettings.EnableSolverStabilization)
                {
                    StabilizeVelocities(motionVelocities, stepInput.IsFirstIteration, timeStep, solverStabilizationData);
                }
            }
        }

        // Schedule jobs to solve the Jacobians stored in the simulation context
        internal static unsafe SimulationJobHandles ScheduleSolveJacobiansJobs(
            ref DynamicsWorld dynamicsWorld, sfloat timestep, int numIterations,
            ref NativeStream jacobians, ref NativeStream collisionEvents, ref NativeStream triggerEvents,
            ref DispatchPairSequencer.SolverSchedulerInfo solverSchedulerInfo,
            StabilizationData solverStabilizationData, JobHandle inputDeps, bool multiThreaded = true)
        {
            SimulationJobHandles returnHandles = default;

            if (!multiThreaded)
            {
                collisionEvents = new NativeStream(1, Allocator.Persistent);
                triggerEvents = new NativeStream(1, Allocator.Persistent);
                returnHandles.FinalExecutionHandle = new SolverJob
                {
                    CollisionEventsWriter = collisionEvents.AsWriter(),
                    JacobiansReader = jacobians.AsReader(),
                    NumIterations = numIterations,
                    Timestep = timestep,
                    TriggerEventsWriter = triggerEvents.AsWriter(),
                    MotionVelocities = dynamicsWorld.MotionVelocities,
                    SolverStabilizationData = solverStabilizationData,
                }.Schedule(inputDeps);

                return returnHandles;
            }

            JobHandle handle;

            int numPhases = solverSchedulerInfo.NumPhases;

            // Use persistent allocator to allow these to live until the start of next step
            {
                NativeArray<int> workItemList = solverSchedulerInfo.NumWorkItems;

                //TODO: Change this to Allocator.TempJob when https://github.com/Unity-Technologies/Unity.Physics/issues/7 is resolved
                JobHandle collisionEventStreamHandle = NativeStream.ScheduleConstruct(out collisionEvents, workItemList, inputDeps, Allocator.Persistent);
                JobHandle triggerEventStreamHandle = NativeStream.ScheduleConstruct(out triggerEvents, workItemList, inputDeps, Allocator.Persistent);

                handle = JobHandle.CombineDependencies(collisionEventStreamHandle, triggerEventStreamHandle);

                sfloat invNumIterations = math.rcp((sfloat)numIterations);

                var phaseInfoPtrs = (DispatchPairSequencer.SolverSchedulerInfo.SolvePhaseInfo*)NativeArrayUnsafeUtility.GetUnsafeBufferPointerWithoutChecks(solverSchedulerInfo.PhaseInfo);

                float3 gravityNormalized = float3.zero;
                if (solverStabilizationData.StabilizationHeuristicSettings.EnableSolverStabilization)
                {
                    gravityNormalized = math.normalizesafe(solverStabilizationData.Gravity);
                }

                for (int solverIterationId = 0; solverIterationId < numIterations; solverIterationId++)
                {
                    bool firstIteration = solverIterationId == 0;
                    bool lastIteration = solverIterationId == numIterations - 1;
                    for (int phaseId = 0; phaseId < numPhases; phaseId++)
                    {
                        var job = new ParallelSolverJob
                        {
                            JacobiansReader = jacobians.AsReader(),
                            PhaseIndex = phaseId,
                            Phases = solverSchedulerInfo.PhaseInfo,
                            MotionVelocities = dynamicsWorld.MotionVelocities,
                            SolverStabilizationData = solverStabilizationData,
                            StepInput = new StepInput
                            {
                                InvNumSolverIterations = invNumIterations,
                                IsFirstIteration = firstIteration,
                                IsLastIteration = lastIteration,
                                Timestep = timestep,
                                InvTimestep = timestep > sfloat.Zero ? sfloat.One / timestep : sfloat.Zero
                            }
                        };

                        // Only initialize event writers for last solver iteration jobs
                        if (lastIteration)
                        {
                            job.CollisionEventsWriter = collisionEvents.AsWriter();
                            job.TriggerEventsWriter = triggerEvents.AsWriter();
                        }

                        // NOTE: The last phase must be executed on a single job since it
                        // int.MaxValue can't be used as batchSize since 19.1 overflows in that case...
                        bool isLastPhase = phaseId == numPhases - 1;
                        int batchSize = isLastPhase ? (int.MaxValue / 2) : 1;

                        int* numWorkItems = &(phaseInfoPtrs[phaseId].NumWorkItems);
                        handle = job.Schedule(numWorkItems, batchSize, handle);
                    }

                    // Stabilize velocities
                    if (solverStabilizationData.StabilizationHeuristicSettings.EnableSolverStabilization)
                    {
                        var stabilizeVelocitiesJob = new StabilizeVelocitiesJob
                        {
                            MotionVelocities = dynamicsWorld.MotionVelocities,
                            SolverStabilizationData = solverStabilizationData,
                            GravityPerStep = solverStabilizationData.Gravity * timestep,
                            GravityNormalized = gravityNormalized,
                            IsFirstIteration = firstIteration
                        };

                        handle = stabilizeVelocitiesJob.Schedule(dynamicsWorld.NumMotions, 64, handle);
                    }
                }
            }

            // Dispose processed data
            returnHandles.FinalDisposeHandle = JobHandle.CombineDependencies(
                jacobians.Dispose(handle),
                solverSchedulerInfo.ScheduleDisposeJob(handle));
            returnHandles.FinalExecutionHandle = handle;

            return returnHandles;
        }

        #region Jobs

        [BurstCompile]
        private struct ParallelApplyGravityAndCopyInputVelocitiesJob : IJobParallelFor
        {
            public NativeArray<MotionVelocity> MotionVelocities;
            public NativeArray<Velocity> InputVelocities;
            public float3 GravityAcceleration;

            public void Execute(int i)
            {
                ExecuteImpl(i, GravityAcceleration, MotionVelocities, InputVelocities);
            }

            internal static void ExecuteImpl(int i, float3 gravityAcceleration,
                NativeArray<MotionVelocity> motionVelocities, NativeArray<Velocity> inputVelocities)
            {
                MotionVelocity motionVelocity = motionVelocities[i];

                // Apply gravity
                motionVelocity.LinearVelocity += gravityAcceleration * motionVelocity.GravityFactor;

                // Write back
                motionVelocities[i] = motionVelocity;

                // Make a copy
                inputVelocities[i] = new Velocity
                {
                    Linear = motionVelocity.LinearVelocity,
                    Angular = motionVelocity.AngularVelocity
                };
            }
        }

        [BurstCompile]
        private struct ApplyGravityAndCopyInputVelocitiesJob : IJob
        {
            public NativeArray<MotionVelocity> MotionVelocities;
            public NativeArray<Velocity> InputVelocities;
            public float3 GravityAcceleration;

            public void Execute()
            {
                ApplyGravityAndCopyInputVelocities(MotionVelocities, InputVelocities, GravityAcceleration);
            }
        }

        [BurstCompile]
        private struct ParallelBuildJacobiansJob : IJobParallelForDefer
        {
            [ReadOnly] public PhysicsWorld World;

            public NativeStream.Reader ContactsReader;
            public NativeStream.Writer JacobiansWriter;
            public sfloat TimeStep;
            [ReadOnly] public NativeArray<DispatchPairSequencer.DispatchPair> DispatchPairs;
            [ReadOnly] public int NumSolverIterations;
            public sfloat InvTimeStep;
            public sfloat GravityAcceleration;
            [ReadOnly] public DispatchPairSequencer.SolverSchedulerInfo SolverSchedulerInfo;

            public void Execute(int workItemIndex)
            {
                int firstDispatchPairIndex = SolverSchedulerInfo.GetWorkItemReadOffset(workItemIndex, out int dispatchPairCount);

                ContactsReader.BeginForEachIndex(workItemIndex);
                JacobiansWriter.BeginForEachIndex(workItemIndex);
                BuildJacobians(ref World, TimeStep, InvTimeStep, GravityAcceleration, NumSolverIterations,
                    DispatchPairs, firstDispatchPairIndex, dispatchPairCount,
                    ref ContactsReader, ref JacobiansWriter);
            }
        }

        [BurstCompile]
        private struct BuildJacobiansJob : IJob
        {
            [ReadOnly] public PhysicsWorld World;

            public NativeStream.Reader ContactsReader;
            public NativeStream.Writer JacobiansWriter;
            public sfloat TimeStep;
            [ReadOnly] public NativeArray<DispatchPairSequencer.DispatchPair> DispatchPairs;
            [ReadOnly] public int NumSolverIterations;
            public float3 Gravity;

            public void Execute()
            {
                BuildJacobians(ref World, TimeStep, Gravity, NumSolverIterations,
                    DispatchPairs, ref ContactsReader, ref JacobiansWriter);
            }
        }

        [BurstCompile]
        [NoAlias]
        private struct ParallelSolverJob : IJobParallelForDefer
        {
            [NativeDisableParallelForRestriction]
            public NativeArray<MotionVelocity> MotionVelocities;

            [NativeDisableParallelForRestriction]
            public StabilizationData SolverStabilizationData;

            [NoAlias]
            public NativeStream.Reader JacobiansReader;

            //@TODO: Unity should have a Allow null safety restriction
            [NativeDisableContainerSafetyRestriction]
            [NoAlias]
            public NativeStream.Writer CollisionEventsWriter;

            //@TODO: Unity should have a Allow null safety restriction
            [NativeDisableContainerSafetyRestriction]
            [NoAlias]
            public NativeStream.Writer TriggerEventsWriter;

            [ReadOnly]
            public NativeArray<DispatchPairSequencer.SolverSchedulerInfo.SolvePhaseInfo> Phases;
            public int PhaseIndex;
            public StepInput StepInput;

            public void Execute(int workItemIndex)
            {
                int workItemStartIndexOffset = Phases[PhaseIndex].FirstWorkItemIndex;

                CollisionEventsWriter.PatchMinMaxRange(workItemIndex + workItemStartIndexOffset);
                TriggerEventsWriter.PatchMinMaxRange(workItemIndex + workItemStartIndexOffset);

                Solve(MotionVelocities, ref JacobiansReader, ref CollisionEventsWriter, ref TriggerEventsWriter,
                    workItemIndex + workItemStartIndexOffset, StepInput, SolverStabilizationData);
            }
        }

        [BurstCompile]
        [NoAlias]
        private struct SolverJob : IJob
        {
            public NativeArray<MotionVelocity> MotionVelocities;

            public StabilizationData SolverStabilizationData;

            [NoAlias]
            public NativeStream.Reader JacobiansReader;

            //@TODO: Unity should have a Allow null safety restriction
            [NativeDisableContainerSafetyRestriction]
            [NoAlias]
            public NativeStream.Writer CollisionEventsWriter;

            //@TODO: Unity should have a Allow null safety restriction
            [NativeDisableContainerSafetyRestriction]
            [NoAlias]
            public NativeStream.Writer TriggerEventsWriter;

            public int NumIterations;
            public sfloat Timestep;

            public void Execute()
            {
                SolveJacobians(ref JacobiansReader, MotionVelocities, Timestep, NumIterations,
                    ref CollisionEventsWriter, ref TriggerEventsWriter, SolverStabilizationData);
            }
        }

        [BurstCompile]
        private struct StabilizeVelocitiesJob : IJobParallelFor
        {
            public NativeArray<MotionVelocity> MotionVelocities;
            public StabilizationData SolverStabilizationData;
            public float3 GravityPerStep;
            public float3 GravityNormalized;
            public bool IsFirstIteration;

            public void Execute(int i)
            {
                ExecuteImpl(i, MotionVelocities, IsFirstIteration, GravityPerStep, GravityNormalized, SolverStabilizationData);
            }

            internal static void ExecuteImpl(int i, NativeArray<MotionVelocity> motionVelocities,
                bool isFirstIteration, float3 gravityPerStep, float3 gravityNormalized,
                StabilizationData solverStabilizationData)
            {
                var motionData = solverStabilizationData.MotionData[i];
                int numPairs = motionData.NumPairs;
                if (numPairs == 0)
                {
                    return;
                }

                MotionVelocity motionVelocity = motionVelocities[i];

                // Skip kinematic bodies
                if (motionVelocity.InverseMass.IsZero())
                {
                    return;
                }

                // Scale up inertia for other iterations
                if (isFirstIteration && numPairs > 1)
                {
                    sfloat inertiaScale = sfloat.One + sfloat.FromRaw(0x3e4ccccd) * (sfloat)(numPairs - 1) * solverStabilizationData.StabilizationHeuristicSettings.InertiaScalingFactor;
                    motionData.InverseInertiaScale = math.rcp(inertiaScale);
                    solverStabilizationData.MotionData[i] = motionData;
                }

                // Don't stabilize velocity component along the gravity vector
                float3 linVelVertical = math.dot(motionVelocity.LinearVelocity, gravityNormalized) * gravityNormalized;
                float3 linVelSideways = motionVelocity.LinearVelocity - linVelVertical;

                // Choose a very small gravity coefficient for clipping threshold
                sfloat gravityCoefficient = (numPairs == 1 ? sfloat.FromRaw(0x3dcccccd) : (sfloat)0.25f) * solverStabilizationData.StabilizationHeuristicSettings.VelocityClippingFactor;

                // Linear velocity threshold
                sfloat smallLinVelThresholdSq = math.lengthsq(gravityPerStep * motionVelocity.GravityFactor * gravityCoefficient);

                // Stabilize the velocities
                if (math.lengthsq(linVelSideways) < smallLinVelThresholdSq)
                {
                    motionVelocity.LinearVelocity = linVelVertical;

                    // Only clip angular if in contact with at least 2 bodies
                    if (numPairs > 1)
                    {
                        // Angular velocity threshold
                        if (motionVelocity.AngularExpansionFactor > sfloat.Zero)
                        {
                            sfloat angularFactorSq = math.rcp(motionVelocity.AngularExpansionFactor * motionVelocity.AngularExpansionFactor) * sfloat.FromRaw(0x3c23d70a);
                            sfloat smallAngVelThresholdSq = smallLinVelThresholdSq * angularFactorSq;
                            if (math.lengthsq(motionVelocity.AngularVelocity) < smallAngVelThresholdSq)
                            {
                                motionVelocity.AngularVelocity = float3.zero;
                            }
                        }
                    }

                    // Write back
                    motionVelocities[i] = motionVelocity;
                }
            }
        }

        #endregion

        #region Implementation

        private static void BuildJacobian(MTransform worldFromA, MTransform worldFromB, float3 normal, float3 armA, float3 armB,
            float3 invInertiaA, float3 invInertiaB, sfloat sumInvMass, out float3 angularA, out float3 angularB, out sfloat invEffectiveMass)
        {
            float3 crossA = math.cross(armA, normal);
            angularA = math.mul(worldFromA.InverseRotation, crossA).xyz;

            float3 crossB = math.cross(normal, armB);
            angularB = math.mul(worldFromB.InverseRotation, crossB).xyz;

            float3 temp = angularA * angularA * invInertiaA + angularB * angularB * invInertiaB;
            invEffectiveMass = temp.x + temp.y + temp.z + sumInvMass;
        }

        private static void BuildContactJacobian(
            int contactPointIndex,
            float3 normal,
            MTransform worldFromA,
            MTransform worldFromB,
            sfloat invTimestep,
            MotionVelocity velocityA,
            MotionVelocity velocityB,
            sfloat sumInvMass,
            sfloat maxDepenetrationVelocity,
            ref JacobianHeader jacobianHeader,
            ref float3 centerA,
            ref float3 centerB,
            ref NativeStream.Reader contactReader)
        {
            ref ContactJacAngAndVelToReachCp jacAngular = ref jacobianHeader.AccessAngularJacobian(contactPointIndex);
            ContactPoint contact = contactReader.Read<ContactPoint>();
            float3 pointOnB = contact.Position;
            float3 pointOnA = contact.Position + normal * contact.Distance;
            float3 armA = pointOnA - worldFromA.Translation;
            float3 armB = pointOnB - worldFromB.Translation;
            BuildJacobian(worldFromA, worldFromB, normal, armA, armB, velocityA.InverseInertia, velocityB.InverseInertia, sumInvMass,
                out jacAngular.Jac.AngularA, out jacAngular.Jac.AngularB, out sfloat invEffectiveMass);
            jacAngular.Jac.EffectiveMass = sfloat.One / invEffectiveMass;
            jacAngular.Jac.Impulse = sfloat.Zero;

            sfloat solveDistance = contact.Distance;
            sfloat solveVelocity = solveDistance * invTimestep;

            solveVelocity = math.max(-maxDepenetrationVelocity, solveVelocity);

            jacAngular.VelToReachCp = -solveVelocity;

            // Calculate average position for friction
            centerA += armA;
            centerB += armB;

            // Write the contact point to the jacobian stream if requested
            if (jacobianHeader.HasContactManifold)
            {
                jacobianHeader.AccessContactPoint(contactPointIndex) = contact;
            }
        }

        private static void InitModifierData(ref JacobianHeader jacobianHeader, ColliderKeyPair colliderKeys, EntityPair entities)
        {
            if (jacobianHeader.HasContactManifold)
            {
                jacobianHeader.AccessColliderKeys() = colliderKeys;
                jacobianHeader.AccessEntities() = entities;
            }
            if (jacobianHeader.HasSurfaceVelocity)
            {
                jacobianHeader.AccessSurfaceVelocity() = new SurfaceVelocity();
            }
            if (jacobianHeader.HasMassFactors)
            {
                jacobianHeader.AccessMassFactors() = MassFactors.Default;
            }
        }

        private static void GetMotions(
            BodyIndexPair pair,
            ref NativeArray<MotionData> motionDatas,
            ref NativeArray<MotionVelocity> motionVelocities,
            out MotionVelocity velocityA,
            out MotionVelocity velocityB,
            out MTransform worldFromA,
            out MTransform worldFromB)
        {
            bool bodyAIsStatic = pair.BodyIndexA >= motionVelocities.Length;
            bool bodyBIsStatic = pair.BodyIndexB >= motionVelocities.Length;

            if (bodyAIsStatic)
            {
                if (bodyBIsStatic)
                {
                    Assert.IsTrue(false); // static-static pairs should have been filtered during broadphase overlap test
                    velocityA = MotionVelocity.Zero;
                    velocityB = MotionVelocity.Zero;
                    worldFromA = MTransform.Identity;
                    worldFromB = MTransform.Identity;
                    return;
                }

                velocityA = MotionVelocity.Zero;
                velocityB = motionVelocities[pair.BodyIndexB];

                worldFromA = MTransform.Identity;
                worldFromB = new MTransform(motionDatas[pair.BodyIndexB].WorldFromMotion);
            }
            else if (bodyBIsStatic)
            {
                velocityA = motionVelocities[pair.BodyIndexA];
                velocityB = MotionVelocity.Zero;

                worldFromA = new MTransform(motionDatas[pair.BodyIndexA].WorldFromMotion);
                worldFromB = MTransform.Identity;
            }
            else
            {
                velocityA = motionVelocities[pair.BodyIndexA];
                velocityB = motionVelocities[pair.BodyIndexB];

                worldFromA = new MTransform(motionDatas[pair.BodyIndexA].WorldFromMotion);
                worldFromB = new MTransform(motionDatas[pair.BodyIndexB].WorldFromMotion);
            }
        }

        // Gets a body's motion, even if the body is static
        // TODO - share code with GetMotions()?
        private static void GetMotion([NoAlias] ref PhysicsWorld world, int bodyIndex,
            [NoAlias] out MotionVelocity velocity, [NoAlias] out MotionData motion)
        {
            if (bodyIndex >= world.MotionVelocities.Length)
            {
                // Body is static
                velocity = MotionVelocity.Zero;
                motion = new MotionData
                {
                    WorldFromMotion = world.Bodies[bodyIndex].WorldFromBody,
                    BodyFromMotion = RigidTransform.identity
                    // remaining fields all zero
                };
            }
            else
            {
                // Body is dynamic
                velocity = world.MotionVelocities[bodyIndex];
                motion = world.MotionDatas[bodyIndex];
            }
        }

        private static unsafe void BuildJacobians(
            ref PhysicsWorld world,
            sfloat timestep,
            sfloat invTimestep,
            sfloat gravityAcceleration,
            int numSolverIterations,
            NativeArray<DispatchPairSequencer.DispatchPair> dispatchPairs,
            int firstDispatchPairIndex,
            int dispatchPairCount,
            ref NativeStream.Reader contactReader,
            ref NativeStream.Writer jacobianWriter)
        {
            // Contact resting velocity for restitution
            sfloat negContactRestingVelocity = -gravityAcceleration * timestep;

            for (int i = 0; i < dispatchPairCount; i++)
            {
                var pair = dispatchPairs[i + firstDispatchPairIndex];
                if (!pair.IsValid)
                {
                    continue;
                }

                var motionDatas = world.MotionDatas;
                var motionVelocities = world.MotionVelocities;
                var bodies = world.Bodies;

                if (pair.IsContact)
                {
                    while (contactReader.RemainingItemCount > 0)
                    {
                        // Check if this is the matching contact
                        {
                            var header = contactReader.Peek<ContactHeader>();
                            if (pair.BodyIndexA != header.BodyPair.BodyIndexA ||
                                pair.BodyIndexB != header.BodyPair.BodyIndexB)
                            {
                                break;
                            }
                        }

                        ref ContactHeader contactHeader = ref contactReader.Read<ContactHeader>();
                        GetMotions(contactHeader.BodyPair, ref motionDatas, ref motionVelocities, out MotionVelocity velocityA, out MotionVelocity velocityB, out MTransform worldFromA, out MTransform worldFromB);

                        sfloat sumInvMass = velocityA.InverseMass + velocityB.InverseMass;
                        bool bodiesHaveInfiniteMass = velocityA.HasInfiniteInertiaAndMass && velocityB.HasInfiniteInertiaAndMass;

                        // Skip contact between infinite mass bodies which don't want to raise events. These cannot have any effect during solving.
                        // These should not normally appear, because the collision detector doesn't generate such contacts.
                        if (bodiesHaveInfiniteMass)
                        {
                            if ((contactHeader.JacobianFlags & (JacobianFlags.IsTrigger | JacobianFlags.EnableCollisionEvents)) == 0)
                            {
                                for (int j = 0; j < contactHeader.NumContacts; j++)
                                {
                                    contactReader.Read<ContactPoint>();
                                }
                                continue;
                            }
                        }

                        JacobianType jacType = ((int)(contactHeader.JacobianFlags) & (int)(JacobianFlags.IsTrigger)) != 0 ?
                            JacobianType.Trigger : JacobianType.Contact;

                        // Write size before every jacobian and allocate all necessary data for this jacobian
                        int jacobianSize = JacobianHeader.CalculateSize(jacType, contactHeader.JacobianFlags, contactHeader.NumContacts);
                        jacobianWriter.Write(jacobianSize);
                        byte* jacobianPtr = jacobianWriter.Allocate(jacobianSize);

#if DEVELOPMENT_BUILD
                        SafetyChecks.Check4ByteAlignmentAndThrow(jacobianPtr, nameof(jacobianPtr));
#endif
                        ref JacobianHeader jacobianHeader = ref UnsafeUtility.AsRef<JacobianHeader>(jacobianPtr);
                        jacobianHeader.BodyPair = contactHeader.BodyPair;
                        jacobianHeader.Type = jacType;
                        jacobianHeader.Flags = contactHeader.JacobianFlags;

                        var baseJac = new BaseContactJacobian
                        {
                            NumContacts = contactHeader.NumContacts,
                            Normal = contactHeader.Normal
                        };

                        // Body A must be dynamic
                        Assert.IsTrue(contactHeader.BodyPair.BodyIndexA < motionVelocities.Length);
                        bool isDynamicStaticPair = contactHeader.BodyPair.BodyIndexB >= motionVelocities.Length;

                        // If contact distance is negative, use an artificially reduced penetration depth to prevent the dynamic-dynamic contacts from depenetrating too quickly
                        sfloat maxDepenetrationVelocity = isDynamicStaticPair ? sfloat.MaxValue : (sfloat)3.0f; // meter/seconds time step independent

                        if (jacobianHeader.Type == JacobianType.Contact)
                        {
                            ref ContactJacobian contactJacobian = ref jacobianHeader.AccessBaseJacobian<ContactJacobian>();
                            contactJacobian.BaseJacobian = baseJac;
                            contactJacobian.CoefficientOfFriction = contactHeader.CoefficientOfFriction;

                            // Indicator whether restitution will be applied,
                            // used to scale down friction on bounce.
                            bool applyRestitution = false;

                            // Initialize modifier data (in order from JacobianModifierFlags) before angular jacobians
                            InitModifierData(ref jacobianHeader, contactHeader.ColliderKeys,
                                new EntityPair { EntityA = bodies[contactHeader.BodyPair.BodyIndexA].Entity, EntityB = bodies[contactHeader.BodyPair.BodyIndexB].Entity });

                            // Build normal jacobians
                            var centerA = new float3(sfloat.Zero);
                            var centerB = new float3(sfloat.Zero);
                            for (int j = 0; j < contactHeader.NumContacts; j++)
                            {
                                // Build the jacobian
                                BuildContactJacobian(
                                    j, contactJacobian.BaseJacobian.Normal, worldFromA, worldFromB, invTimestep, velocityA, velocityB, sumInvMass, maxDepenetrationVelocity,
                                    ref jacobianHeader, ref centerA, ref centerB, ref contactReader);

                                // Restitution (optional)
                                if (contactHeader.CoefficientOfRestitution > sfloat.Zero)
                                {
                                    ref ContactJacAngAndVelToReachCp jacAngular = ref jacobianHeader.AccessAngularJacobian(j);
                                    sfloat relativeVelocity = BaseContactJacobian.GetJacVelocity(baseJac.Normal, jacAngular.Jac,
                                        velocityA.LinearVelocity, velocityA.AngularVelocity, velocityB.LinearVelocity, velocityB.AngularVelocity);
                                    sfloat dv = jacAngular.VelToReachCp - relativeVelocity;
                                    if (dv > sfloat.Zero && relativeVelocity < negContactRestingVelocity)
                                    {
                                        // Restitution impulse is applied as if contact point is on the contact plane.
                                        // However, it can (and will) be slightly away from contact plane at the moment restitution is applied.
                                        // So we have to apply vertical shot equation to make sure we don't gain energy:
                                        // effectiveRestitutionVelocity^2 = restitutionVelocity^2 - 2.0f * gravityAcceleration * distanceToGround
                                        // From this formula we calculate the effective restitution velocity, which is the velocity
                                        // that the contact point needs to reach the same height from current position
                                        // as if it was shot with the restitutionVelocity from the contact plane.
                                        // ------------------------------------------------------------
                                        // This is still an approximation for 2 reasons:
                                        // - We are assuming the contact point will hit the contact plane with its current velocity,
                                        // while actually it would have a portion of gravity applied before the actual hit. However,
                                        // that velocity increase is quite small (less than gravity in one step), so it's safe
                                        // to use current velocity instead.
                                        // - gravityAcceleration is the actual value of gravity applied only when contact plane is
                                        // directly opposite to gravity direction. Otherwise, this value will only be smaller.
                                        // However, since this can only result in smaller bounce than the "correct" one, we can
                                        // safely go with the default gravity value in all cases.
                                        sfloat restitutionVelocity = (relativeVelocity - negContactRestingVelocity) * contactHeader.CoefficientOfRestitution;
                                        sfloat distanceToGround = math.max(-jacAngular.VelToReachCp * timestep, sfloat.Zero);
                                        sfloat effectiveRestitutionVelocity =
                                            math.sqrt(math.max(restitutionVelocity * restitutionVelocity - (sfloat)2.0f * gravityAcceleration * distanceToGround, sfloat.Zero));

                                        jacAngular.VelToReachCp =
                                            math.max(jacAngular.VelToReachCp - effectiveRestitutionVelocity, sfloat.Zero) +
                                            effectiveRestitutionVelocity;

                                        // Remember that restitution should be applied
                                        applyRestitution = true;
                                    }
                                }
                            }

                            // Build friction jacobians
                            // (skip friction between two infinite-mass objects)
                            if (!bodiesHaveInfiniteMass)
                            {
                                // Clear accumulated impulse
                                contactJacobian.Friction0.Impulse = sfloat.Zero;
                                contactJacobian.Friction1.Impulse = sfloat.Zero;
                                contactJacobian.AngularFriction.Impulse = sfloat.Zero;

                                // Calculate average position
                                sfloat invNumContacts = math.rcp((sfloat)contactJacobian.BaseJacobian.NumContacts);
                                centerA *= invNumContacts;
                                centerB *= invNumContacts;

                                // Choose friction axes
                                CalculatePerpendicularNormalized(contactJacobian.BaseJacobian.Normal, out float3 frictionDir0, out float3 frictionDir1);

                                // Build linear jacobian
                                sfloat invEffectiveMass0, invEffectiveMass1;
                                {
                                    float3 armA = centerA;
                                    float3 armB = centerB;
                                    BuildJacobian(worldFromA, worldFromB, frictionDir0, armA, armB, velocityA.InverseInertia, velocityB.InverseInertia, sumInvMass,
                                        out contactJacobian.Friction0.AngularA, out contactJacobian.Friction0.AngularB, out invEffectiveMass0);
                                    BuildJacobian(worldFromA, worldFromB, frictionDir1, armA, armB, velocityA.InverseInertia, velocityB.InverseInertia, sumInvMass,
                                        out contactJacobian.Friction1.AngularA, out contactJacobian.Friction1.AngularB, out invEffectiveMass1);
                                }

                                // Build angular jacobian
                                sfloat invEffectiveMassAngular;
                                {
                                    contactJacobian.AngularFriction.AngularA = math.mul(worldFromA.InverseRotation, contactJacobian.BaseJacobian.Normal);
                                    contactJacobian.AngularFriction.AngularB = math.mul(worldFromB.InverseRotation, -contactJacobian.BaseJacobian.Normal);
                                    float3 temp = contactJacobian.AngularFriction.AngularA * contactJacobian.AngularFriction.AngularA * velocityA.InverseInertia;
                                    temp += contactJacobian.AngularFriction.AngularB * contactJacobian.AngularFriction.AngularB * velocityB.InverseInertia;
                                    invEffectiveMassAngular = math.csum(temp);
                                }

                                // Build effective mass
                                {
                                    // Build the inverse effective mass matrix
                                    var invEffectiveMassDiag = new float3(invEffectiveMass0, invEffectiveMass1, invEffectiveMassAngular);
                                    var invEffectiveMassOffDiag = new float3( // (0, 1), (0, 2), (1, 2)
                                        JacobianUtilities.CalculateInvEffectiveMassOffDiag(contactJacobian.Friction0.AngularA, contactJacobian.Friction1.AngularA, velocityA.InverseInertia,
                                            contactJacobian.Friction0.AngularB, contactJacobian.Friction1.AngularB, velocityB.InverseInertia),
                                        JacobianUtilities.CalculateInvEffectiveMassOffDiag(contactJacobian.Friction0.AngularA, contactJacobian.AngularFriction.AngularA, velocityA.InverseInertia,
                                            contactJacobian.Friction0.AngularB, contactJacobian.AngularFriction.AngularB, velocityB.InverseInertia),
                                        JacobianUtilities.CalculateInvEffectiveMassOffDiag(contactJacobian.Friction1.AngularA, contactJacobian.AngularFriction.AngularA, velocityA.InverseInertia,
                                            contactJacobian.Friction1.AngularB, contactJacobian.AngularFriction.AngularB, velocityB.InverseInertia));

                                    // Invert the matrix and store it to the jacobians
                                    if (!JacobianUtilities.InvertSymmetricMatrix(invEffectiveMassDiag, invEffectiveMassOffDiag, out float3 effectiveMassDiag, out float3 effectiveMassOffDiag))
                                    {
                                        // invEffectiveMass can be singular if the bodies have infinite inertia about the normal.
                                        // In that case angular friction does nothing so we can regularize the matrix, set col2 = row2 = (0, 0, 1)
                                        invEffectiveMassOffDiag.y = sfloat.Zero;
                                        invEffectiveMassOffDiag.z = sfloat.Zero;
                                        invEffectiveMassDiag.z = sfloat.One;
                                        bool success = JacobianUtilities.InvertSymmetricMatrix(invEffectiveMassDiag, invEffectiveMassOffDiag, out effectiveMassDiag, out effectiveMassOffDiag);
                                        Assert.IsTrue(success); // it should never fail, if it does then friction will be disabled
                                    }
                                    contactJacobian.Friction0.EffectiveMass = effectiveMassDiag.x;
                                    contactJacobian.Friction1.EffectiveMass = effectiveMassDiag.y;
                                    contactJacobian.AngularFriction.EffectiveMass = effectiveMassDiag.z;
                                    contactJacobian.FrictionEffectiveMassOffDiag = effectiveMassOffDiag;
                                }

                                // Reduce friction to 1/4 of the impulse if there will be restitution
                                if (applyRestitution)
                                {
                                    contactJacobian.Friction0.EffectiveMass *= (sfloat)0.25f;
                                    contactJacobian.Friction1.EffectiveMass *= (sfloat)0.25f;
                                    contactJacobian.AngularFriction.EffectiveMass *= (sfloat)0.25f;
                                    contactJacobian.FrictionEffectiveMassOffDiag *= (sfloat)0.25f;
                                }
                            }
                        }
                        // Much less data needed for triggers
                        else
                        {
                            ref TriggerJacobian triggerJacobian = ref jacobianHeader.AccessBaseJacobian<TriggerJacobian>();

                            triggerJacobian.BaseJacobian = baseJac;
                            triggerJacobian.ColliderKeys = contactHeader.ColliderKeys;
                            triggerJacobian.Entities = new EntityPair
                            {
                                EntityA = bodies[contactHeader.BodyPair.BodyIndexA].Entity,
                                EntityB = bodies[contactHeader.BodyPair.BodyIndexB].Entity
                            };

                            // Build normal jacobians
                            var centerA = new float3(sfloat.Zero);
                            var centerB = new float3(sfloat.Zero);
                            for (int j = 0; j < contactHeader.NumContacts; j++)
                            {
                                // Build the jacobian
                                BuildContactJacobian(
                                    j, triggerJacobian.BaseJacobian.Normal, worldFromA, worldFromB, invTimestep, velocityA, velocityB, sumInvMass, maxDepenetrationVelocity,
                                    ref jacobianHeader, ref centerA, ref centerB, ref contactReader);
                            }
                        }
                    }
                }
                else
                {
                    Joint joint = world.Joints[pair.JointIndex];
                    // Need to fetch the real body indices from the joint, as the scheduler may have reordered them
                    int bodyIndexA = joint.BodyPair.BodyIndexA;
                    int bodyIndexB = joint.BodyPair.BodyIndexB;

                    GetMotion(ref world, bodyIndexA, out MotionVelocity velocityA, out MotionData motionA);
                    GetMotion(ref world, bodyIndexB, out MotionVelocity velocityB, out MotionData motionB);

                    BuildJointJacobian(joint, velocityA, velocityB,
                        motionA, motionB, timestep, numSolverIterations, ref jacobianWriter);
                }
            }

            contactReader.EndForEachIndex();
            jacobianWriter.EndForEachIndex();
        }

        internal static unsafe void BuildJointJacobian(Joint joint,
            MotionVelocity velocityA, MotionVelocity velocityB, MotionData motionA, MotionData motionB,
            sfloat timestep, int numIterations, [NoAlias] ref NativeStream.Writer jacobianWriter)
        {
            var bodyAFromMotionA = new MTransform(motionA.BodyFromMotion);
            MTransform motionAFromJoint = Mul(Inverse(bodyAFromMotionA), joint.AFromJoint);

            var bodyBFromMotionB = new MTransform(motionB.BodyFromMotion);
            MTransform motionBFromJoint = Mul(Inverse(bodyBFromMotionB), joint.BFromJoint);

            for (var i = 0; i < joint.Constraints.Length; i++)
            {
                Constraint constraint = joint.Constraints[i];
                int constraintDimension = constraint.Dimension;
                if (0 == constraintDimension)
                {
                    // Unconstrained, so no need to create a header.
                    continue;
                }

                JacobianType jacType;
                switch (constraint.Type)
                {
                    case ConstraintType.Linear:
                        jacType = JacobianType.LinearLimit;
                        break;
                    case ConstraintType.Angular:
                        switch (constraintDimension)
                        {
                            case 1:
                                jacType = JacobianType.AngularLimit1D;
                                break;
                            case 2:
                                jacType = JacobianType.AngularLimit2D;
                                break;
                            case 3:
                                jacType = JacobianType.AngularLimit3D;
                                break;
                            default:
                                SafetyChecks.ThrowNotImplementedException();
                                return;
                        }
                        break;
                    default:
                        SafetyChecks.ThrowNotImplementedException();
                        return;
                }

                // Write size before every jacobian
                JacobianFlags jacFlags = 0;
                int jacobianSize = JacobianHeader.CalculateSize(jacType, jacFlags);
                jacobianWriter.Write(jacobianSize);

                // Allocate all necessary data for this jacobian
                byte* jacobianPtr = jacobianWriter.Allocate(jacobianSize);
#if DEVELOPMENT_BUILD
                SafetyChecks.Check4ByteAlignmentAndThrow(jacobianPtr, nameof(jacobianPtr));
#endif
                ref JacobianHeader header = ref UnsafeUtility.AsRef<JacobianHeader>(jacobianPtr);
                header.BodyPair = joint.BodyPair;
                header.Type = jacType;
                header.Flags = jacFlags;

                JacobianUtilities.CalculateTauAndDamping(constraint, timestep, numIterations, out sfloat tau, out sfloat damping);

                // Build the Jacobian
                switch (constraint.Type)
                {
                    case ConstraintType.Linear:
                        header.AccessBaseJacobian<LinearLimitJacobian>().Build(
                            motionAFromJoint, motionBFromJoint,
                            velocityA, velocityB, motionA, motionB, constraint, tau, damping);
                        break;
                    case ConstraintType.Angular:
                        switch (constraintDimension)
                        {
                            case 1:
                                header.AccessBaseJacobian<AngularLimit1DJacobian>().Build(
                                    motionAFromJoint, motionBFromJoint,
                                    velocityA, velocityB, motionA, motionB, constraint, tau, damping);
                                break;
                            case 2:
                                header.AccessBaseJacobian<AngularLimit2DJacobian>().Build(
                                    motionAFromJoint, motionBFromJoint,
                                    velocityA, velocityB, motionA, motionB, constraint, tau, damping);
                                break;
                            case 3:
                                header.AccessBaseJacobian<AngularLimit3DJacobian>().Build(
                                    motionAFromJoint, motionBFromJoint,
                                    velocityA, velocityB, motionA, motionB, constraint, tau, damping);
                                break;
                            default:
                                SafetyChecks.ThrowNotImplementedException();
                                return;
                        }
                        break;
                    default:
                        SafetyChecks.ThrowNotImplementedException();
                        return;
                }
            }
        }

        // Updates data for solver stabilization heuristic.
        // Updates number of pairs for dynamic bodies and resets inverse inertia scale in first iteration.
        // Also prepares motion stabilization solver data for current Jacobian to solve.
        private static void SolverStabilizationUpdate(
            ref JacobianHeader header, bool isFirstIteration,
            MotionVelocity velocityA, MotionVelocity velocityB,
            StabilizationData solverStabilizationData,
            ref MotionStabilizationInput motionStabilizationSolverInputA,
            ref MotionStabilizationInput motionStabilizationSolverInputB)
        {
            // Solver stabilization heuristic, count pairs and reset inverse inertia scale only in first iteration
            var inputVelocities = solverStabilizationData.InputVelocities;
            var motionData = solverStabilizationData.MotionData;
            if (isFirstIteration)
            {
                // Only count heavier (or up to 2 times lighter) bodies as pairs
                // Also reset inverse inertia scale
                if (header.BodyPair.BodyIndexA < motionData.Length)
                {
                    var data = motionData[header.BodyPair.BodyIndexA];
                    if ((sfloat)0.5f * velocityB.InverseMass <= velocityA.InverseMass)
                    {
                        data.NumPairs++;
                    }
                    data.InverseInertiaScale = sfloat.One;
                    motionData[header.BodyPair.BodyIndexA] = data;
                }
                if (header.BodyPair.BodyIndexB < motionData.Length)
                {
                    var data = motionData[header.BodyPair.BodyIndexB];
                    if ((sfloat)0.5f * velocityA.InverseMass <= velocityB.InverseMass)
                    {
                        data.NumPairs++;
                    }
                    data.InverseInertiaScale = sfloat.One;
                    motionData[header.BodyPair.BodyIndexB] = data;
                }
            }

            // Motion solver input stabilization data
            {
                if (solverStabilizationData.StabilizationHeuristicSettings.EnableFrictionVelocities)
                {
                    motionStabilizationSolverInputA.InputVelocity = header.BodyPair.BodyIndexA < inputVelocities.Length ?
                        inputVelocities[header.BodyPair.BodyIndexA] : Velocity.Zero;
                    motionStabilizationSolverInputB.InputVelocity = header.BodyPair.BodyIndexB < inputVelocities.Length ?
                        inputVelocities[header.BodyPair.BodyIndexB] : Velocity.Zero;
                }

                motionStabilizationSolverInputA.InverseInertiaScale = header.BodyPair.BodyIndexA < motionData.Length ?
                    motionData[header.BodyPair.BodyIndexA].InverseInertiaScale : sfloat.One;
                motionStabilizationSolverInputB.InverseInertiaScale = header.BodyPair.BodyIndexB < motionData.Length ?
                    motionData[header.BodyPair.BodyIndexB].InverseInertiaScale : sfloat.One;
            }
        }

        private static void Solve(
            NativeArray<MotionVelocity> motionVelocities,
            [NoAlias] ref NativeStream.Reader jacobianReader,
            [NoAlias] ref NativeStream.Writer collisionEventsWriter,
            [NoAlias] ref NativeStream.Writer triggerEventsWriter,
            int workItemIndex, StepInput stepInput,
            StabilizationData solverStabilizationData)
        {
            if (stepInput.IsLastIteration)
            {
                collisionEventsWriter.BeginForEachIndex(workItemIndex);
                triggerEventsWriter.BeginForEachIndex(workItemIndex);
            }

            MotionStabilizationInput motionStabilizationSolverInputA = MotionStabilizationInput.Default;
            MotionStabilizationInput motionStabilizationSolverInputB = MotionStabilizationInput.Default;

            var jacIterator = new JacobianIterator(jacobianReader, workItemIndex);
            while (jacIterator.HasJacobiansLeft())
            {
                ref JacobianHeader header = ref jacIterator.ReadJacobianHeader();

                // Static-static pairs should have been filtered during broadphase overlap test
                Assert.IsTrue(header.BodyPair.BodyIndexA < motionVelocities.Length || header.BodyPair.BodyIndexB < motionVelocities.Length);

                // Get the motion pair
                MotionVelocity velocityA = header.BodyPair.BodyIndexA < motionVelocities.Length ? motionVelocities[header.BodyPair.BodyIndexA] : MotionVelocity.Zero;
                MotionVelocity velocityB = header.BodyPair.BodyIndexB < motionVelocities.Length ? motionVelocities[header.BodyPair.BodyIndexB] : MotionVelocity.Zero;

                // Solver stabilization
                if (solverStabilizationData.StabilizationHeuristicSettings.EnableSolverStabilization && header.Type == JacobianType.Contact)
                {
                    SolverStabilizationUpdate(ref header, stepInput.IsFirstIteration, velocityA, velocityB, solverStabilizationData,
                        ref motionStabilizationSolverInputA, ref motionStabilizationSolverInputB);
                }

                // Solve the jacobian
                header.Solve(ref velocityA, ref velocityB, stepInput, ref collisionEventsWriter, ref triggerEventsWriter,
                    solverStabilizationData.StabilizationHeuristicSettings.EnableSolverStabilization &&
                    solverStabilizationData.StabilizationHeuristicSettings.EnableFrictionVelocities,
                    motionStabilizationSolverInputA, motionStabilizationSolverInputB);

                // Write back velocity for dynamic bodies
                if (header.BodyPair.BodyIndexA < motionVelocities.Length)
                {
                    motionVelocities[header.BodyPair.BodyIndexA] = velocityA;
                }
                if (header.BodyPair.BodyIndexB < motionVelocities.Length)
                {
                    motionVelocities[header.BodyPair.BodyIndexB] = velocityB;
                }
            }

            if (stepInput.IsLastIteration)
            {
                collisionEventsWriter.EndForEachIndex();
                triggerEventsWriter.EndForEachIndex();
            }
        }

        private static void StabilizeVelocities(NativeArray<MotionVelocity> motionVelocities,
            bool isFirstIteration, sfloat timeStep, StabilizationData solverStabilizationData)
        {
            float3 gravityPerStep = solverStabilizationData.Gravity * timeStep;
            float3 gravityNormalized = math.normalizesafe(solverStabilizationData.Gravity);
            for (int i = 0; i < motionVelocities.Length; i++)
            {
                StabilizeVelocitiesJob.ExecuteImpl(i, motionVelocities, isFirstIteration,
                    gravityPerStep, gravityNormalized, solverStabilizationData);
            }
        }

        #endregion
    }
}
