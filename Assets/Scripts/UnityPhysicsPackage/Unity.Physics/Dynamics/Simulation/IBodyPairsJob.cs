using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;

namespace UnityS.Physics
{
    // INTERNAL UnityPhysics interface for jobs that iterate through the list of potentially overlapping body pairs produced by the broad phase
    // Important: Only use inside UnityPhysics code! Jobs in other projects should implement IBodyPairsJob.
    [JobProducerType(typeof(IBodyPairsJobExtensions.BodyPairsJobProcess<>))]
    public interface IBodyPairsJobBase
    {
        void Execute(ref ModifiableBodyPair pair);
    }

#if !HAVOK_PHYSICS_EXISTS

    // Interface for jobs that iterate through the list of potentially overlapping body pairs produced by the broad phase
    public interface IBodyPairsJob : IBodyPairsJobBase
    {
    }

#endif

    public struct ModifiableBodyPair
    {
        internal EntityPair EntityPair;
        internal BodyIndexPair BodyIndexPair;

        public Entity EntityB => EntityPair.EntityB;
        public Entity EntityA => EntityPair.EntityA;
        public int BodyIndexB => BodyIndexPair.BodyIndexB;
        public int BodyIndexA => BodyIndexPair.BodyIndexA;

        public void Disable()
        {
            BodyIndexPair = BodyIndexPair.Invalid;
        }
    }

    public static class IBodyPairsJobExtensions
    {
#if !HAVOK_PHYSICS_EXISTS
        // Default Schedule() implementation for IBodyPairsJob
        public static unsafe JobHandle Schedule<T>(this T jobData, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps)
            where T : struct, IBodyPairsJobBase
        {
            // Should work only for UnityPhysics
            if (simulation.Type != SimulationType.UnityPhysics)
            {
                return inputDeps;
            }

            return ScheduleUnityPhysicsBodyPairsJob(jobData, simulation, ref world, inputDeps);
        }

#else
        // In this case Schedule() implementation for IBodyPairsJob is provided by the Havok.Physics assembly.
        // This is a stub to catch when that assembly is missing.
        //<todo.eoin.modifier Put in a link to documentation for this
        [Obsolete("This error occurs when HAVOK_PHYSICS_EXISTS is defined but Havok.Physics is missing from your package's asmdef references. (DoNotRemove)", true)]
        public static unsafe JobHandle Schedule<T>(this T jobData, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps,
            HAVOK_PHYSICS_MISSING_FROM_ASMDEF _causeCompileError = HAVOK_PHYSICS_MISSING_FROM_ASMDEF.HAVOK_PHYSICS_MISSING_FROM_ASMDEF)
            where T : struct, IBodyPairsJobBase
        {
            return new JobHandle();
        }

        public enum HAVOK_PHYSICS_MISSING_FROM_ASMDEF
        {
            HAVOK_PHYSICS_MISSING_FROM_ASMDEF
        }
#endif

        internal static unsafe JobHandle ScheduleUnityPhysicsBodyPairsJob<T>(T jobData, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps)
            where T : struct, IBodyPairsJobBase
        {
            SafetyChecks.CheckAreEqualAndThrow(SimulationType.UnityPhysics, simulation.Type);

            var data = new BodyPairsJobData<T>
            {
                UserJobData = jobData,
                PhasedDispatchPairs = ((Simulation)simulation).StepContext.PhasedDispatchPairs.AsDeferredJobArray(),
                Bodies = world.Bodies
            };
            var parameters = new JobsUtility.JobScheduleParameters(
#if UNITY_2020_2_OR_NEWER
                UnsafeUtility.AddressOf(ref data), BodyPairsJobProcess<T>.Initialize(), inputDeps, ScheduleMode.Single);
#else
                UnsafeUtility.AddressOf(ref data), BodyPairsJobProcess<T>.Initialize(), inputDeps, ScheduleMode.Batched);
#endif
            return JobsUtility.Schedule(ref parameters);
        }

        internal struct BodyPairsJobData<T> where T : struct
        {
            public T UserJobData;
            public NativeArray<DispatchPairSequencer.DispatchPair> PhasedDispatchPairs;
            //Need to disable aliasing restriction in case T has a NativeArray of PhysicsWorld.Bodies:
            [ReadOnly] [NativeDisableContainerSafetyRestriction] public NativeArray<RigidBody> Bodies;
        }

        internal struct BodyPairsJobProcess<T> where T : struct, IBodyPairsJobBase
        {
            static IntPtr jobReflectionData;

            public static IntPtr Initialize()
            {
                if (jobReflectionData == IntPtr.Zero)
                {
#if UNITY_2020_2_OR_NEWER
                    jobReflectionData = JobsUtility.CreateJobReflectionData(typeof(BodyPairsJobData<T>), typeof(T), (ExecuteJobFunction)Execute);
#else
                    jobReflectionData = JobsUtility.CreateJobReflectionData(typeof(BodyPairsJobData<T>), typeof(T), JobType.Single, (ExecuteJobFunction)Execute);
#endif
                }
                return jobReflectionData;
            }

            public delegate void ExecuteJobFunction(ref BodyPairsJobData<T> jobData, IntPtr additionalData,
                IntPtr bufferRangePatchData, ref JobRanges ranges, int jobIndex);

            public unsafe static void Execute(ref BodyPairsJobData<T> jobData, IntPtr additionalData,
                IntPtr bufferRangePatchData, ref JobRanges ranges, int jobIndex)
            {
                for (int currentIdx = 0; currentIdx < jobData.PhasedDispatchPairs.Length; currentIdx++)
                {
                    DispatchPairSequencer.DispatchPair dispatchPair = jobData.PhasedDispatchPairs[currentIdx];

                    // Skip joint pairs and invalid pairs
                    if (dispatchPair.IsJoint || !dispatchPair.IsValid)
                    {
                        continue;
                    }

                    var pair = new ModifiableBodyPair
                    {
                        BodyIndexPair = new BodyIndexPair { BodyIndexA = dispatchPair.BodyIndexA, BodyIndexB = dispatchPair.BodyIndexB },
                        EntityPair = new EntityPair
                        {
                            EntityA = jobData.Bodies[dispatchPair.BodyIndexA].Entity,
                            EntityB = jobData.Bodies[dispatchPair.BodyIndexB].Entity
                        }
                    };

                    jobData.UserJobData.Execute(ref pair);

                    if (pair.BodyIndexA == -1 || pair.BodyIndexB == -1)
                    {
                        jobData.PhasedDispatchPairs[currentIdx] = DispatchPairSequencer.DispatchPair.Invalid;
                    }
                }
            }
        }
    }
}
