using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;
using UnityS.Mathematics;

namespace UnityS.Physics
{
    // INTERNAL UnityPhysics interface for jobs that iterate through the list of Jacobians before they are solved
    // Important: Only use inside UnityPhysics code! Jobs in other projects should implement IJacobiansJob.
    [JobProducerType(typeof(IJacobiansJobExtensions.JacobiansJobProcess<>))]
    public interface IJacobiansJobBase
    {
        // Note, multiple Jacobians can share the same header.
        void Execute(ref ModifiableJacobianHeader header, ref ModifiableContactJacobian jacobian);
        void Execute(ref ModifiableJacobianHeader header, ref ModifiableTriggerJacobian jacobian);
    }

#if !HAVOK_PHYSICS_EXISTS

    // Interface for jobs that iterate through the list of Jacobians before they are solved
    public interface IJacobiansJob : IJacobiansJobBase
    {
    }

#endif

    public unsafe struct ModifiableJacobianHeader
    {
        internal JacobianHeader* m_Header;
        public bool ModifiersChanged { get; private set; }
        public bool AngularChanged { get; private set; }

        internal EntityPair EntityPair;

        public Entity EntityB => EntityPair.EntityB;
        public Entity EntityA => EntityPair.EntityA;

        public int BodyIndexB => m_Header->BodyPair.BodyIndexB;
        public int BodyIndexA => m_Header->BodyPair.BodyIndexA;

        public JacobianType Type => m_Header->Type;
        public JacobianFlags Flags
        {
            get => m_Header->Flags;
            set
            {
                // Some flags change the size of the jacobian; don't allow these to be changed:
                byte notPermitted = (byte)(JacobianFlags.EnableSurfaceVelocity | JacobianFlags.EnableMassFactors | JacobianFlags.EnableMassFactors);
                byte userFlags = (byte)value;
                byte alreadySet = (byte)m_Header->Flags;

                if ((notPermitted & (userFlags ^ alreadySet)) != 0)
                {
                    SafetyChecks.ThrowNotSupportedException("Cannot change flags which alter jacobian size");
                    return;
                }

                m_Header->Flags = value;
            }
        }
        public bool HasColliderKeys => m_Header->HasContactManifold;
        public ColliderKey ColliderKeyB => m_Header->ColliderKeys.ColliderKeyB;
        public ColliderKey ColliderKeyA => m_Header->ColliderKeys.ColliderKeyA;

        public bool HasMassFactors => m_Header->HasMassFactors;
        public MassFactors MassFactors
        {
            get => m_Header->MassFactors;
            set
            {
                m_Header->MassFactors = value;
                ModifiersChanged = true;
            }
        }

        public bool HasSurfaceVelocity => m_Header->HasSurfaceVelocity;
        public SurfaceVelocity SurfaceVelocity
        {
            get => m_Header->SurfaceVelocity;
            set
            {
                m_Header->SurfaceVelocity = value;
                ModifiersChanged = true;
            }
        }

        public ContactJacAngAndVelToReachCp GetAngularJacobian(int i)
        {
            return m_Header->AccessAngularJacobian(i);
        }

        public void SetAngularJacobian(int i, ContactJacAngAndVelToReachCp j)
        {
            m_Header->AccessAngularJacobian(i) = j;
            AngularChanged = true;
        }
    }

    public unsafe struct ModifiableContactJacobian
    {
        internal ContactJacobian* m_ContactJacobian;
        public bool Modified { get; private set; }

        public int NumContacts => m_ContactJacobian->BaseJacobian.NumContacts;

        public float3 Normal
        {
            get => m_ContactJacobian->BaseJacobian.Normal;
            set
            {
                m_ContactJacobian->BaseJacobian.Normal = value;
                Modified = true;
            }
        }

        public sfloat CoefficientOfFriction
        {
            get => m_ContactJacobian->CoefficientOfFriction;
            set
            {
                m_ContactJacobian->CoefficientOfFriction = value;
                Modified = true;
            }
        }

        public ContactJacobianAngular Friction0
        {
            get => m_ContactJacobian->Friction0;
            set
            {
                m_ContactJacobian->Friction0 = value;
                Modified = true;
            }
        }

        public ContactJacobianAngular Friction1
        {
            get => m_ContactJacobian->Friction1;
            set
            {
                m_ContactJacobian->Friction1 = value;
                Modified = true;
            }
        }

        public ContactJacobianAngular AngularFriction
        {
            get => m_ContactJacobian->AngularFriction;
            set
            {
                m_ContactJacobian->AngularFriction = value;
                Modified = true;
            }
        }
    }

    public struct ModifiableTriggerJacobian
    {
        internal unsafe TriggerJacobian* m_TriggerJacobian;
    }

    public static class IJacobiansJobExtensions
    {
#if !HAVOK_PHYSICS_EXISTS
        // Default Schedule() implementation for IJacobiansJob.
        public static unsafe JobHandle Schedule<T>(this T jobData, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps)
            where T : struct, IJacobiansJobBase
        {
            // Should work only for UnityPhysics
            if (simulation.Type != SimulationType.UnityPhysics)
            {
                return inputDeps;
            }

            return ScheduleUnityPhysicsJacobiansJob(jobData, simulation, ref world, inputDeps);
        }

#else
        // In this case Schedule() implementation for IJacobiansJob is provided by the Havok.Physics assembly.
        // This is a stub to catch when that assembly is missing.
        //<todo.eoin.modifier Put in a link to documentation for this:
        [Obsolete("This error occurs when HAVOK_PHYSICS_EXISTS is defined but Havok.Physics is missing from your package's asmdef references. (DoNotRemove)", true)]
        public static unsafe JobHandle Schedule<T>(this T jobData, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps,
            HAVOK_PHYSICS_MISSING_FROM_ASMDEF _causeCompileError = HAVOK_PHYSICS_MISSING_FROM_ASMDEF.HAVOK_PHYSICS_MISSING_FROM_ASMDEF)
            where T : struct, IJacobiansJobBase
        {
            return new JobHandle();
        }

        public enum HAVOK_PHYSICS_MISSING_FROM_ASMDEF
        {
            HAVOK_PHYSICS_MISSING_FROM_ASMDEF
        }
#endif

        internal static unsafe JobHandle ScheduleUnityPhysicsJacobiansJob<T>(T jobData, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps)
            where T : struct, IJacobiansJobBase
        {
            SafetyChecks.CheckAreEqualAndThrow(SimulationType.UnityPhysics, simulation.Type);

            var data = new JacobiansJobData<T>
            {
                UserJobData = jobData,
                StreamReader = ((Simulation)simulation).StepContext.Jacobians.AsReader(),
                NumWorkItems = ((Simulation)simulation).StepContext.SolverSchedulerInfo.NumWorkItems,
                Bodies = world.Bodies
            };
            var parameters = new JobsUtility.JobScheduleParameters(
#if UNITY_2020_2_OR_NEWER
            UnsafeUtility.AddressOf(ref data), JacobiansJobProcess<T>.Initialize(), inputDeps, ScheduleMode.Single);
#else
                    UnsafeUtility.AddressOf(ref data), JacobiansJobProcess<T>.Initialize(), inputDeps, ScheduleMode.Batched);
#endif
            return JobsUtility.Schedule(ref parameters);
        }

        internal unsafe struct JacobiansJobData<T> where T : struct
        {
            public T UserJobData;
            public NativeStream.Reader StreamReader;

            [ReadOnly] public NativeArray<int> NumWorkItems;

            // Disable aliasing restriction in case T has a NativeArray of PhysicsWorld.Bodies
            [ReadOnly, NativeDisableContainerSafetyRestriction] public NativeArray<RigidBody> Bodies;

            int m_CurrentWorkItem;

            public bool HasItemsLeft => StreamReader.RemainingItemCount > 0;

            public JacobianHeader* ReadJacobianHeader()
            {
                int readSize = Read<int>();
                return (JacobianHeader*)Read(readSize);
            }

            private byte* Read(int size)
            {
                byte* dataPtr = StreamReader.ReadUnsafePtr(size);
                MoveReaderToNextForEachIndex();
                return dataPtr;
            }

            private ref T2 Read<T2>() where T2 : struct
            {
                int size = UnsafeUtility.SizeOf<T2>();
                return ref UnsafeUtility.AsRef<T2>(Read(size));
            }

            public void MoveReaderToNextForEachIndex()
            {
                int numWorkItems = NumWorkItems[0];
                while (StreamReader.RemainingItemCount == 0 && m_CurrentWorkItem < numWorkItems)
                {
                    StreamReader.BeginForEachIndex(m_CurrentWorkItem);
                    m_CurrentWorkItem++;
                }
            }
        }

        internal struct JacobiansJobProcess<T> where T : struct, IJacobiansJobBase
        {
            static IntPtr jobReflectionData;

            public static IntPtr Initialize()
            {
                if (jobReflectionData == IntPtr.Zero)
                {
#if UNITY_2020_2_OR_NEWER
                    jobReflectionData = JobsUtility.CreateJobReflectionData(typeof(JacobiansJobData<T>), typeof(T), (ExecuteJobFunction)Execute);
#else
                    jobReflectionData = JobsUtility.CreateJobReflectionData(typeof(JacobiansJobData<T>), typeof(T), JobType.Single, (ExecuteJobFunction)Execute);
#endif
                }
                return jobReflectionData;
            }

            public delegate void ExecuteJobFunction(ref JacobiansJobData<T> jobData, IntPtr additionalData,
                IntPtr bufferRangePatchData, ref JobRanges ranges, int jobIndex);

            public unsafe static void Execute(ref JacobiansJobData<T> jobData, IntPtr additionalData,
                IntPtr bufferRangePatchData, ref JobRanges ranges, int jobIndex)
            {
                jobData.MoveReaderToNextForEachIndex();
                while (jobData.HasItemsLeft)
                {
                    JacobianHeader* header = jobData.ReadJacobianHeader();

                    var h = new ModifiableJacobianHeader
                    {
                        m_Header = header,
                        EntityPair = new EntityPair
                        {
                            EntityA = jobData.Bodies[header->BodyPair.BodyIndexA].Entity,
                            EntityB = jobData.Bodies[header->BodyPair.BodyIndexB].Entity
                        }
                    };
                    if (header->Type == JacobianType.Contact)
                    {
                        var contact = new ModifiableContactJacobian
                        {
                            m_ContactJacobian = (ContactJacobian*)UnsafeUtility.AddressOf(ref header->AccessBaseJacobian<ContactJacobian>())
                        };
                        jobData.UserJobData.Execute(ref h, ref contact);
                    }
                    else if (header->Type == JacobianType.Trigger)
                    {
                        var trigger = new ModifiableTriggerJacobian
                        {
                            m_TriggerJacobian = (TriggerJacobian*)UnsafeUtility.AddressOf(ref header->AccessBaseJacobian<TriggerJacobian>())
                        };

                        jobData.UserJobData.Execute(ref h, ref trigger);
                    }
                }
            }
        }
    }
}
