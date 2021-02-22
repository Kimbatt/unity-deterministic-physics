using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;
using UnityS.Mathematics;

namespace UnityS.Physics
{
    // INTERNAL UnityPhysics interface for jobs that iterate through the list of contact manifolds produced by the narrow phase
    // Important: Only use inside UnityPhysics code! Jobs in other projects should implement IContactsJob.
    [JobProducerType(typeof(IContactsJobExtensions.ContactsJobProcess<>))]
    public interface IContactsJobBase
    {
        // Note, multiple contacts can share the same header, but will have a different ModifiableContactPoint.Index.
        void Execute(ref ModifiableContactHeader header, ref ModifiableContactPoint contact);
    }

#if !HAVOK_PHYSICS_EXISTS

    // Interface for jobs that iterate through the list of contact manifolds produced by the narrow phase
    public interface IContactsJob : IContactsJobBase
    {
    }

#endif

    public struct ModifiableContactHeader
    {
        internal ContactHeader ContactHeader;
        public bool Modified { get; private set; }

        internal EntityPair EntityPair;

        public Entity EntityB => EntityPair.EntityB;
        public Entity EntityA => EntityPair.EntityA;
        public int BodyIndexB => ContactHeader.BodyPair.BodyIndexB;
        public int BodyIndexA => ContactHeader.BodyPair.BodyIndexA;
        public byte CustomTagsB => ContactHeader.BodyCustomTags.CustomTagsB;
        public byte CustomTagsA => ContactHeader.BodyCustomTags.CustomTagsA;
        public ColliderKey ColliderKeyB => ContactHeader.ColliderKeys.ColliderKeyB;
        public ColliderKey ColliderKeyA => ContactHeader.ColliderKeys.ColliderKeyA;

        public int NumContacts => ContactHeader.NumContacts;

        public JacobianFlags JacobianFlags
        {
            get => ContactHeader.JacobianFlags;
            set
            {
                ContactHeader.JacobianFlags = value;
                Modified = true;
            }
        }

        public float3 Normal
        {
            get => ContactHeader.Normal;
            set
            {
                ContactHeader.Normal = value;
                Modified = true;
            }
        }

        public sfloat CoefficientOfFriction
        {
            get => ContactHeader.CoefficientOfFriction;
            set
            {
                ContactHeader.CoefficientOfFriction = value;
                Modified = true;
            }
        }

        public sfloat CoefficientOfRestitution
        {
            get => ContactHeader.CoefficientOfRestitution;
            set
            {
                ContactHeader.CoefficientOfRestitution = value;
                Modified = true;
            }
        }
    }

    public struct ModifiableContactPoint
    {
        internal ContactPoint ContactPoint;
        public bool Modified { get; private set; }

        /// Index of this point, within the ModifiableContactHeader
        public int Index { get; internal set; }

        public float3 Position
        {
            get => ContactPoint.Position;
            set
            {
                ContactPoint.Position = value;
                Modified = true;
            }
        }

        public sfloat Distance
        {
            get => ContactPoint.Distance;
            set
            {
                ContactPoint.Distance = value;
                Modified = true;
            }
        }
    }

    public static class IContactsJobExtensions
    {
#if !HAVOK_PHYSICS_EXISTS
        // Default Schedule() implementation for IContactsJob.
        public static unsafe JobHandle Schedule<T>(this T jobData, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps)
            where T : struct, IContactsJobBase
        {
            // Should work only for UnityPhysics
            if (simulation.Type != SimulationType.UnityPhysics)
            {
                return inputDeps;
            }

            return ScheduleUnityPhysicsContactsJob(jobData, simulation, ref world, inputDeps);
        }

#else
        // In this case Schedule() implementation for IContactsJob is provided by the Havok.Physics assembly.
        // This is a stub to catch when that assembly is missing.
        //<todo.eoin.modifier Put in a link to documentation for this:
        [Obsolete("This error occurs when HAVOK_PHYSICS_EXISTS is defined but Havok.Physics is missing from your package's asmdef references. (DoNotRemove)", true)]
        public static unsafe JobHandle Schedule<T>(this T jobData, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps,
            HAVOK_PHYSICS_MISSING_FROM_ASMDEF _causeCompileError = HAVOK_PHYSICS_MISSING_FROM_ASMDEF.HAVOK_PHYSICS_MISSING_FROM_ASMDEF)
            where T : struct, IContactsJobBase
        {
            return new JobHandle();
        }

        public enum HAVOK_PHYSICS_MISSING_FROM_ASMDEF
        {
            HAVOK_PHYSICS_MISSING_FROM_ASMDEF
        }
#endif

        internal static unsafe JobHandle ScheduleUnityPhysicsContactsJob<T>(this T jobData, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps)
            where T : struct, IContactsJobBase
        {
            SafetyChecks.CheckAreEqualAndThrow(SimulationType.UnityPhysics, simulation.Type);

            var data = new ContactsJobData<T>
            {
                UserJobData = jobData,
                ContactReader = ((Simulation)simulation).StepContext.Contacts.AsReader(),
                NumWorkItems = ((Simulation)simulation).StepContext.SolverSchedulerInfo.NumWorkItems,
                Bodies = world.Bodies
            };
            var parameters = new JobsUtility.JobScheduleParameters(
#if UNITY_2020_2_OR_NEWER
                UnsafeUtility.AddressOf(ref data), ContactsJobProcess<T>.Initialize(), inputDeps, ScheduleMode.Single);
#else
                UnsafeUtility.AddressOf(ref data), ContactsJobProcess<T>.Initialize(), inputDeps, ScheduleMode.Batched);
#endif
            return JobsUtility.Schedule(ref parameters);
        }

        internal unsafe struct ContactsJobData<T> where T : struct
        {
            public T UserJobData;

            [NativeDisableContainerSafetyRestriction] public NativeStream.Reader ContactReader;
            [ReadOnly] public NativeArray<int> NumWorkItems;
            // Disable aliasing restriction in case T has a NativeArray of PhysicsWorld.Bodies
            [ReadOnly, NativeDisableContainerSafetyRestriction] public NativeArray<RigidBody> Bodies;
        }

        internal struct ContactsJobProcess<T> where T : struct, IContactsJobBase
        {
            static IntPtr jobReflectionData;

            public static IntPtr Initialize()
            {
                if (jobReflectionData == IntPtr.Zero)
                {
#if UNITY_2020_2_OR_NEWER
                    jobReflectionData = JobsUtility.CreateJobReflectionData(typeof(ContactsJobData<T>), typeof(T), (ExecuteJobFunction)Execute);
#else
                    jobReflectionData = JobsUtility.CreateJobReflectionData(typeof(ContactsJobData<T>), typeof(T), JobType.Single, (ExecuteJobFunction)Execute);
#endif
                }
                return jobReflectionData;
            }

            public delegate void ExecuteJobFunction(ref ContactsJobData<T> jobData, IntPtr additionalData,
                IntPtr bufferRangePatchData, ref JobRanges ranges, int jobIndex);

            public unsafe static void Execute(ref ContactsJobData<T> jobData, IntPtr additionalData,
                IntPtr bufferRangePatchData, ref JobRanges ranges, int jobIndex)
            {
                var iterator = new ContactsJobIterator(jobData.ContactReader, jobData.NumWorkItems[0]);

                while (iterator.HasItemsLeft())
                {
                    iterator.Next();

                    //<todo.eoin.modifier Could store the pointer, to avoid copies, like the jacobian job?
                    var header = new ModifiableContactHeader
                    {
                        ContactHeader = *iterator.m_LastHeader,
                        EntityPair = new EntityPair
                        {
                            EntityA = jobData.Bodies[iterator.m_LastHeader->BodyPair.BodyIndexA].Entity,
                            EntityB = jobData.Bodies[iterator.m_LastHeader->BodyPair.BodyIndexB].Entity
                        }
                    };
                    var contact = new ModifiableContactPoint
                    {
                        ContactPoint = *iterator.m_LastContact,
                        Index = iterator.CurrentPointIndex
                    };

                    jobData.UserJobData.Execute(ref header, ref contact);

                    if (header.Modified)
                    {
                        *iterator.m_LastHeader = header.ContactHeader;
                    }

                    if (contact.Modified)
                    {
                        *iterator.m_LastContact = contact.ContactPoint;
                    }
                }
            }
        }

        // Utility to help iterate over all the items in the contacts job stream
        private unsafe struct ContactsJobIterator
        {
            [NativeDisableContainerSafetyRestriction] NativeStream.Reader m_ContactReader;
            [NativeDisableUnsafePtrRestriction] public ContactHeader* m_LastHeader;
            [NativeDisableUnsafePtrRestriction] public ContactPoint* m_LastContact;
            int m_NumPointsLeft;
            int m_CurrentWorkItem;
            readonly int m_MaxNumWorkItems;

            public unsafe ContactsJobIterator(NativeStream.Reader reader, int numWorkItems)
            {
                m_ContactReader = reader;
                m_MaxNumWorkItems = numWorkItems;

                m_CurrentWorkItem = 0;
                m_NumPointsLeft = 0;
                m_LastHeader = null;
                m_LastContact = null;
                AdvanceForEachIndex();
            }

            public bool HasItemsLeft()
            {
                return m_ContactReader.RemainingItemCount > 0;
            }

            public unsafe int CurrentPointIndex => m_LastHeader->NumContacts - m_NumPointsLeft - 1;

            public void Next()
            {
                if (HasItemsLeft())
                {
                    if (m_NumPointsLeft == 0)
                    {
                        // Need to get a new header
                        m_LastHeader = (ContactHeader*)m_ContactReader.ReadUnsafePtr(sizeof(ContactHeader));
                        m_NumPointsLeft = m_LastHeader->NumContacts;
                        AdvanceForEachIndex();
                    }

                    m_LastContact = (ContactPoint*)m_ContactReader.ReadUnsafePtr(sizeof(ContactPoint));
                    m_NumPointsLeft--;
                    AdvanceForEachIndex();
                }
            }

            void AdvanceForEachIndex()
            {
                while (m_ContactReader.RemainingItemCount == 0 && m_CurrentWorkItem < m_MaxNumWorkItems)
                {
                    m_ContactReader.BeginForEachIndex(m_CurrentWorkItem);
                    m_CurrentWorkItem++;
                }
            }
        }
    }
}
