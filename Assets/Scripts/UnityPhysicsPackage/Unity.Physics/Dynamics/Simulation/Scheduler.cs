using System;
using System.Diagnostics;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using UnityS.Mathematics;
using UnityEngine.Assertions;

namespace UnityS.Physics
{
    // Builds phased pairs of interacting bodies, used to parallelize work items during the simulation step.
    public class DispatchPairSequencer : IDisposable
    {
        internal readonly BitLookupTableDynamicDynamicPairs m_PhaseLookupTableDynamicDynamicPairs;
        internal readonly BitLookupTableDynamicStaticPairs m_PhaseLookupTableDynamicStaticPairs;

        // A pair of interacting bodies (either potentially colliding, or constrained together using a Joint).
        // The indices are compressed into a single 64 bit value, for deterministic sorting, as follows:
        //
        //        6         5         4         3         2         1         
        //    4321098765432109876543210987654321098765432109876543210987654321
        // 0b_1111111111111111111111111111111111111111111111111111111111111111
        //    [        BodyA-24      ][       BodyB-24       ]C[  Joint-15   ]
        //
        //
        // This gives a limit of 
        //    16,777,216 Rigid bodies
        //    32,767 Joints (1 bit used for Enable [C]ollisions flag)
        //
        // We additionally choose indices so that BodyIndexA < BodyIndexB. This has subtle side-effects:
        // * If one body in the pair is static, it will be body B.
        // * Indices used for jointed pairs are not necessarily the same as selected in the joint
        // * For some body A, all it's static collisions will be contiguous.
        //
        [DebuggerDisplay("{IsContact ? \"Contact\" : \"Joint\"}, [{BodyIndexA}, {BodyIndexB}]")]
        public struct DispatchPair
        {
            private ulong m_Data;

            internal static readonly uint k_InvalidBodyIndex = 0b_0000_0000_1111_1111_1111_1111_1111_1111; // 24 bits
            internal static readonly uint k_InvalidJointIndex = 0b_0000_0000_0000_0000_0111_1111_1111_1111; // 15 bits

            internal static readonly int k_JointIndexShift = 0;
            internal static readonly int k_BodyIndexBShift = k_JointIndexShift + math.countbits(k_InvalidJointIndex) + 1;//EnableCollisions
            internal static readonly int k_BodyIndexAShift = k_BodyIndexBShift + math.countbits(k_InvalidBodyIndex);

            internal static readonly ulong k_BodyAMask = ~((ulong)(k_InvalidBodyIndex) << k_BodyIndexAShift);
            internal static readonly ulong k_BodyBMask = ~((ulong)(k_InvalidBodyIndex) << k_BodyIndexBShift);
            internal static readonly ulong k_JointMask = ~((ulong)(k_InvalidJointIndex) << k_JointIndexShift);

            private static readonly ulong k_EnableJointCollisionBit = ((ulong)k_InvalidJointIndex + 1) << k_JointIndexShift;

            public bool IsValid => m_Data != DispatchPair.Invalid.m_Data;
            public bool IsJoint => JointIndex != k_InvalidJointIndex;
            public bool IsContact => !(IsJoint);

            public static DispatchPair Invalid => new DispatchPair { m_Data = ~(ulong)0x0 };

            public int BodyIndexA
            {
                get => (int)((m_Data >> k_BodyIndexAShift) & k_InvalidBodyIndex);
                internal set
                {
                    Assert.IsTrue(value < k_InvalidBodyIndex);
                    m_Data = (m_Data & k_BodyAMask) | ((ulong)value << k_BodyIndexAShift);
                }
            }

            public int BodyIndexB
            {
                get => (int)((m_Data >> k_BodyIndexBShift) & k_InvalidBodyIndex);
                internal set
                {
                    Assert.IsTrue(value < k_InvalidBodyIndex);
                    m_Data = (m_Data & k_BodyBMask) | ((ulong)value << k_BodyIndexBShift);
                }
            }

            public int JointIndex
            {
                get => (int)((m_Data >> k_JointIndexShift) & k_InvalidJointIndex);
                internal set
                {
                    Assert.IsTrue(value < k_InvalidJointIndex);
                    m_Data = (m_Data & k_JointMask) | (uint)(value << k_JointIndexShift);
                }
            }

            internal bool JointAllowsCollision
            {
                get => (m_Data & k_EnableJointCollisionBit) != 0;
            }

            public static DispatchPair CreateCollisionPair(BodyIndexPair pair)
            {
                // Note: The EnabledCollisions flag is set (1) deliberately.
                // Setting all bits related to the Joint ensures that joints are
                // always solved before the higher priority contacts.
                return Create(pair, (int)k_InvalidJointIndex, 1);
            }

            public static DispatchPair CreateJoint(BodyIndexPair pair, int jointIndex, int allowCollision)
            {
                Assert.IsTrue(jointIndex < k_InvalidJointIndex);
                return Create(pair, jointIndex, allowCollision);
            }

            private static DispatchPair Create(BodyIndexPair pair, int jointIndex, int allowCollision)
            {
                Assert.IsTrue(pair.BodyIndexA < k_InvalidBodyIndex && pair.BodyIndexB < k_InvalidBodyIndex);
                int selectedA = math.min(pair.BodyIndexA, pair.BodyIndexB);
                int selectedB = math.max(pair.BodyIndexA, pair.BodyIndexB);
                return new DispatchPair
                {
                    m_Data =
                        ((ulong)selectedA << k_BodyIndexAShift) |
                        ((ulong)selectedB << k_BodyIndexBShift) |
                        (allowCollision == 0 ? 0 : k_EnableJointCollisionBit) |
                        ((ulong)jointIndex << k_JointIndexShift)
                };
            }
        }

        // A phased set of dispatch pairs used to schedule solver jobs and distribute work between them.
        [NoAlias]
        internal struct SolverSchedulerInfo : IDisposable
        {
            // A structure which describes the number of items in a single phase
            internal struct SolvePhaseInfo
            {
                internal int DispatchPairCount; // The total number of pairs in this phase
                internal int BatchSize; // The number of items per thread work item; at most, the number of pairs
                internal int NumWorkItems; // The amount of "subtasks" of size BatchSize in this phase
                internal int FirstWorkItemIndex; // The sum of NumWorkItems in all previous phases. Used for output native stream.
                internal int FirstDispatchPairIndex; // Index into the array of DispatchPairs for this phase.
            }

            [NoAlias]
            internal NativeArray<SolvePhaseInfo> PhaseInfo;
            [NoAlias]
            internal NativeArray<int> NumActivePhases;
            [NoAlias]
            internal NativeArray<int> NumWorkItems;

            internal int NumPhases => PhaseInfo.Length;
            internal static int CalculateNumWorkItems(NativeArray<SolvePhaseInfo> phaseInfos)
            {
                int numWorkItems = 0;
                for (int i = 0; i < phaseInfos.Length; i++)
                    numWorkItems += phaseInfos[i].NumWorkItems;
                return numWorkItems;
            }

            // For a given work item returns phase id.
            internal int FindPhaseId(int workItemIndex)
            {
                int phaseId = 0;
                for (int i = NumActivePhases[0] - 1; i >= 0; i--)
                {
                    if (workItemIndex >= PhaseInfo[i].FirstWorkItemIndex)
                    {
                        phaseId = i;
                        break;
                    }
                }

                return phaseId;
            }

            // For a given work item returns index into PhasedDispatchPairs and number of pairs to read.
            internal int GetWorkItemReadOffset(int workItemIndex, out int pairReadCount)
            {
                SolvePhaseInfo phaseInfo = PhaseInfo[FindPhaseId(workItemIndex)];

                int numItemsToRead = phaseInfo.BatchSize;
                int readStartOffset = phaseInfo.FirstDispatchPairIndex + (workItemIndex - phaseInfo.FirstWorkItemIndex) * phaseInfo.BatchSize;

                int lastWorkItemIndex = phaseInfo.FirstWorkItemIndex + phaseInfo.NumWorkItems - 1;
                bool isLastWorkItemInPhase = workItemIndex == lastWorkItemIndex;
                if (isLastWorkItemInPhase)
                {
                    int numPairsBeforeLastWorkItem = (phaseInfo.NumWorkItems - 1) * phaseInfo.BatchSize;
                    numItemsToRead = phaseInfo.DispatchPairCount - numPairsBeforeLastWorkItem;
                    readStartOffset = phaseInfo.FirstDispatchPairIndex + numPairsBeforeLastWorkItem;
                }

                pairReadCount = numItemsToRead;

                return readStartOffset;
            }

            public SolverSchedulerInfo(int numPhases)
            {
                PhaseInfo = new NativeArray<SolvePhaseInfo>(numPhases, Allocator.TempJob);
                NumActivePhases = new NativeArray<int>(1, Allocator.TempJob);
                NumWorkItems = new NativeArray<int>(1, Allocator.TempJob);
            }

            public void Dispose()
            {
                if (PhaseInfo.IsCreated)
                {
                    PhaseInfo.Dispose();
                }

                if (NumActivePhases.IsCreated)
                {
                    NumActivePhases.Dispose();
                }

                if (NumWorkItems.IsCreated)
                {
                    NumWorkItems.Dispose();
                }
            }

            public JobHandle ScheduleDisposeJob(JobHandle inputDeps)
            {
                return new DisposeJob { PhaseInfo = PhaseInfo, NumActivePhases = NumActivePhases, NumWorkItems = NumWorkItems }.Schedule(inputDeps);
            }

            // A job to dispose the phase information
            [BurstCompile]
            private struct DisposeJob : IJob
            {
                [DeallocateOnJobCompletion]
                public NativeArray<SolvePhaseInfo> PhaseInfo;

                [DeallocateOnJobCompletion]
                public NativeArray<int> NumActivePhases;

                [DeallocateOnJobCompletion]
                public NativeArray<int> NumWorkItems;

                public void Execute() { }
            }
        }

        public DispatchPairSequencer()
        {
            m_PhaseLookupTableDynamicDynamicPairs = new BitLookupTableDynamicDynamicPairs(numPhases: 16);
            m_PhaseLookupTableDynamicStaticPairs = new BitLookupTableDynamicStaticPairs(numPhases: 16);
        }

        public void Dispose()
        {
            m_PhaseLookupTableDynamicDynamicPairs.Dispose();
            m_PhaseLookupTableDynamicStaticPairs.Dispose();
        }

        // Merge streams of body pairs and joints into an sorted array of dispatch pairs
        public static void CreateDispatchPairs(
            ref NativeStream dynamicVsDynamicBodyPairs, ref NativeStream staticVsDynamicBodyPairs,
            int numDynamicBodies, NativeArray<Joint> joints, ref NativeList<DispatchPair> dispatchPairs)
        {
            // Create dispatch pairs
            var tempPairs = new NativeList<DispatchPair>(Allocator.Temp);
            CreateUnsortedDispatchPairsJob.ExecuteImpl(dynamicVsDynamicBodyPairs, staticVsDynamicBodyPairs,
                joints, tempPairs, dispatchPairs);

            // Early out if no dispatch pairs
            if (tempPairs.Length > 0)
            {
                // Sort the pairs
                unsafe
                {
                    NativeSortExtension.Sort((ulong*)tempPairs.GetUnsafePtr(), tempPairs.Length);
                }

                const int numPhases = 16;
                NativeArray<SolverSchedulerInfo.SolvePhaseInfo> phaseInfo = new NativeArray<SolverSchedulerInfo.SolvePhaseInfo>(numPhases, Allocator.Temp);
                CreateDispatchPairPhasesJob.CreateDispatchPairPhasesJobFunction(
                    default, default, tempPairs, numDynamicBodies, numPhases,
                    dispatchPairs, out int numActivePhases, out int numWorkItems, ref phaseInfo, false);

                int totalDispatchPairs = 0;
                for (int i = 0; i < numActivePhases; i++)
                {
                    totalDispatchPairs += phaseInfo[i].DispatchPairCount;
                }

                // Invalidate the leftover pairs
                for (int i = totalDispatchPairs; i < dispatchPairs.Length; i++)
                {
                    dispatchPairs[i] = DispatchPair.Invalid;
                }
            }
        }

        // Schedule a set of jobs to merge streams of body pairs and joints into an sorted array of dispatch pairs
        internal unsafe SimulationJobHandles ScheduleCreatePhasedDispatchPairsJob(
            ref PhysicsWorld world, ref NativeStream dynamicVsDynamicBroadphasePairsStream, ref NativeStream staticVsDynamicBroadphasePairStream,
            JobHandle inputDeps, ref NativeList<DispatchPair> dispatchPairs, out SolverSchedulerInfo solverSchedulerInfo,
            bool multiThreaded = true)
        {
            SimulationJobHandles returnHandles = default;

            if (!multiThreaded)
            {
                dispatchPairs = new NativeList<DispatchPair>(Allocator.TempJob);

                solverSchedulerInfo = new SolverSchedulerInfo(1);
                solverSchedulerInfo.NumActivePhases[0] = 1;
                solverSchedulerInfo.NumWorkItems[0] = 1;

                returnHandles.FinalExecutionHandle = new CreateDispatchPairsJobST
                {
                    DispatchPairs = dispatchPairs,
                    DynamicVsDynamicBroadphasePairsStream = dynamicVsDynamicBroadphasePairsStream,
                    StaticVsDynamicBroadphasePairsStream = staticVsDynamicBroadphasePairStream,
                    Joints = world.Joints,
                    NumDynamicBodies = world.NumDynamicBodies
                }.Schedule(inputDeps);

                return returnHandles;
            }

            // First build a sorted array of dispatch pairs.
            var unsortedPairs = new NativeList<DispatchPair>(Allocator.TempJob);
            var sortedPairs = new NativeList<DispatchPair>(Allocator.TempJob);

            JobHandle sortHandle;
            {
                // Merge broadphase pairs and joint pairs into the unsorted array
                JobHandle dispatchHandle = new CreateUnsortedDispatchPairsJob
                {
                    DynamicVsDynamicPairs = dynamicVsDynamicBroadphasePairsStream,
                    StaticVsDynamicPairs = staticVsDynamicBroadphasePairStream,
                    Joints = world.Joints,
                    UnsortedDispatchPairs = unsortedPairs,
                    DispatchPairsUninitialized = sortedPairs
                }.Schedule(inputDeps);

                // Dispose our broad phase pairs
                var disposeBroadphasePairs0 = dynamicVsDynamicBroadphasePairsStream.Dispose(dispatchHandle);
                var disposeBroadphasePairs1 = staticVsDynamicBroadphasePairStream.Dispose(dispatchHandle);
                returnHandles.FinalDisposeHandle = JobHandle.CombineDependencies(disposeBroadphasePairs0, disposeBroadphasePairs1);

                // Sort into the target array
                sortHandle = ScheduleSortJob(world.NumBodies,
                    unsortedPairs.AsDeferredJobArray(), sortedPairs.AsDeferredJobArray(), dispatchHandle);
            }

            // Create phases for multi-threading
            solverSchedulerInfo = new SolverSchedulerInfo(m_PhaseLookupTableDynamicDynamicPairs.NumPhases);
            dispatchPairs = unsortedPairs;
            JobHandle dispatchPairHandle = new CreateDispatchPairPhasesJob
            {
                DispatchPairs = sortedPairs.AsDeferredJobArray(),
                SolverSchedulerInfo = solverSchedulerInfo,
                NumDynamicBodies = world.NumDynamicBodies,
                PhaseLookupTableDynamicDynamicPairs = m_PhaseLookupTableDynamicDynamicPairs.Table,
                PhaseLookupTableDynamicStaticPairs = m_PhaseLookupTableDynamicStaticPairs.Table,
                NumPhases = m_PhaseLookupTableDynamicDynamicPairs.NumPhases,
                PhasedDispatchPairs = unsortedPairs.AsDeferredJobArray()
            }.Schedule(sortHandle);

            // Dispose
            var disposePhasedDispatchPairs = sortedPairs.Dispose(dispatchPairHandle);
            returnHandles.FinalDisposeHandle = JobHandle.CombineDependencies(returnHandles.FinalDisposeHandle, disposePhasedDispatchPairs);

            returnHandles.FinalExecutionHandle = dispatchPairHandle;

            return returnHandles;
        }

        #region Helpers

        // A lookup table used by CreateDispatchPairPhasesJob
        internal struct BitLookupTableDynamicDynamicPairs : IDisposable
        {
            public readonly int NumPhases;
            public readonly NativeArray<ushort> Table;

            public BitLookupTableDynamicDynamicPairs(int numPhases, Allocator allocator = Allocator.Persistent)
            {
                NumPhases = numPhases;

                Table = new NativeArray<ushort>(ushort.MaxValue + 1, allocator, NativeArrayOptions.UninitializedMemory);
                const ushort end = ushort.MaxValue;
                ushort numBits = (ushort)numPhases;

                // For each 'value' find index of first zero bit.
                for (ushort value = 0; value < end; value++)
                {
                    ushort valueCopy = value;
                    for (ushort i = 0; i < numBits; i++)
                    {
                        if ((valueCopy & 1) == 0)
                        {
                            Table[value] = i;

                            break;
                        }

                        valueCopy >>= 1;
                    }
                }
                Table[end] = (ushort)(numBits - 1);
            }

            public void Dispose()
            {
                if (Table.IsCreated)
                {
                    Table.Dispose();
                }
            }
        }

        // A lookup table used by CreateDispatchPairPhasesJob
        internal struct BitLookupTableDynamicStaticPairs : IDisposable
        {
            public readonly int NumPhases;
            public readonly NativeArray<ushort> Table;

            public BitLookupTableDynamicStaticPairs(int numPhases, Allocator allocator = Allocator.Persistent)
            {
                NumPhases = numPhases;

                Table = new NativeArray<ushort>(ushort.MaxValue + 1, allocator, NativeArrayOptions.UninitializedMemory);
                const ushort end = ushort.MaxValue;
                ushort maxPhaseIndex = (ushort)(numPhases - 1);

                // For each 'value' find the first zero bit after last one bit.
                for (ushort value = 0; value < end; value++)
                {
                    int phaseIndex = 0;
                    for (int i = maxPhaseIndex; i >= 0; i--)
                    {
                        if ((value & (1 << i)) != 0)
                        {
                            phaseIndex = i + 1;
                            break;
                        }
                    }

                    phaseIndex = math.min(maxPhaseIndex, phaseIndex);

                    Table[value] = (ushort)phaseIndex;
                }

                Table[end] = maxPhaseIndex;
            }

            public void Dispose()
            {
                if (Table.IsCreated)
                {
                    Table.Dispose();
                }
            }
        }

        // Helper function to schedule jobs to sort an array of dispatch pairs.
        // The first single threaded job is a single pass Radix sort on the bits associated
        // with DispatchPair.BodyIndexA, resulting in sub arrays with the same bodyA index.
        // The second parallel job dispatches default sorts on each sub array.
        private static unsafe JobHandle ScheduleSortJob(
            int numBodies,
            NativeArray<DispatchPair> unsortedPairsIn,
            NativeArray<DispatchPair> sortedPairsOut,
            JobHandle handle)
        {
            var totalCountUpToDigit = new NativeArray<int>(numBodies + 1, Allocator.TempJob);

            // Calculate number digits needed to encode all body indices
            int numDigits = 0;
            int maxBodyIndex = numBodies - 1;
            {
                int val = maxBodyIndex;
                while (val > 0)
                {
                    val >>= 1;
                    numDigits++;
                }
            }

            // Perform single pass of single threaded radix sort.
            handle = new RadixSortPerBodyAJob
            {
                InputArray = unsortedPairsIn,
                OutputArray = sortedPairsOut,
                MaxDigits = numDigits,
                MaxIndex = maxBodyIndex,
                DigitCount = totalCountUpToDigit
            }.Schedule(handle);

            // Sort sub arrays with default sort.
            int numPerBatch = math.max(1, maxBodyIndex / 32);

            handle = new SortSubArraysJob
            {
                InOutArray = sortedPairsOut,
                NextElementIndex = totalCountUpToDigit
            }.Schedule(totalCountUpToDigit.Length, numPerBatch, handle);

            return handle;
        }

        #endregion

        #region Jobs

        // Combines body pairs and joint pairs into an array of dispatch pairs
        [BurstCompile]
        private struct CreateUnsortedDispatchPairsJob : IJob
        {
            // Body pairs from broadphase overlap jobs
            [ReadOnly] public NativeStream DynamicVsDynamicPairs;
            [ReadOnly] public NativeStream StaticVsDynamicPairs;

            // Joints from dynamics world
            [ReadOnly] public NativeArray<Joint> Joints;

            // Outputs
            public NativeList<DispatchPair> UnsortedDispatchPairs;
            public NativeList<DispatchPair> DispatchPairsUninitialized;

            public void Execute()
            {
                ExecuteImpl(DynamicVsDynamicPairs, StaticVsDynamicPairs, Joints, UnsortedDispatchPairs, DispatchPairsUninitialized);
            }

            internal static void ExecuteImpl(
                NativeStream dynamicVsDynamicPairs, NativeStream staticVsDynamicPairs,
                NativeArray<Joint> joints,
                NativeList<DispatchPair> unsortedDispatchPairs, NativeList<DispatchPair> dispatchPairsUninitialized)
            {
                int numValidJoints = 0;
                for (int i = 0; i < joints.Length; i++)
                {
                    if (joints[i].BodyPair.IsValid)
                    {
                        numValidJoints++;
                    }
                }

                var dynamicVsDynamicPairReader = dynamicVsDynamicPairs.AsReader();
                var staticVsDynamicPairReader = staticVsDynamicPairs.AsReader();

                int numDispatchPairs =
                    staticVsDynamicPairReader.Count() +
                    dynamicVsDynamicPairReader.Count() +
                    numValidJoints;

                unsortedDispatchPairs.ResizeUninitialized(numDispatchPairs);
                dispatchPairsUninitialized.ResizeUninitialized(numDispatchPairs);

                NativeArray<DispatchPair> pairs = unsortedDispatchPairs.AsArray();

                int counter = 0;
                for (int i = 0; i < dynamicVsDynamicPairReader.ForEachCount; i++)
                {
                    dynamicVsDynamicPairReader.BeginForEachIndex(i);
                    int rangeItemCount = dynamicVsDynamicPairReader.RemainingItemCount;
                    for (int j = 0; j < rangeItemCount; j++)
                    {
                        BodyIndexPair pair = dynamicVsDynamicPairReader.Read<BodyIndexPair>();
                        pairs[counter++] = DispatchPair.CreateCollisionPair(pair);
                    }
                    dynamicVsDynamicPairReader.EndForEachIndex();
                }

                for (int i = 0; i < staticVsDynamicPairReader.ForEachCount; i++)
                {
                    staticVsDynamicPairReader.BeginForEachIndex(i);
                    int rangeItemCount = staticVsDynamicPairReader.RemainingItemCount;
                    for (int j = 0; j < rangeItemCount; j++)
                    {
                        BodyIndexPair pair = staticVsDynamicPairReader.Read<BodyIndexPair>();
                        pairs[counter++] = DispatchPair.CreateCollisionPair(pair);
                    }
                    staticVsDynamicPairReader.EndForEachIndex();
                }

                for (int i = 0; i < joints.Length; i++)
                {
                    if (joints[i].BodyPair.IsValid)
                    {
                        pairs[counter++] = DispatchPair.CreateJoint(joints[i].BodyPair, i, joints[i].EnableCollision);
                    }
                }

                Assert.AreEqual(counter, pairs.Length);
            }
        }

        // Combines body pairs and joint pairs into an unsorted array of dispatch pairs
        [BurstCompile]
        private struct CreateDispatchPairsJobST : IJob
        {
            [ReadOnly] public NativeArray<Joint> Joints;
            [ReadOnly] public NativeStream DynamicVsDynamicBroadphasePairsStream;
            [ReadOnly] public NativeStream StaticVsDynamicBroadphasePairsStream;
            public NativeList<DispatchPair> DispatchPairs;
            public int NumDynamicBodies;

            public void Execute()
            {
                CreateDispatchPairs(
                    ref DynamicVsDynamicBroadphasePairsStream, ref StaticVsDynamicBroadphasePairsStream,
                    NumDynamicBodies, Joints, ref DispatchPairs);
            }
        }

        // Sorts an array of dispatch pairs by Body A index
        [BurstCompile]
        unsafe internal struct RadixSortPerBodyAJob : IJob
        {
            [ReadOnly]
            public NativeArray<DispatchPair> InputArray;
            [NativeDisableParallelForRestriction]
            public NativeArray<DispatchPair> OutputArray;
            [NativeDisableParallelForRestriction]
            public NativeArray<int> DigitCount;

            public int MaxDigits;
            public int MaxIndex;

            //@TODO: Add ReinterpretCast to NativeArray

            public void Execute()
            {
                Assert.AreEqual(InputArray.Length, OutputArray.Length);
                RadixSortPerBodyA(
                    (ulong*)InputArray.GetUnsafeReadOnlyPtr(),
                    (ulong*)OutputArray.GetUnsafePtr(),
                    InputArray.Length, DigitCount, MaxDigits, MaxIndex,
                    DispatchPair.k_BodyIndexAShift);
            }

            // Performs single pass of Radix sort on NativeArray<ulong> based on the bits
            // associated with BodyIndexA in DispatchPair.
            public static void RadixSortPerBodyA(ulong* inputArray, ulong* outputArray,
                int length, NativeArray<int> digitCount, int maxDigits, int maxIndex, int bitShift)
            {
                ulong mask = ((ulong)(1 << maxDigits) - 1) << bitShift;

                // Count digits
                for (int i = 0; i < length; i++)
                {
                    ulong usIndex = inputArray[i] & mask;
                    int sIndex = (int)(usIndex >> bitShift);
                    digitCount[sIndex]++;
                }

                // Calculate start index for each digit
                int prev = digitCount[0];
                digitCount[0] = 0;
                for (int i = 1; i <= maxIndex; i++)
                {
                    int current = digitCount[i];
                    digitCount[i] = digitCount[i - 1] + prev;
                    prev = current;
                }

                // Copy elements into buckets based on bodyA index
                for (int i = 0; i < length; i++)
                {
                    ulong value = inputArray[i];
                    ulong usindex = value & mask;
                    int sindex = (int)(usindex >> bitShift);
                    int index = digitCount[sindex]++;
                    if (index == 1 && length == 1)
                    {
                        outputArray[0] = 0;
                    }
                    outputArray[index] = value;
                }
            }
        }

        // Sorts slices of an array in parallel
        [BurstCompile]
        internal struct SortSubArraysJob : IJobParallelFor
        {
            [NativeDisableParallelForRestriction]
            public NativeArray<DispatchPair> InOutArray;

            // Typically lastDigitIndex is resulting in 
            // RadixSortPerBodyAJob.digitCount.nextElementIndex[i] = index of first element 
            // with bodyA index == i + 1
            [NativeDisableParallelForRestriction]
            [DeallocateOnJobCompletion] public NativeArray<int> NextElementIndex;

            unsafe public void Execute(int workItemIndex)
            {
                int startIndex = 0;
                if (workItemIndex > 0)
                {
                    startIndex = NextElementIndex[workItemIndex - 1];
                }

                if (startIndex < InOutArray.Length)
                {
                    int length = NextElementIndex[workItemIndex] - startIndex;
                    DefaultSortOfSubArrays((ulong*)InOutArray.GetUnsafePtr(), startIndex, length);
                }
            }

            // Sorts sub array using default sort
            unsafe public static void DefaultSortOfSubArrays(ulong* inOutArray, int startIndex, int length)
            {
                // inOutArray[startIndex] to inOutArray[startIndex + length - 1] have the same bodyA index 
                // so we can do a simple sorting.
                if (length > 2)
                {
                    NativeSortExtension.Sort(inOutArray + startIndex, length);
                }
                else if (length == 2)
                {
                    if (inOutArray[startIndex] > inOutArray[startIndex + 1])
                    {
                        ulong temp = inOutArray[startIndex + 1];
                        inOutArray[startIndex + 1] = inOutArray[startIndex];
                        inOutArray[startIndex] = temp;
                    }
                }
            }
        }

        // Creates phases based on sorted list of dispatch pairs
        [BurstCompile]
        internal struct CreateDispatchPairPhasesJob : IJob
        {
            [ReadOnly]
            public NativeArray<DispatchPair> DispatchPairs;

            [ReadOnly]
            public NativeArray<ushort> PhaseLookupTableDynamicDynamicPairs;

            [ReadOnly]
            public NativeArray<ushort> PhaseLookupTableDynamicStaticPairs;

            public SolverSchedulerInfo SolverSchedulerInfo;

            public NativeArray<DispatchPair> PhasedDispatchPairs;

            public int NumDynamicBodies;
            public int NumPhases;

            const int k_MinBatchSize = 8;

            public void Execute()
            {
                // Call function here, so that one can find it in VTune
                CreateDispatchPairPhasesJobFunction(
                    PhaseLookupTableDynamicDynamicPairs, PhaseLookupTableDynamicStaticPairs,
                    DispatchPairs, NumDynamicBodies, NumPhases, PhasedDispatchPairs,
                    out int numActivePhases, out int numWorkItems, ref SolverSchedulerInfo.PhaseInfo);
                SolverSchedulerInfo.NumActivePhases[0] = numActivePhases;
                SolverSchedulerInfo.NumWorkItems[0] = numWorkItems;
            }

            internal static unsafe void CreateDispatchPairPhasesJobFunction(
                NativeArray<ushort> phaseLookupTableDynamicDynamicPairs, NativeArray<ushort> phaseLookupTableDynamicStaticPairs,
                NativeArray<DispatchPair> dispatchPairs, int numDynamicBodies, int numPhases,
                NativeArray<DispatchPair> phasedDispatchPairs, out int numActivePhases, out int numWorkItems,
                ref NativeArray<SolverSchedulerInfo.SolvePhaseInfo> phaseInfo, bool useTables = true)
            {
                const byte k_unitializedPair = 254;
                var phaseIdPerPair = new NativeArray<byte>(dispatchPairs.Length, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
                for (int i = 0; i < dispatchPairs.Length; i++)
                {
                    phaseIdPerPair[i] = k_unitializedPair;
                }

                var rigidBodyMask = new NativeArray<ushort>(numDynamicBodies, Allocator.Temp);

                int lastPhaseIndex = numPhases - 1;
                int* numPairsPerPhase = stackalloc int[numPhases];
                numPairsPerPhase[lastPhaseIndex] = 0;

                const byte invalidPhaseId = 0xff;

                // Guaranteed not to be a real pair, as bodyA==bodyB==0
                int lastPairA = 0;
                int lastPairB = 0;

                bool contactsPermitted = true; // Will avoid creating a contact (=non-joint) pair if this is false

                BatchInfo* batchInfos = stackalloc BatchInfo[lastPhaseIndex];
                for (int i = 0; i < lastPhaseIndex; i++)
                {
                    batchInfos[i].m_NumElements = 0;
                    batchInfos[i].m_PhaseMask = (ushort)(1 << i);
                    batchInfos[i].m_NumBatchesProcessed = 0;
                }

                // Find phase for each dynamic-dynamic pair
                for (int i = 0; i < dispatchPairs.Length; i++)
                {
                    int bodyIndexA = dispatchPairs[i].BodyIndexA;
                    int bodyIndexB = dispatchPairs[i].BodyIndexB;

                    bool indicesChanged = !(lastPairA == bodyIndexA && lastPairB == bodyIndexB);
                    bool isJoint = dispatchPairs[i].IsJoint;

                    bool isBodyBStatic = bodyIndexB >= numDynamicBodies;

                    if (indicesChanged || contactsPermitted || isJoint)
                    {
                        // Skip dynamic-static pairs since we need to ensure that those are solved after all dynamic-dynamic pairs.
                        if (!isBodyBStatic)
                        {
                            int phaseIndex = FindFreePhaseDynamicDynamicPair(rigidBodyMask, bodyIndexA, bodyIndexB, lastPhaseIndex,
                                phaseLookupTableDynamicDynamicPairs, useTables);
                            phaseIdPerPair[i] = (byte)phaseIndex;

                            if (phaseIndex != lastPhaseIndex)
                            {
                                batchInfos[phaseIndex].Add(rigidBodyMask, bodyIndexA, bodyIndexB);
                            }
                            else
                            {
                                rigidBodyMask[bodyIndexA] |= (ushort)(1 << lastPhaseIndex);
                                numPairsPerPhase[lastPhaseIndex]++;
                            }
                        }

                        // If _any_ Joint between each Pair has Enable Collision set to true, then contacts will be permitted
                        bool thisPermitsContacts = isJoint && dispatchPairs[i].JointAllowsCollision;
                        contactsPermitted = (contactsPermitted && !indicesChanged) || thisPermitsContacts;

                        lastPairA = bodyIndexA;
                        lastPairB = bodyIndexB;
                    }
                    else
                    {
                        phaseIdPerPair[i] = invalidPhaseId;
                    }
                }

                // Commit rigid bodies masks for all batches that aren't flushed.
                for (int i = 0; i < lastPhaseIndex; i++)
                {
                    batchInfos[i].CommitRigidBodyMasks(rigidBodyMask);
                }

                // Find phase for each dynamic-static pair.
                for (int pairIndex = 0; pairIndex < dispatchPairs.Length; pairIndex++)
                {
                    int bodyIndexB = dispatchPairs[pairIndex].BodyIndexB;

                    bool isBodyBStatic = bodyIndexB >= numDynamicBodies;
                    if (isBodyBStatic && phaseIdPerPair[pairIndex] == k_unitializedPair)
                    {
                        int bodyIndexA = dispatchPairs[pairIndex].BodyIndexA;

                        int phaseIndex = FindFreePhaseDynamicStaticPair(rigidBodyMask, bodyIndexA, lastPhaseIndex,
                            phaseLookupTableDynamicStaticPairs, useTables);
                        phaseIdPerPair[pairIndex] = (byte)phaseIndex;

                        rigidBodyMask[bodyIndexA] |= (ushort)(1 << phaseIndex);
                        if (phaseIndex != lastPhaseIndex)
                        {
                            batchInfos[phaseIndex].m_NumElements++;
                        }
                        else
                        {
                            numPairsPerPhase[lastPhaseIndex]++;
                        }
                    }
                }

                for (int i = 0; i < lastPhaseIndex; i++)
                {
                    numPairsPerPhase[i] = batchInfos[i].m_NumBatchesProcessed * k_MinBatchSize + batchInfos[i].m_NumElements;
                }

                // Calculate phase start offset
                int* offsetInPhase = stackalloc int[numPhases];
                offsetInPhase[0] = 0;
                for (int i = 1; i < numPhases; i++)
                {
                    offsetInPhase[i] = offsetInPhase[i - 1] + numPairsPerPhase[i - 1];
                }

                // Populate PhasedDispatchPairsArray with dynamic-dynamic pairs
                for (int i = 0; i < dispatchPairs.Length; i++)
                {
                    int bodyIndexB = dispatchPairs[i].BodyIndexB;
                    bool isBodyBStatic = bodyIndexB >= numDynamicBodies;

                    if (!isBodyBStatic && phaseIdPerPair[i] != invalidPhaseId)
                    {
                        int phaseForPair = phaseIdPerPair[i];
                        int indexInArray = offsetInPhase[phaseForPair]++;
                        phasedDispatchPairs[indexInArray] = dispatchPairs[i];
                    }
                }

                // Populate PhasedDispatchPairsArray with dynamic-static pairs
                for (int i = 0; i < dispatchPairs.Length; i++)
                {
                    int bodyIndexB = dispatchPairs[i].BodyIndexB;
                    bool isBodyBStatic = bodyIndexB >= numDynamicBodies;

                    if (isBodyBStatic && phaseIdPerPair[i] != invalidPhaseId)
                    {
                        int phaseForPair = phaseIdPerPair[i];
                        int indexInArray = offsetInPhase[phaseForPair]++;
                        phasedDispatchPairs[indexInArray] = dispatchPairs[i];
                    }
                }

                // Populate SolvePhaseInfo for each solve phase
                int firstWorkItemIndex = 0;
                int numPairs = 0;
                numActivePhases = 0;
                for (int i = 0; i < numPhases; i++)
                {
                    SolverSchedulerInfo.SolvePhaseInfo info;
                    info.DispatchPairCount = numPairsPerPhase[i];

                    if (info.DispatchPairCount == 0)
                    {
                        break;
                    }

                    info.BatchSize = math.min(k_MinBatchSize, info.DispatchPairCount);
                    info.NumWorkItems = (info.DispatchPairCount + info.BatchSize - 1) / info.BatchSize;
                    info.FirstWorkItemIndex = firstWorkItemIndex;
                    info.FirstDispatchPairIndex = numPairs;

                    firstWorkItemIndex += info.NumWorkItems;
                    numPairs += info.DispatchPairCount;

                    phaseInfo[i] = info;
                    numActivePhases++;
                }

                // Uncomment this code when testing scheduler
                //               CheckIntegrity(phasedDispatchPairs, numDynamicBodies, ref phaseInfo);

                //<todo.eoin.usermod Can we get rid of this max()? Needed if the user wants to add contacts themselves.
                numWorkItems = math.max(1, SolverSchedulerInfo.CalculateNumWorkItems(phaseInfo));
            }

            private static void CheckIntegrity(NativeArray<DispatchPair> phasedDispatchPairs,
                int numDynamicBodies, ref NativeArray<SolverSchedulerInfo.SolvePhaseInfo> solverPhaseInfos)
            {
                int dispatchPairCount = 0;
                for (int i = 0; i < solverPhaseInfos.Length; i++)
                {
                    SolverSchedulerInfo.SolvePhaseInfo info = solverPhaseInfos[i];
                    dispatchPairCount += info.DispatchPairCount;

                    var firstWorkItemIndex = info.FirstWorkItemIndex;
                    {
                        for (int j = firstWorkItemIndex; j < firstWorkItemIndex + info.NumWorkItems - 1; j++)
                        {
                            for (int k = j + 1; k < j + 1 + info.NumWorkItems; k++)
                            {
                                int firstIndex = j * info.BatchSize;
                                int secondIndex = k * info.BatchSize;
                                for (int pairIndex1 = firstIndex; pairIndex1 < math.min(firstIndex + info.BatchSize, dispatchPairCount); pairIndex1++)
                                {
                                    int aIndex1 = phasedDispatchPairs[pairIndex1].BodyIndexA;
                                    int bIndex1 = phasedDispatchPairs[pairIndex1].BodyIndexB;
                                    for (int pairIndex2 = secondIndex; pairIndex2 < math.min(secondIndex + info.BatchSize, dispatchPairCount); pairIndex2++)
                                    {
                                        int aIndex2 = phasedDispatchPairs[pairIndex2].BodyIndexA;
                                        int bIndex2 = phasedDispatchPairs[pairIndex2].BodyIndexB;

                                        // Verify that different batches can't contain same dynamic bodies
                                        Assert.AreNotEqual(aIndex1, aIndex2);
                                        if (bIndex1 < numDynamicBodies || bIndex2 < numDynamicBodies)
                                        {
                                            Assert.AreNotEqual(bIndex1, bIndex2);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            private unsafe struct BatchInfo
            {
                internal void Add(NativeArray<ushort> rigidBodyMasks, int bodyIndexA, int bodyIndexB)
                {
                    int indexInBuffer = m_NumElements++ * 2;

                    fixed (int* bodyIndices = m_BodyIndices)
                    {
                        bodyIndices[indexInBuffer++] = bodyIndexA;
                        bodyIndices[indexInBuffer] = bodyIndexB;

                        if (m_NumElements == k_MinBatchSize)
                        {
                            indexInBuffer = 0;
                            for (int i = 0; i < k_MinBatchSize; i++)
                            {
                                rigidBodyMasks[bodyIndices[indexInBuffer++]] |= m_PhaseMask;
                                rigidBodyMasks[bodyIndices[indexInBuffer++]] |= m_PhaseMask;
                            }

                            m_NumBatchesProcessed++;
                            m_NumElements = 0;
                        }
                    }
                }

                internal void CommitRigidBodyMasks(NativeArray<ushort> rigidBodyMasks)
                {
                    // Flush
                    int indexInBuffer = 0;
                    fixed (int* bodyIndices = m_BodyIndices)
                    {
                        for (int i = 0; i < m_NumElements; i++)
                        {
                            rigidBodyMasks[bodyIndices[indexInBuffer++]] |= m_PhaseMask;
                            rigidBodyMasks[bodyIndices[indexInBuffer++]] |= m_PhaseMask;
                        }
                    }
                }

                private fixed int m_BodyIndices[k_MinBatchSize * 2];

                internal int m_NumBatchesProcessed;
                internal ushort m_PhaseMask;
                internal int m_NumElements;
            }

            private static int FindFreePhaseDynamicDynamicPair(NativeArray<ushort> rigidBodyMask, int bodyIndexA, int bodyIndexB,
                int lastPhaseIndex, NativeArray<ushort> phaseLookupTableDynamicDynamicPairs, bool useTable = true)
            {
                int mask = rigidBodyMask[bodyIndexA] | rigidBodyMask[bodyIndexB];
                int phaseIndex = -1;
                if (useTable)
                {
                    phaseIndex = phaseLookupTableDynamicDynamicPairs[mask];
                }
                else
                {
                    if (mask == ushort.MaxValue)
                    {
                        phaseIndex = lastPhaseIndex;
                    }
                    else
                    {
                        // Find index of first zero bit
                        for (ushort i = 0; i <= lastPhaseIndex; i++)
                        {
                            if ((mask & 1) == 0)
                            {
                                phaseIndex = i;

                                break;
                            }

                            mask >>= 1;
                        }
                    }
                }

                Assert.IsTrue(phaseIndex >= 0 && phaseIndex <= lastPhaseIndex);
                return phaseIndex;
            }

            private static int FindFreePhaseDynamicStaticPair(NativeArray<ushort> rigidBodyMask, int bodyIndexA,
                int lastPhaseIndex, NativeArray<ushort> phaseLookupTableDynamicStaticPairs, bool useTable = true)
            {
                int mask = rigidBodyMask[bodyIndexA];

                int phaseIndex;
                if (useTable)
                {
                    phaseIndex = phaseLookupTableDynamicStaticPairs[mask];
                }
                else
                {
                    if (mask == ushort.MaxValue)
                    {
                        phaseIndex = lastPhaseIndex;
                    }
                    else
                    {
                        // Find the first zero bit after last one bit
                        phaseIndex = 0;
                        for (int i = lastPhaseIndex; i >= 0; i--)
                        {
                            if ((mask & (1 << i)) != 0)
                            {
                                phaseIndex = i + 1;
                                break;
                            }
                        }

                        phaseIndex = math.min(lastPhaseIndex, phaseIndex);
                    }
                }

                Assert.IsTrue(phaseIndex >= 0 && phaseIndex <= lastPhaseIndex);
                return phaseIndex;
            }
        }

        #endregion
    }
}
