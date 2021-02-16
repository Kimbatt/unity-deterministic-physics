using System;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using UnityS.Mathematics;
using UnityEngine.Assertions;
using static UnityS.Physics.Math;

namespace UnityS.Physics
{
    // Utilities for building bounding volume hierarchies
    public partial struct BoundingVolumeHierarchy
    {
        public struct Constants
        {
            public const int MaxNumTreeBranches = 64;
            public const int SmallRangeSize = 32;
            public const int UnaryStackSize = 256;
            public const int BinaryStackSize = 512;
        }

        public struct PointAndIndex
        {
            public float3 Position;
            public int Index;
        }

        /// <summary>
        /// Builder.
        /// </summary>
        public unsafe struct Builder
        {
            /// <summary>
            /// Range.
            /// </summary>
            public struct Range
            {
                public Range(int start, int length, int root, Aabb domain)
                {
                    Start = start;
                    Length = length;
                    Root = root;
                    Domain = domain;
                }

                public int Start;
                public int Length;
                public int Root;
                public Aabb Domain;
            }

            void SortRange(int axis, ref Range range)
            {
                for (int i = range.Start; i < range.Start + range.Length; ++i)
                {
                    PointAndIndex value = Points[i];
                    sfloat key = value.Position[axis];
                    int j = i;
                    while (j > range.Start && key < Points[j - 1].Position[axis])
                    {
                        Points[j] = Points[j - 1];
                        j--;
                    }
                    Points[j] = value;
                }
            }

            /// <summary>
            /// Compute axis and pivot of a given range.
            /// </summary>
            /// <param name="range"></param>
            /// <param name="axis"></param>
            /// <param name="pivot"></param>
            static void ComputeAxisAndPivot(ref Range range, out int axis, out sfloat pivot)
            {
                // Compute axis and pivot.
                axis = IndexOfMaxComponent(range.Domain.Extents);
                pivot = ((range.Domain.Min + range.Domain.Max) / 2)[axis];
            }

            static void SplitRange(ref Range range, int size, ref Range lRange, ref Range rRange)
            {
                lRange.Start = range.Start;
                lRange.Length = size;
                rRange.Start = lRange.Start + lRange.Length;
                rRange.Length = range.Length - lRange.Length;
            }

            struct CompareVertices : IComparer<float4>
            {
                public int Compare(float4 x, float4 y)
                {
                    return x[SortAxis].CompareTo(y[SortAxis]);
                }

                public int SortAxis;
            }

            void ProcessAxis(int rangeLength, int axis, NativeArray<sfloat> scores, NativeArray<float4> points, ref int bestAxis, ref int pivot, ref sfloat minScore)
            {
                CompareVertices comparator;
                comparator.SortAxis = axis;
                NativeSortExtension.Sort((float4*)points.GetUnsafePtr(), rangeLength, comparator);

                PointAndIndex* p = (PointAndIndex*)points.GetUnsafePtr();

                Aabb runningAabb = Aabb.Empty;

                for (int i = 0; i < rangeLength; i++)
                {
                    runningAabb.Include(Aabbs[p[i].Index]);
                    scores[i] = (sfloat)(i + 1) * runningAabb.SurfaceArea;
                }

                runningAabb = Aabb.Empty;

                for (int i = rangeLength - 1, j = 1; i > 0; --i, ++j)
                {
                    runningAabb.Include(Aabbs[p[i].Index]);
                    sfloat sum = scores[i - 1] + (sfloat)j * runningAabb.SurfaceArea;
                    if (sum < minScore)
                    {
                        pivot = i;
                        bestAxis = axis;
                        minScore = sum;
                    }
                }
            }

            void SegregateSah3(Range range, int minItems, ref Range lRange, ref Range rRange)
            {
                if (!ScratchScores.IsCreated)
                {
                    ScratchScores = new NativeArray<sfloat>(Aabbs.Length, Allocator.Temp);
                    ScratchPointsX = new NativeArray<float4>(Aabbs.Length, Allocator.Temp);
                    ScratchPointsY = new NativeArray<float4>(Aabbs.Length, Allocator.Temp);
                    ScratchPointsZ = new NativeArray<float4>(Aabbs.Length, Allocator.Temp);
                }
                
                // This code relies on range.length always being less than or equal to the number of primitives, which 
                // happens to be Aabbs.length.  If that ever becomes not true then scratch memory size should be increased.
                Assert.IsTrue(range.Length <= ScratchScores.Length/*, "Aabbs.Length isn't a large enough scratch memory size for SegregateSah3"*/);
                
                float4* p = PointsAsFloat4 + range.Start;

                for (int i = 0; i < range.Length; i++)
                {
                    ScratchPointsX[i] = p[i];
                    ScratchPointsY[i] = p[i];
                    ScratchPointsZ[i] = p[i];
                }

                int bestAxis = -1, pivot = -1;
                sfloat minScore = sfloat.MaxValue;

                ProcessAxis(range.Length, 0, ScratchScores, ScratchPointsX, ref bestAxis, ref pivot, ref minScore);
                ProcessAxis(range.Length, 1, ScratchScores, ScratchPointsY, ref bestAxis, ref pivot, ref minScore);
                ProcessAxis(range.Length, 2, ScratchScores, ScratchPointsZ, ref bestAxis, ref pivot, ref minScore);

                // Build sub-ranges.
                int lSize = pivot;
                int rSize = range.Length - lSize;
                if (lSize < minItems || rSize < minItems)
                {
                    // Make sure sub-ranges contains at least minItems nodes, in these rare cases (i.e. all points at the same position), we just split the set in half regardless of positions.
                    SplitRange(ref range, range.Length / 2, ref lRange, ref rRange);
                }
                else
                {
                    SplitRange(ref range, lSize, ref lRange, ref rRange);
                }

                float4* sortedPoints;

                if (bestAxis == 0)
                {
                    sortedPoints = (float4*)ScratchPointsX.GetUnsafePtr();
                }
                else if (bestAxis == 1)
                {
                    sortedPoints = (float4*)ScratchPointsY.GetUnsafePtr();
                }
                else // bestAxis == 2
                {
                    sortedPoints = (float4*)ScratchPointsZ.GetUnsafePtr();
                }

                // Write back sorted points.
                for (int i = 0; i < range.Length; i++)
                {
                    p[i] = sortedPoints[i];
                }
            }


            void Segregate(int axis, sfloat pivot, Range range, int minItems, ref Range lRange, ref Range rRange)
            {
                Assert.IsTrue(range.Length > 1/*, "Range length must be greater than 1."*/);

                Aabb lDomain = Aabb.Empty;
                Aabb rDomain = Aabb.Empty;

                float4* p = PointsAsFloat4;
                float4* start = p + range.Start;
                float4* end = start + range.Length - 1;

                do
                {
                    // Consume left.

                    while (start <= end && (*start)[axis] < pivot)
                    {
                        lDomain.Include((*(start++)).xyz);
                    }

                    // Consume right.
                    while (end > start && (*end)[axis] >= pivot)
                    {
                        rDomain.Include((*(end--)).xyz);
                    }

                    if (start >= end) goto FINISHED;

                    lDomain.Include((*end).xyz);
                    rDomain.Include((*start).xyz);

                    Swap(ref *(start++), ref *(end--));
                } while (true);
            FINISHED:
                // Build sub-ranges.
                int lSize = (int)(start - p);
                int rSize = range.Length - lSize;
                if (lSize < minItems || rSize < minItems)
                {
                    // Make sure sub-ranges contains at least minItems nodes, in these rare cases (i.e. all points at the same position), we just split the set in half regardless of positions.
                    SplitRange(ref range, range.Length / 2, ref lRange, ref rRange);

                    SetAabbFromPoints(ref lDomain, PointsAsFloat4 + lRange.Start, lRange.Length);
                    SetAabbFromPoints(ref rDomain, PointsAsFloat4 + rRange.Start, rRange.Length);
                }
                else
                {
                    SplitRange(ref range, lSize, ref lRange, ref rRange);
                }

                lRange.Domain = lDomain;
                rRange.Domain = rDomain;
            }

            void CreateChildren(Range* subRanges, int numSubRanges, int parentNodeIndex, ref int freeNodeIndex, Range* rangeStack, ref int stackSize)
            {
                int4 parentData = int4.zero;

                for (int i = 0; i < numSubRanges; i++)
                {
                    // Add child node.
                    int childNodeIndex = freeNodeIndex++;
                    parentData[i] = childNodeIndex;

                    if (subRanges[i].Length > 4)
                    {
                        // Keep splitting the range, push it on the stack.
                        rangeStack[stackSize] = subRanges[i];
                        rangeStack[stackSize++].Root = childNodeIndex;
                    }
                    else
                    {
                        Node* childNode = GetNode(childNodeIndex);
                        childNode->IsLeaf = true;

                        for (int pointIndex = 0; pointIndex < subRanges[i].Length; pointIndex++)
                        {
                            childNode->Data[pointIndex] = Points[subRanges[i].Start + pointIndex].Index;
                        }

                        for (int j = subRanges[i].Length; j < 4; j++)
                        {
                            childNode->ClearLeafData(j);
                        }
                    }
                }

                Node* parentNode = GetNode(parentNodeIndex);
                parentNode->Data = parentData;
                parentNode->IsInternal = true;
            }

            Node* GetNode(int nodeIndex) => Bvh.m_Nodes + nodeIndex;

            float4* PointsAsFloat4 => (float4*)Points.GetUnsafePtr();

            void ProcessSmallRange(Range baseRange, ref int freeNodeIndex)
            {
                Range range = baseRange;

                ComputeAxisAndPivot(ref range, out int axis, out sfloat pivot);
                SortRange(axis, ref range);

                Range* subRanges = stackalloc Range[4];
                int hasLeftOvers = 1;
                do
                {
                    int numSubRanges = 0;
                    while (range.Length > 4 && numSubRanges < 3)
                    {
                        subRanges[numSubRanges].Start = range.Start;
                        subRanges[numSubRanges].Length = 4;
                        numSubRanges++;

                        range.Start += 4;
                        range.Length -= 4;
                    }

                    if (range.Length > 0)
                    {
                        subRanges[numSubRanges].Start = range.Start;
                        subRanges[numSubRanges].Length = range.Length;

                        numSubRanges++;
                    }

                    hasLeftOvers = 0;
                    CreateChildren(subRanges, numSubRanges, range.Root, ref freeNodeIndex, &range, ref hasLeftOvers);

                    Assert.IsTrue(hasLeftOvers <= 1/*, "Internal error"*/);
                } while (hasLeftOvers > 0);
            }

            public void ProcessLargeRange(Range range, Range* subRanges)
            {
                if (!UseSah)
                {
                    ComputeAxisAndPivot(ref range, out int axis, out sfloat pivot);

                    Range* temps = stackalloc Range[2];
                    Segregate(axis, pivot, range, 2, ref temps[0], ref temps[1]);

                    ComputeAxisAndPivot(ref temps[0], out int lAxis, out sfloat lPivot);
                    Segregate(lAxis, lPivot, temps[0], 1, ref subRanges[0], ref subRanges[1]);

                    ComputeAxisAndPivot(ref temps[1], out int rAxis, out sfloat rPivot);
                    Segregate(rAxis, rPivot, temps[1], 1, ref subRanges[2], ref subRanges[3]);
                }
                else
                {
                    Range* temps = stackalloc Range[2];
                    SegregateSah3(range, 2, ref temps[0], ref temps[1]);

                    SegregateSah3(temps[0], 1, ref subRanges[0], ref subRanges[1]);
                    SegregateSah3(temps[1], 1, ref subRanges[2], ref subRanges[3]);
                }
            }

            public void CreateInternalNodes(Range* subRanges, int numSubRanges, int root, Range* rangeStack, ref int stackSize, ref int freeNodeIndex)
            {
                int4 rootData = int4.zero;

                for (int i = 0; i < numSubRanges; ++i)
                {
                    rootData[i] = freeNodeIndex++;
                    rangeStack[stackSize] = subRanges[i];
                    rangeStack[stackSize++].Root = rootData[i];
                }

                Node* rootNode = GetNode(root);
                rootNode->Data = rootData;
                rootNode->IsInternal = true;
            }

            public void Build(Range baseRange)
            {
                Range* ranges = stackalloc Range[Constants.UnaryStackSize];
                int rangeStackSize = 1;
                ranges[0] = baseRange;
                Range* subRanges = stackalloc Range[4];

                if (baseRange.Length > 4)
                {
                    do
                    {
                        Range range = ranges[--rangeStackSize];

                        if (range.Length <= Constants.SmallRangeSize)
                        {
                            ProcessSmallRange(range, ref FreeNodeIndex);
                        }
                        else
                        {
                            ProcessLargeRange(range, subRanges);
                            CreateChildren(subRanges, 4, range.Root, ref FreeNodeIndex, ranges, ref rangeStackSize);
                        }
                    }
                    while (rangeStackSize > 0);
                }
                else
                {
                    CreateChildren(ranges, 1, baseRange.Root, ref FreeNodeIndex, ranges, ref rangeStackSize);
                }
            }

            public BoundingVolumeHierarchy Bvh;
            public NativeArray<PointAndIndex> Points;
            public NativeArray<Aabb> Aabbs;
            public int FreeNodeIndex;
            public bool UseSah;

            // These arrays are only used if SAH is used for BVH building.
            private NativeArray<sfloat> ScratchScores;
            private NativeArray<float4> ScratchPointsX;
            private NativeArray<float4> ScratchPointsY;
            private NativeArray<float4> ScratchPointsZ;
        }

        public unsafe JobHandle ScheduleBuildJobs(
            NativeArray<PointAndIndex> points, NativeArray<Aabb> aabbs, NativeArray<CollisionFilter> bodyFilters, NativeArray<int> shouldDoWork,
            int numThreadsHint, JobHandle inputDeps, int numNodes, NativeArray<Builder.Range> ranges, NativeArray<int> numBranches)
        {
            JobHandle handle = inputDeps;

            var branchNodeOffsets = new NativeArray<int>(Constants.MaxNumTreeBranches, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var oldNumBranches = new NativeArray<int>(1, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            // Build initial branches
            handle = new BuildFirstNLevelsJob
            {
                Points = points,
                Nodes = m_Nodes,
                NodeFilters = m_NodeFilters,
                Ranges = ranges,
                BranchNodeOffsets = branchNodeOffsets,
                BranchCount = numBranches,
                OldBranchCount = oldNumBranches,
                ThreadCount = numThreadsHint,
                ShouldDoWork = shouldDoWork
            }.Schedule(handle);

            // Build branches
            handle = new BuildBranchesJob
            {
                Points = points,
                Aabbs = aabbs,
                BodyFilters = bodyFilters,
                Nodes = m_Nodes,
                NodeFilters = m_NodeFilters,
                Ranges = ranges,
                BranchNodeOffsets = branchNodeOffsets,
                BranchCount = numBranches
            }.ScheduleUnsafeIndex0(numBranches, 1, handle);

            // Note: This job also deallocates the aabbs and lookup arrays on completion
            handle = new FinalizeTreeJob
            {
                Aabbs = aabbs,
                Nodes = m_Nodes,
                NodeFilters = m_NodeFilters,
                LeafFilters = bodyFilters,
                NumNodes = numNodes,
                BranchNodeOffsets = branchNodeOffsets,
                BranchCount = numBranches,
                OldBranchCount = oldNumBranches,
                ShouldDoWork = shouldDoWork
            }.Schedule(handle);

            return handle;
        }

        public unsafe void Build(NativeArray<PointAndIndex> points, NativeArray<Aabb> aabbs, out int nodeCount, bool useSah = false)
        {
            m_Nodes[0] = Node.Empty;

            if (aabbs.Length > 0)
            {
                var builder = new Builder
                {
                    Bvh = this,
                    Points = points,
                    Aabbs = aabbs,
                    FreeNodeIndex = 2,
                    UseSah = useSah
                };

                Aabb aabb = new Aabb();
                SetAabbFromPoints(ref aabb, (float4*)points.GetUnsafePtr(), points.Length);
                builder.Build(new Builder.Range(0, points.Length, 1, aabb));
                nodeCount = builder.FreeNodeIndex;

                Refit(aabbs, 1, builder.FreeNodeIndex - 1);
            }
            else
            {
                // No input AABBs - building a tree for no nodes.
                // Make an empty node for the root, as most algorithms jump to the 1th node.
                m_Nodes[1] = Node.Empty;
                nodeCount = 2;
            }
        }

        // For each node between nodeStartIndex and nodeEnd index, set the collision filter info to the combination of the node's children
        internal unsafe void BuildCombinedCollisionFilter(NativeArray<CollisionFilter> leafFilterInfo, int nodeStartIndex, int nodeEndIndex)
        {
            Node* baseNode = m_Nodes;
            Node* currentNode = baseNode + nodeEndIndex;

            for (int i = nodeEndIndex; i >= nodeStartIndex; i--, currentNode--)
            {
                CollisionFilter combinationFilter = new CollisionFilter();

                if (currentNode->IsLeaf)
                {
                    // We know that at least one child will be valid, so start with that leaf's filter:
                    combinationFilter = leafFilterInfo[currentNode->Data[0]];
                    for (int j = 1; j < 4; ++j)
                    {
                        if (currentNode->IsLeafValid(j))
                        {
                            CollisionFilter leafFilter = leafFilterInfo[currentNode->Data[j]];
                            combinationFilter = CollisionFilter.CreateUnion(combinationFilter, leafFilter);
                        }
                    }
                }
                else
                {
                    combinationFilter = m_NodeFilters[currentNode->Data[0]];
                    for (int j = 1; j < 4; j++)
                    {
                        if (currentNode->IsInternalValid(j))
                        {
                            CollisionFilter nodeFilter = m_NodeFilters[currentNode->Data[j]];
                            combinationFilter = CollisionFilter.CreateUnion(combinationFilter, nodeFilter);
                        }
                    }
                }

                m_NodeFilters[i] = combinationFilter;
            }
        }

        // Set the collision filter on nodeIndex to the combination of all it's child filters. Node must not be a leaf.
        unsafe void BuildCombinedCollisionFilter(int nodeIndex)
        {
            Node* baseNode = m_Nodes;
            Node* currentNode = baseNode + nodeIndex;

            Assert.IsTrue(currentNode->IsInternal);

            CollisionFilter combinedFilter = new CollisionFilter();
            for (int j = 0; j < 4; j++)
            {
                combinedFilter = CollisionFilter.CreateUnion(combinedFilter, m_NodeFilters[currentNode->Data[j]]);
            }

            m_NodeFilters[nodeIndex] = combinedFilter;
        }


        public unsafe void Refit(NativeArray<Aabb> aabbs, int nodeStartIndex, int nodeEndIndex)
        {
            Node* baseNode = m_Nodes;
            Node* currentNode = baseNode + nodeEndIndex;

            for (int i = nodeEndIndex; i >= nodeStartIndex; i--, currentNode--)
            {
                if (currentNode->IsLeaf)
                {
                    for (int j = 0; j < 4; ++j)
                    {
                        Aabb aabb;
                        if (currentNode->IsLeafValid(j))
                        {
                            aabb = aabbs[currentNode->Data[j]];
                        }
                        else
                        {
                            aabb = Aabb.Empty;
                        }

                        currentNode->Bounds.SetAabb(j, aabb);
                    }
                }
                else
                {
                    for (int j = 0; j < 4; j++)
                    {
                        Aabb aabb;
                        if (currentNode->IsInternalValid(j))
                        {
                            aabb = baseNode[currentNode->Data[j]].Bounds.GetCompoundAabb();
                        }
                        else
                        {
                            aabb = Aabb.Empty;
                        }

                        currentNode->Bounds.SetAabb(j, aabb);
                    }
                }
            }
        }

        unsafe void RefitNode(int nodeIndex)
        {
            Node* baseNode = m_Nodes;
            Node* currentNode = baseNode + nodeIndex;

            Assert.IsTrue(currentNode->IsInternal);

            for (int j = 0; j < 4; j++)
            {
                Aabb compoundAabb = baseNode[currentNode->Data[j]].Bounds.GetCompoundAabb();
                currentNode->Bounds.SetAabb(j, compoundAabb);
            }
        }

        private struct RangeSizeAndIndex
        {
            public int RangeIndex;
            public int RangeSize;
            public int RangeFirstNodeOffset;
        }

        unsafe void SortRangeMap(RangeSizeAndIndex* rangeMap, int numElements)
        {
            for (int i = 0; i < numElements; i++)
            {
                RangeSizeAndIndex value = rangeMap[i];
                int key = rangeMap[i].RangeSize;
                int j = i;
                while (j > 0 && key > rangeMap[j - 1].RangeSize)
                {
                    rangeMap[j] = rangeMap[j - 1];
                    j--;
                }

                rangeMap[j] = value;
            }
        }

        internal unsafe void BuildFirstNLevels(
            NativeArray<PointAndIndex> points,
            NativeArray<Builder.Range> branchRanges, NativeArray<int> branchNodeOffset,
            int threadCount, out int branchCount)
        {
            Builder.Range* level0 = stackalloc Builder.Range[Constants.MaxNumTreeBranches];
            Builder.Range* level1 = stackalloc Builder.Range[Constants.MaxNumTreeBranches];
            int level0Size = 1;
            int level1Size = 0;

            Aabb aabb = new Aabb();
            SetAabbFromPoints(ref aabb, (float4*)points.GetUnsafePtr(), points.Length);
            level0[0] = new Builder.Range(0, points.Length, 1, aabb);

            int largestAllowedRange = math.max(level0[0].Length / threadCount, Constants.SmallRangeSize);
            int smallRangeThreshold = math.max(largestAllowedRange / threadCount, Constants.SmallRangeSize);
            int largestRangeInLastLevel;
            int maxNumBranchesMinusOneSplit = Constants.MaxNumTreeBranches - 3;
            int freeNodeIndex = 2;

            var builder = new Builder { Bvh = this, Points = points, UseSah = false };

            Builder.Range* subRanges = stackalloc Builder.Range[4];

            do
            {
                largestRangeInLastLevel = 0;

                for (int i = 0; i < level0Size; ++i)
                {
                    if (level0[i].Length > smallRangeThreshold && freeNodeIndex < maxNumBranchesMinusOneSplit)
                    {
                        // Split range in up to 4 sub-ranges.

                        builder.ProcessLargeRange(level0[i], subRanges);

                        largestRangeInLastLevel = math.max(largestRangeInLastLevel, subRanges[0].Length);
                        largestRangeInLastLevel = math.max(largestRangeInLastLevel, subRanges[1].Length);
                        largestRangeInLastLevel = math.max(largestRangeInLastLevel, subRanges[2].Length);
                        largestRangeInLastLevel = math.max(largestRangeInLastLevel, subRanges[3].Length);

                        // Create nodes for the sub-ranges and append level 1 sub-ranges.
                        builder.CreateInternalNodes(subRanges, 4, level0[i].Root, level1, ref level1Size, ref freeNodeIndex);
                    }
                    else
                    {
                        // Too small, ignore.
                        level1[level1Size++] = level0[i];
                    }
                }

                Builder.Range* tmp = level0;
                level0 = level1;
                level1 = tmp;

                level0Size = level1Size;
                level1Size = 0;
                smallRangeThreshold = largestAllowedRange;
            } while (level0Size < Constants.MaxNumTreeBranches && largestRangeInLastLevel > largestAllowedRange);

            RangeSizeAndIndex* rangeMapBySize = stackalloc RangeSizeAndIndex[Constants.MaxNumTreeBranches];

            int nodeOffset = freeNodeIndex;
            for (int i = 0; i < level0Size; i++)
            {
                rangeMapBySize[i] = new RangeSizeAndIndex { RangeIndex = i, RangeSize = level0[i].Length, RangeFirstNodeOffset = nodeOffset };
                nodeOffset += level0[i].Length;
            }

            SortRangeMap(rangeMapBySize, level0Size);

            for (int i = 0; i < level0Size; i++)
            {
                branchRanges[i] = level0[rangeMapBySize[i].RangeIndex];
                branchNodeOffset[i] = rangeMapBySize[i].RangeFirstNodeOffset;
            }

            for (int i = level0Size; i < Constants.MaxNumTreeBranches; i++)
            {
                branchNodeOffset[i] = -1;
            }

            branchCount = level0Size;

            m_Nodes[0] = Node.Empty;
        }

        // Build the branch for range. Returns the index of the last built node in the range
        internal int BuildBranch(NativeArray<PointAndIndex> points, NativeArray<Aabb> aabb, Builder.Range range, int firstNodeIndex)
        {
            var builder = new Builder
            {
                Bvh = this,
                Points = points,
                FreeNodeIndex = firstNodeIndex,
                UseSah = false
            };

            builder.Build(range);

            Refit(aabb, firstNodeIndex, builder.FreeNodeIndex - 1);
            RefitNode(range.Root);
            return builder.FreeNodeIndex - 1;
        }

        // helper
        private static unsafe void SetAabbFromPoints(ref Aabb aabb, float4* points, int length)
        {
            aabb.Min = Math.Constants.Max3F;
            aabb.Max = Math.Constants.Min3F;
            for (int i = 0; i < length; i++)
            {
                aabb.Min = math.min(aabb.Min, points[i].xyz);
                aabb.Max = math.max(aabb.Max, points[i].xyz);
            }
        }

        [BurstCompile]
        internal unsafe struct BuildFirstNLevelsJob : IJob
        {
            public NativeArray<PointAndIndex> Points;
            [NativeDisableUnsafePtrRestriction]
            public Node* Nodes;
            [NativeDisableUnsafePtrRestriction]
            public CollisionFilter* NodeFilters;
            public NativeArray<Builder.Range> Ranges;
            public NativeArray<int> BranchNodeOffsets;
            public NativeArray<int> BranchCount;
            public NativeArray<int> OldBranchCount;
            public NativeArray<int> ShouldDoWork;

            public int ThreadCount;

            public void Execute()
            {
                // Save old branch count for finalize tree job
                OldBranchCount[0] = BranchCount[0];

                if (ShouldDoWork[0] == 0)
                {
                    // If we need to to skip tree building tasks, than set BranchCount to zero so
                    // that BuildBranchesJob also gets early out in runtime.
                    BranchCount[0] = 0;
                    return;
                }

                var bvh = new BoundingVolumeHierarchy(Nodes, NodeFilters);
                bvh.BuildFirstNLevels(Points, Ranges, BranchNodeOffsets, ThreadCount, out int branchCount);
                BranchCount[0] = branchCount;
            }
        }

        [BurstCompile]
        internal unsafe struct BuildBranchesJob : IJobParallelForDefer
        {
            [ReadOnly] public NativeArray<Aabb> Aabbs;
            [ReadOnly] public NativeArray<CollisionFilter> BodyFilters;
            [ReadOnly] public NativeArray<Builder.Range> Ranges;
            [ReadOnly] public NativeArray<int> BranchNodeOffsets;
            [ReadOnly] public NativeArray<int> BranchCount;

            [NativeDisableUnsafePtrRestriction]
            public Node* Nodes;
            [NativeDisableUnsafePtrRestriction]
            public CollisionFilter* NodeFilters;

            [NativeDisableContainerSafetyRestriction]
            [DeallocateOnJobCompletion] public NativeArray<PointAndIndex> Points;

            public void Execute(int index)
            {
                Assert.IsTrue(BranchNodeOffsets[index] >= 0);
                var bvh = new BoundingVolumeHierarchy(Nodes, NodeFilters);
                int lastNode = bvh.BuildBranch(Points, Aabbs, Ranges[index], BranchNodeOffsets[index]);

                if (NodeFilters != null)
                {
                    bvh.BuildCombinedCollisionFilter(BodyFilters, BranchNodeOffsets[index], lastNode);
                    bvh.BuildCombinedCollisionFilter(Ranges[index].Root);
                }
            }
        }

        [BurstCompile]
        internal unsafe struct FinalizeTreeJob : IJob
        {
            [ReadOnly] [DeallocateOnJobCompletion] public NativeArray<Aabb> Aabbs;
            [ReadOnly] [DeallocateOnJobCompletion] public NativeArray<int> BranchNodeOffsets;
            [ReadOnly] public NativeArray<CollisionFilter> LeafFilters;
            [ReadOnly] public NativeArray<int> ShouldDoWork;
            [NativeDisableUnsafePtrRestriction]
            public Node* Nodes;
            [NativeDisableUnsafePtrRestriction]
            public CollisionFilter* NodeFilters;
            public int NumNodes;
            [DeallocateOnJobCompletion] [ReadOnly] public NativeArray<int> OldBranchCount;
            public NativeArray<int> BranchCount;

            public void Execute()
            {
                if (ShouldDoWork[0] == 0)
                {
                    // Restore original branch count
                    BranchCount[0] = OldBranchCount[0];
                    return;
                }

                int minBranchNodeIndex = BranchNodeOffsets[0] - 1;
                int branchCount = BranchCount[0];
                for (int i = 1; i < BranchCount[0]; i++)
                {
                    minBranchNodeIndex = math.min(BranchNodeOffsets[i] - 1, minBranchNodeIndex);
                }

                var bvh = new BoundingVolumeHierarchy(Nodes, NodeFilters);
                bvh.Refit(Aabbs, 1, minBranchNodeIndex);

                if (NodeFilters != null)
                {
                    bvh.BuildCombinedCollisionFilter(LeafFilters, 1, minBranchNodeIndex);
                }
            }
        }

        internal unsafe void CheckIntegrity(int nodeIndex = 1, int parentIndex = 0, int childIndex = 0)
        {
            Node parent = m_Nodes[parentIndex];
            Node node = m_Nodes[nodeIndex];
            Aabb parentAabb = parent.Bounds.GetAabb(childIndex);

            for (int i = 0; i < 4; ++i)
            {
                int data = node.Data[i];
                Aabb aabb = node.Bounds.GetAabb(i);

                bool validData = node.IsChildValid(i);

                bool validAabb = aabb.IsValid;

                if (validData != validAabb)
                {
                    SafetyChecks.ThrowInvalidOperationException("Invalid node should have empty AABB.");
                    return;
                }

                if (validData)
                {
                    if (parentIndex != 0)
                    {
                        if (!parentAabb.Contains(aabb))
                        {
                            SafetyChecks.ThrowInvalidOperationException("Parent AABB do not contains child AABB");
                            return;
                        }
                    }

                    if (node.IsInternal)
                    {
                        CheckIntegrity(data, nodeIndex, i);
                    }
                }
            }
        }
    }
}
