using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using UnityS.Mathematics;

namespace UnityS.Transforms
{
    [Serializable]
    [WriteGroup(typeof(LocalToWorld))]
    [WriteGroup(typeof(LocalToParent))]
    public struct CompositeRotation : IComponentData
    {
        public float4x4 Value;
    }

    [Serializable]
    [WriteGroup(typeof(CompositeRotation))]
    public struct PostRotation : IComponentData
    {
        public quaternion Value;
    }

    [Serializable]
    [WriteGroup(typeof(CompositeRotation))]
    public struct RotationPivot : IComponentData
    {
        public float3 Value;
    }

    [Serializable]
    [WriteGroup(typeof(CompositeRotation))]
    public struct RotationPivotTranslation : IComponentData
    {
        public float3 Value;
    }

    // CompositeRotation = RotationPivotTranslation * RotationPivot * Rotation * PostRotation * RotationPivot^-1
    public abstract class CompositeRotationSystem : JobComponentSystem
    {
        private EntityQuery m_Group;

        [BurstCompile]
        struct ToCompositeRotation : IJobChunk
        {
            [ReadOnly] public ComponentTypeHandle<PostRotation> PostRotationTypeHandle;
            [ReadOnly] public ComponentTypeHandle<Rotation> RotationTypeHandle;
            [ReadOnly] public ComponentTypeHandle<RotationPivot> RotationPivotTypeHandle;
            [ReadOnly] public ComponentTypeHandle<RotationPivotTranslation> RotationPivotTranslationTypeHandle;
            public ComponentTypeHandle<CompositeRotation> CompositeRotationTypeHandle;
            public uint LastSystemVersion;

            public void Execute(ArchetypeChunk chunk, int index, int entityOffset)
            {
                var chunkRotationPivotTranslations = chunk.GetNativeArray(RotationPivotTranslationTypeHandle);
                var chunkRotations = chunk.GetNativeArray(RotationTypeHandle);
                var chunkPostRotation = chunk.GetNativeArray(PostRotationTypeHandle);
                var chunkRotationPivots = chunk.GetNativeArray(RotationPivotTypeHandle);
                var chunkCompositeRotations = chunk.GetNativeArray(CompositeRotationTypeHandle);

                var hasRotationPivotTranslation = chunk.Has(RotationPivotTranslationTypeHandle);
                var hasRotation = chunk.Has(RotationTypeHandle);
                var hasPostRotation = chunk.Has(PostRotationTypeHandle);
                var hasRotationPivot = chunk.Has(RotationPivotTypeHandle);
                var count = chunk.Count;

                var hasAnyRotation = hasRotation || hasPostRotation;

                // 000 - Invalid. Must have at least one.
                // 001
                if (!hasAnyRotation && !hasRotationPivotTranslation && hasRotationPivot)
                {
                    var didChange = chunk.DidChange(RotationPivotTypeHandle, LastSystemVersion);
                    if (!didChange)
                        return;

                    // Only pivot? Doesn't do anything.
                    for (int i = 0; i < count; i++)
                        chunkCompositeRotations[i] = new CompositeRotation {Value = float4x4.identity};
                }
                // 010
                else if (!hasAnyRotation && hasRotationPivotTranslation && !hasRotationPivot)
                {
                    var didChange = chunk.DidChange(RotationPivotTranslationTypeHandle, LastSystemVersion);
                    if (!didChange)
                        return;

                    for (int i = 0; i < count; i++)
                    {
                        var translation = chunkRotationPivotTranslations[i].Value;

                        chunkCompositeRotations[i] = new CompositeRotation
                        {Value = float4x4.Translate(translation)};
                    }
                }
                // 011
                else if (!hasAnyRotation && hasRotationPivotTranslation && hasRotationPivot)
                {
                    var didChange = chunk.DidChange(RotationPivotTranslationTypeHandle, LastSystemVersion);
                    if (!didChange)
                        return;

                    // Pivot without rotation doesn't affect anything. Only translation.
                    for (int i = 0; i < count; i++)
                    {
                        var translation = chunkRotationPivotTranslations[i].Value;

                        chunkCompositeRotations[i] = new CompositeRotation
                        {Value = float4x4.Translate(translation)};
                    }
                }
                // 100
                else if (hasAnyRotation && !hasRotationPivotTranslation && !hasRotationPivot)
                {
                    // 00 - Not valid
                    // 01
                    if (!hasPostRotation && hasRotation)
                    {
                        var didChange = chunk.DidChange(RotationTypeHandle, LastSystemVersion);
                        if (!didChange)
                            return;

                        for (int i = 0; i < count; i++)
                        {
                            var rotation = chunkRotations[i].Value;

                            chunkCompositeRotations[i] = new CompositeRotation
                            {Value = new float4x4(rotation, float3.zero)};
                        }
                    }
                    // 10
                    else if (hasPostRotation && !hasRotation)
                    {
                        var didChange = chunk.DidChange(PostRotationTypeHandle, LastSystemVersion);
                        if (!didChange)
                            return;

                        for (int i = 0; i < count; i++)
                        {
                            var rotation = chunkPostRotation[i].Value;

                            chunkCompositeRotations[i] = new CompositeRotation
                            {Value = new float4x4(rotation, float3.zero)};
                        }
                    }
                    // 11
                    else if (hasPostRotation && hasRotation)
                    {
                        var didChange = chunk.DidChange(PostRotationTypeHandle, LastSystemVersion) ||
                            chunk.DidChange(RotationTypeHandle, LastSystemVersion);
                        if (!didChange)
                            return;

                        for (int i = 0; i < count; i++)
                        {
                            var rotation = math.mul(chunkRotations[i].Value, chunkPostRotation[i].Value);

                            chunkCompositeRotations[i] = new CompositeRotation
                            {Value = new float4x4(rotation, float3.zero)};
                        }
                    }
                }
                // 101
                else if (hasAnyRotation && !hasRotationPivotTranslation && hasRotationPivot)
                {
                    // 00 - Not valid
                    // 01
                    if (!hasPostRotation && hasRotation)
                    {
                        var didChange = chunk.DidChange(RotationTypeHandle, LastSystemVersion) ||
                            chunk.DidChange(RotationPivotTypeHandle, LastSystemVersion);
                        if (!didChange)
                            return;

                        for (int i = 0; i < count; i++)
                        {
                            var rotation = chunkRotations[i].Value;
                            var pivot = chunkRotationPivots[i].Value;
                            var inversePivot = -pivot;

                            chunkCompositeRotations[i] = new CompositeRotation
                            {Value = math.mul(new float4x4(rotation, pivot), float4x4.Translate(inversePivot))};
                        }
                    }
                    // 10
                    else if (hasPostRotation && !hasRotation)
                    {
                        var didChange = chunk.DidChange(PostRotationTypeHandle, LastSystemVersion) ||
                            chunk.DidChange(RotationPivotTypeHandle, LastSystemVersion);
                        if (!didChange)
                            return;

                        for (int i = 0; i < count; i++)
                        {
                            var rotation = chunkPostRotation[i].Value;
                            var pivot = chunkRotationPivots[i].Value;
                            var inversePivot = -pivot;

                            chunkCompositeRotations[i] = new CompositeRotation
                            {Value = math.mul(new float4x4(rotation, pivot), float4x4.Translate(inversePivot))};
                        }
                    }
                    // 11
                    else if (hasPostRotation && hasRotation)
                    {
                        var didChange = chunk.DidChange(PostRotationTypeHandle, LastSystemVersion) ||
                            chunk.DidChange(RotationTypeHandle, LastSystemVersion) ||
                            chunk.DidChange(RotationPivotTypeHandle, LastSystemVersion);
                        if (!didChange)
                            return;

                        for (int i = 0; i < count; i++)
                        {
                            var rotation = chunkPostRotation[i].Value;
                            var pivot = chunkRotationPivots[i].Value;
                            var inversePivot = -pivot;

                            chunkCompositeRotations[i] = new CompositeRotation
                            {Value = math.mul(new float4x4(rotation, pivot), float4x4.Translate(inversePivot))};
                        }
                    }
                }
                // 110
                else if (hasAnyRotation && hasRotationPivotTranslation && !hasRotationPivot)
                {
                    // 00 - Not valid
                    // 01
                    if (!hasPostRotation && hasRotation)
                    {
                        var didChange = chunk.DidChange(RotationTypeHandle, LastSystemVersion) ||
                            chunk.DidChange(RotationPivotTranslationTypeHandle, LastSystemVersion);
                        if (!didChange)
                            return;

                        for (int i = 0; i < count; i++)
                        {
                            var translation = chunkRotationPivotTranslations[i].Value;
                            var rotation = chunkRotations[i].Value;

                            chunkCompositeRotations[i] = new CompositeRotation
                            {Value = new float4x4(rotation, translation)};
                        }
                    }
                    // 10
                    else if (hasPostRotation && !hasRotation)
                    {
                        var didChange = chunk.DidChange(PostRotationTypeHandle, LastSystemVersion) ||
                            chunk.DidChange(RotationPivotTranslationTypeHandle, LastSystemVersion);
                        if (!didChange)
                            return;

                        for (int i = 0; i < count; i++)
                        {
                            var translation = chunkRotationPivotTranslations[i].Value;
                            var rotation = chunkRotations[i].Value;

                            chunkCompositeRotations[i] = new CompositeRotation
                            {Value = new float4x4(rotation, translation)};
                        }
                    }
                    // 11
                    else if (hasPostRotation && hasRotation)
                    {
                        var didChange = chunk.DidChange(PostRotationTypeHandle, LastSystemVersion) ||
                            chunk.DidChange(RotationPivotTranslationTypeHandle, LastSystemVersion) ||
                            chunk.DidChange(RotationTypeHandle, LastSystemVersion);
                        if (!didChange)
                            return;

                        for (int i = 0; i < count; i++)
                        {
                            var translation = chunkRotationPivotTranslations[i].Value;
                            var rotation = math.mul(chunkRotations[i].Value, chunkPostRotation[i].Value);

                            chunkCompositeRotations[i] = new CompositeRotation
                            {Value = new float4x4(rotation, translation)};
                        }
                    }
                }
                // 111
                else if (hasAnyRotation && hasRotationPivotTranslation && hasRotationPivot)
                {
                    // 00 - Not valid
                    // 01
                    if (!hasPostRotation && hasRotation)
                    {
                        var didChange = chunk.DidChange(RotationTypeHandle, LastSystemVersion) ||
                            chunk.DidChange(RotationPivotTranslationTypeHandle, LastSystemVersion) ||
                            chunk.DidChange(RotationPivotTypeHandle, LastSystemVersion);
                        if (!didChange)
                            return;

                        for (int i = 0; i < count; i++)
                        {
                            var translation = chunkRotationPivotTranslations[i].Value;
                            var rotation = chunkRotations[i].Value;
                            var pivot = chunkRotationPivots[i].Value;
                            var inversePivot = -pivot;

                            chunkCompositeRotations[i] = new CompositeRotation
                            {
                                Value = math.mul(float4x4.Translate(translation),
                                    math.mul(new float4x4(rotation, pivot), float4x4.Translate(inversePivot)))
                            };
                        }
                    }
                    // 10
                    else if (hasPostRotation && !hasRotation)
                    {
                        var didChange = chunk.DidChange(PostRotationTypeHandle, LastSystemVersion) ||
                            chunk.DidChange(RotationPivotTranslationTypeHandle, LastSystemVersion) ||
                            chunk.DidChange(RotationPivotTypeHandle, LastSystemVersion);
                        if (!didChange)
                            return;

                        for (int i = 0; i < count; i++)
                        {
                            var translation = chunkRotationPivotTranslations[i].Value;
                            var rotation = chunkPostRotation[i].Value;
                            var pivot = chunkRotationPivots[i].Value;
                            var inversePivot = -pivot;

                            chunkCompositeRotations[i] = new CompositeRotation
                            {
                                Value = math.mul(float4x4.Translate(translation),
                                    math.mul(new float4x4(rotation, pivot), float4x4.Translate(inversePivot)))
                            };
                        }
                    }
                    // 11
                    else if (hasPostRotation && hasRotation)
                    {
                        var didChange = chunk.DidChange(PostRotationTypeHandle, LastSystemVersion) ||
                            chunk.DidChange(RotationTypeHandle, LastSystemVersion) ||
                            chunk.DidChange(RotationPivotTranslationTypeHandle, LastSystemVersion) ||
                            chunk.DidChange(RotationPivotTypeHandle, LastSystemVersion);
                        if (!didChange)
                            return;

                        for (int i = 0; i < count; i++)
                        {
                            var translation = chunkRotationPivotTranslations[i].Value;
                            var rotation = math.mul(chunkRotations[i].Value, chunkPostRotation[i].Value);
                            var pivot = chunkRotationPivots[i].Value;
                            var inversePivot = -pivot;

                            chunkCompositeRotations[i] = new CompositeRotation
                            {
                                Value = math.mul(float4x4.Translate(translation),
                                    math.mul(new float4x4(rotation, pivot), float4x4.Translate(inversePivot)))
                            };
                        }
                    }
                }
            }
        }

        protected override void OnCreate()
        {
            m_Group = GetEntityQuery(new EntityQueryDesc
            {
                All = new ComponentType[]
                {
                    typeof(CompositeRotation)
                },
                Any = new ComponentType[]
                {
                    ComponentType.ReadOnly<Rotation>(),
                    ComponentType.ReadOnly<PostRotation>(),
                    ComponentType.ReadOnly<RotationPivot>(),
                    ComponentType.ReadOnly<RotationPivotTranslation>()
                },
                Options = EntityQueryOptions.FilterWriteGroup
            });
        }

        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            var compositeRotationType = GetComponentTypeHandle<CompositeRotation>(false);
            var rotationType = GetComponentTypeHandle<Rotation>(true);
            var preRotationType = GetComponentTypeHandle<PostRotation>(true);
            var rotationPivotTranslationType = GetComponentTypeHandle<RotationPivotTranslation>(true);
            var rotationPivotType = GetComponentTypeHandle<RotationPivot>(true);

            var toCompositeRotationJob = new ToCompositeRotation
            {
                CompositeRotationTypeHandle = compositeRotationType,
                PostRotationTypeHandle = preRotationType,
                RotationTypeHandle = rotationType,
                RotationPivotTypeHandle = rotationPivotType,
                RotationPivotTranslationTypeHandle = rotationPivotTranslationType,
                LastSystemVersion = LastSystemVersion
            };
            var toCompositeRotationJobHandle = toCompositeRotationJob.Schedule(m_Group, inputDeps);
            return toCompositeRotationJobHandle;
        }
    }
}
