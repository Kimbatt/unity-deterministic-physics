using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using UnityS.Mathematics;

namespace UnityS.Transforms
{
    [Serializable]
    [WriteGroup(typeof(LocalToParent))]
    public struct ParentScaleInverse : IComponentData
    {
        public float4x4 Value;

        public float3 Right => new float3(Value.c0.x, Value.c0.y, Value.c0.z);
        public float3 Up => new float3(Value.c1.x, Value.c1.y, Value.c1.z);
        public float3 Forward => new float3(Value.c2.x, Value.c2.y, Value.c2.z);
        public float3 Position => new float3(Value.c3.x, Value.c3.y, Value.c3.z);
    }

    // ParentScaleInverse = Parent.CompositeScale^-1
    // (or) ParentScaleInverse = Parent.Scale^-1
    // (or) ParentScaleInverse = Parent.NonUniformScale^-1
    public abstract class ParentScaleInverseSystem : JobComponentSystem
    {
        private EntityQuery m_Group;

        [BurstCompile]
        struct ToChildParentScaleInverse : IJobChunk
        {
            [ReadOnly] public ComponentTypeHandle<Scale> ScaleTypeHandle;
            [ReadOnly] public ComponentTypeHandle<NonUniformScale> NonUniformScaleTypeHandle;
            [ReadOnly] public ComponentTypeHandle<CompositeScale> CompositeScaleTypeHandle;
            [ReadOnly] public BufferTypeHandle<Child> ChildTypeHandle;
            [NativeDisableContainerSafetyRestriction] public ComponentDataFromEntity<ParentScaleInverse> ParentScaleInverseFromEntity;
            public uint LastSystemVersion;

            public void Execute(ArchetypeChunk chunk, int chunkIndex, int firstEntityIndex)
            {
                var hasScale = chunk.Has(ScaleTypeHandle);
                var hasNonUniformScale = chunk.Has(NonUniformScaleTypeHandle);
                var hasCompositeScale = chunk.Has(CompositeScaleTypeHandle);

                if (hasCompositeScale)
                {
                    var didChange = chunk.DidChange(CompositeScaleTypeHandle, LastSystemVersion) ||
                        chunk.DidChange(ChildTypeHandle, LastSystemVersion);
                    if (!didChange)
                        return;

                    var chunkCompositeScales = chunk.GetNativeArray(CompositeScaleTypeHandle);
                    var chunkChildren = chunk.GetBufferAccessor(ChildTypeHandle);
                    for (var i = 0; i < chunk.Count; i++)
                    {
                        var inverseScale = math.inverse(chunkCompositeScales[i].Value);
                        var children = chunkChildren[i];
                        for (var j = 0; j < children.Length; j++)
                        {
                            var childEntity = children[j].Value;
                            if (!ParentScaleInverseFromEntity.HasComponent(childEntity))
                                continue;

                            ParentScaleInverseFromEntity[childEntity] = new ParentScaleInverse {Value = inverseScale};
                        }
                    }
                }
                else if (hasScale)
                {
                    var didChange = chunk.DidChange(ScaleTypeHandle, LastSystemVersion) ||
                        chunk.DidChange(ChildTypeHandle, LastSystemVersion);
                    if (!didChange)
                        return;

                    var chunkScales = chunk.GetNativeArray(ScaleTypeHandle);
                    var chunkChildren = chunk.GetBufferAccessor(ChildTypeHandle);
                    for (var i = 0; i < chunk.Count; i++)
                    {
                        var inverseScale = float4x4.Scale(sfloat.One / chunkScales[i].Value);
                        var children = chunkChildren[i];
                        for (var j = 0; j < children.Length; j++)
                        {
                            var childEntity = children[j].Value;
                            if (!ParentScaleInverseFromEntity.HasComponent(childEntity))
                                continue;

                            ParentScaleInverseFromEntity[childEntity] = new ParentScaleInverse {Value = inverseScale};
                        }
                    }
                }
                else // if (hasNonUniformScale)
                {
                    var didChange = chunk.DidChange(NonUniformScaleTypeHandle, LastSystemVersion) ||
                        chunk.DidChange(ChildTypeHandle, LastSystemVersion);
                    if (!didChange)
                        return;

                    var chunkNonUniformScales = chunk.GetNativeArray(NonUniformScaleTypeHandle);
                    var chunkChildren = chunk.GetBufferAccessor(ChildTypeHandle);
                    for (var i = 0; i < chunk.Count; i++)
                    {
                        var inverseScale = float4x4.Scale(sfloat.One / chunkNonUniformScales[i].Value);
                        var children = chunkChildren[i];
                        for (var j = 0; j < children.Length; j++)
                        {
                            var childEntity = children[j].Value;
                            if (!ParentScaleInverseFromEntity.HasComponent(childEntity))
                                continue;

                            ParentScaleInverseFromEntity[childEntity] = new ParentScaleInverse {Value = inverseScale};
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
                    ComponentType.ReadOnly<Child>(),
                },
                Any = new ComponentType[]
                {
                    ComponentType.ReadOnly<Scale>(),
                    ComponentType.ReadOnly<NonUniformScale>(),
                    ComponentType.ReadOnly<CompositeScale>(),
                },
                Options = EntityQueryOptions.FilterWriteGroup
            });
        }

        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            var toParentScaleInverseJob = new ToChildParentScaleInverse
            {
                ScaleTypeHandle = GetComponentTypeHandle<Scale>(true),
                NonUniformScaleTypeHandle = GetComponentTypeHandle<NonUniformScale>(true),
                CompositeScaleTypeHandle = GetComponentTypeHandle<CompositeScale>(true),
                ChildTypeHandle = GetBufferTypeHandle<Child>(true),
                ParentScaleInverseFromEntity = GetComponentDataFromEntity<ParentScaleInverse>(),
                LastSystemVersion = LastSystemVersion
            };
            var toParentScaleInverseJobHandle = toParentScaleInverseJob.Schedule(m_Group, inputDeps);
            return toParentScaleInverseJobHandle;
        }
    }
}
