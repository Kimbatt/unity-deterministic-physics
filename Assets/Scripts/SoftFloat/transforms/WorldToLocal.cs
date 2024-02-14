using System;
using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using UnityS.Mathematics;

namespace UnityS.Transforms
{
    [Serializable]
    public struct WorldToLocal : IComponentData
    {
        public float4x4 Value;

        public float3 Right => new float3(Value.c0.x, Value.c0.y, Value.c0.z);
        public float3 Up => new float3(Value.c1.x, Value.c1.y, Value.c1.z);
        public float3 Forward => new float3(Value.c2.x, Value.c2.y, Value.c2.z);
        public float3 Position => new float3(Value.c3.x, Value.c3.y, Value.c3.z);
    }

    public abstract partial class WorldToLocalSystem : SystemBase
    {
        private EntityQuery m_Group;

        [BurstCompile]
        struct ToWorldToLocal : IJobChunk
        {
            [ReadOnly] public ComponentTypeHandle<LocalToWorld> LocalToWorldTypeHandle;
            public ComponentTypeHandle<WorldToLocal> WorldToLocalTypeHandle;
            public uint LastSystemVersion;

            public void Execute(in ArchetypeChunk chunk, int chunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
            {
                if (!chunk.DidChange(LocalToWorldTypeHandle, LastSystemVersion))
                    return;

                var chunkLocalToWorld = chunk.GetNativeArray(LocalToWorldTypeHandle);
                var chunkWorldToLocal = chunk.GetNativeArray(WorldToLocalTypeHandle);

                for (int i = 0; i < chunk.Count; i++)
                {
                    var localToWorld = chunkLocalToWorld[i].Value;
                    chunkWorldToLocal[i] = new WorldToLocal { Value = math.inverse(localToWorld) };
                }
            }
        }

        protected override void OnCreate()
        {
            m_Group = GetEntityQuery(new EntityQueryDesc
            {
                All = new ComponentType[]
                {
                    typeof(WorldToLocal),
                    ComponentType.ReadOnly<LocalToWorld>(),
                },
                Options = EntityQueryOptions.FilterWriteGroup
            });
        }

        protected override void OnUpdate()
        {
            var toWorldToLocalJob = new ToWorldToLocal
            {
                LocalToWorldTypeHandle = GetComponentTypeHandle<LocalToWorld>(true),
                WorldToLocalTypeHandle = GetComponentTypeHandle<WorldToLocal>(),
                LastSystemVersion = LastSystemVersion
            };
            Dependency = toWorldToLocalJob.ScheduleParallel(m_Group, this.Dependency);
        }
    }
}