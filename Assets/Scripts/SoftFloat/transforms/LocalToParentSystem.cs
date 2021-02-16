using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using UnityS.Mathematics;

namespace UnityS.Transforms
{
    public abstract class LocalToParentSystem : JobComponentSystem
    {
        private EntityQuery m_RootsGroup;
        private EntityQueryMask m_LocalToWorldWriteGroupMask;

        // LocalToWorld = Parent.LocalToWorld * LocalToParent
        [BurstCompile]
        struct UpdateHierarchy : IJobChunk
        {
            [ReadOnly] public ComponentTypeHandle<LocalToWorld> LocalToWorldTypeHandle;
            [ReadOnly] public BufferTypeHandle<Child> ChildTypeHandle;
            [ReadOnly] public BufferFromEntity<Child> ChildFromEntity;
            [ReadOnly] public ComponentDataFromEntity<LocalToParent> LocalToParentFromEntity;
            [ReadOnly] public EntityQueryMask LocalToWorldWriteGroupMask;
            public uint LastSystemVersion;

            [NativeDisableContainerSafetyRestriction]
            public ComponentDataFromEntity<LocalToWorld> LocalToWorldFromEntity;

            void ChildLocalToWorld(float4x4 parentLocalToWorld, Entity entity, bool updateChildrenTransform)
            {
                bool transformChanged = LocalToParentFromEntity.DidChange(entity, LastSystemVersion);
                updateChildrenTransform = updateChildrenTransform || transformChanged;

                float4x4 localToWorldMatrix;

                if (updateChildrenTransform && LocalToWorldWriteGroupMask.Matches(entity))
                {
                    var localToParent = LocalToParentFromEntity[entity];
                    localToWorldMatrix = math.mul(parentLocalToWorld, localToParent.Value);
                    LocalToWorldFromEntity[entity] = new LocalToWorld {Value = localToWorldMatrix};
                }
                else //This entity has a component with the WriteGroup(LocalToWorld)
                {
                    localToWorldMatrix = LocalToWorldFromEntity[entity].Value;
                }

                if (ChildFromEntity.HasComponent(entity))
                {
                    var children = ChildFromEntity[entity];
                    for (int i = 0; i < children.Length; i++)
                    {
                        ChildLocalToWorld(localToWorldMatrix, children[i].Value, updateChildrenTransform);
                    }
                }
            }

            public void Execute(ArchetypeChunk chunk, int index, int entityOffset)
            {
                bool updateChildrenTransform =
                    chunk.DidChange<LocalToWorld>(LocalToWorldTypeHandle, LastSystemVersion) ||
                    chunk.DidChange<Child>(ChildTypeHandle, LastSystemVersion);

                var chunkLocalToWorld = chunk.GetNativeArray(LocalToWorldTypeHandle);
                var chunkChildren = chunk.GetBufferAccessor(ChildTypeHandle);
                for (int i = 0; i < chunk.Count; i++)
                {
                    var localToWorldMatrix = chunkLocalToWorld[i].Value;
                    var children = chunkChildren[i];
                    for (int j = 0; j < children.Length; j++)
                    {
                        ChildLocalToWorld(localToWorldMatrix, children[j].Value, updateChildrenTransform);
                    }
                }
            }
        }

        protected override void OnCreate()
        {
            m_RootsGroup = GetEntityQuery(new EntityQueryDesc
            {
                All = new ComponentType[]
                {
                    ComponentType.ReadOnly<LocalToWorld>(),
                    ComponentType.ReadOnly<Child>()
                },
                None = new ComponentType[]
                {
                    typeof(Parent)
                },
                Options = EntityQueryOptions.FilterWriteGroup
            });

            m_LocalToWorldWriteGroupMask = EntityManager.GetEntityQueryMask(GetEntityQuery(new EntityQueryDesc
            {
                All = new ComponentType[]
                {
                    typeof(LocalToWorld),
                    ComponentType.ReadOnly<LocalToParent>(),
                    ComponentType.ReadOnly<Parent>()
                },
                Options = EntityQueryOptions.FilterWriteGroup
            }));
        }

        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            var localToWorldType = GetComponentTypeHandle<LocalToWorld>(true);
            var childType = GetBufferTypeHandle<Child>(true);
            var childFromEntity = GetBufferFromEntity<Child>(true);
            var localToParentFromEntity = GetComponentDataFromEntity<LocalToParent>(true);
            var localToWorldFromEntity = GetComponentDataFromEntity<LocalToWorld>();

            var updateHierarchyJob = new UpdateHierarchy
            {
                LocalToWorldTypeHandle = localToWorldType,
                ChildTypeHandle = childType,
                ChildFromEntity = childFromEntity,
                LocalToParentFromEntity = localToParentFromEntity,
                LocalToWorldFromEntity = localToWorldFromEntity,
                LocalToWorldWriteGroupMask = m_LocalToWorldWriteGroupMask,
                LastSystemVersion = LastSystemVersion
            };
            var updateHierarchyJobHandle = updateHierarchyJob.Schedule(m_RootsGroup, inputDeps);
            return updateHierarchyJobHandle;
        }
    }
}
