using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using UnityS.Mathematics;

/* **************
   COPY AND PASTE
   **************
 * PostRotationEuler.cs and RotationEuler.cs are copy-and-paste.
 * Any changes to one must be copied to the other.
 * The only differences are:
 *   s/PostRotation/Rotation/g
*/

namespace UnityS.Transforms
{
    [Serializable]
    [WriteGroup(typeof(PostRotation))]
    public struct PostRotationEulerXYZ : IComponentData
    {
        public float3 Value;
    }

    [Serializable]
    [WriteGroup(typeof(PostRotation))]
    public struct PostRotationEulerXZY : IComponentData
    {
        public float3 Value;
    }

    [Serializable]
    [WriteGroup(typeof(PostRotation))]
    public struct PostRotationEulerYXZ : IComponentData
    {
        public float3 Value;
    }

    [Serializable]
    [WriteGroup(typeof(PostRotation))]
    public struct PostRotationEulerYZX : IComponentData
    {
        public float3 Value;
    }

    [Serializable]
    [WriteGroup(typeof(PostRotation))]
    public struct PostRotationEulerZXY : IComponentData
    {
        public float3 Value;
    }

    [Serializable]
    [WriteGroup(typeof(PostRotation))]
    public struct PostRotationEulerZYX : IComponentData
    {
        public float3 Value;
    }

    // PostRotation = PostRotationEulerXYZ
    // (or) PostRotation = PostRotationEulerXZY
    // (or) PostRotation = PostRotationEulerYXZ
    // (or) PostRotation = PostRotationEulerYZX
    // (or) PostRotation = PostRotationEulerZXY
    // (or) PostRotation = PostRotationEulerZYX
    public abstract class PostRotationEulerSystem : JobComponentSystem
    {
        private EntityQuery m_Group;

        protected override void OnCreate()
        {
            m_Group = GetEntityQuery(new EntityQueryDesc
            {
                All = new ComponentType[]
                {
                    typeof(PostRotation)
                },
                Any = new ComponentType[]
                {
                    ComponentType.ReadOnly<PostRotationEulerXYZ>(),
                    ComponentType.ReadOnly<PostRotationEulerXZY>(),
                    ComponentType.ReadOnly<PostRotationEulerYXZ>(),
                    ComponentType.ReadOnly<PostRotationEulerYZX>(),
                    ComponentType.ReadOnly<PostRotationEulerZXY>(),
                    ComponentType.ReadOnly<PostRotationEulerZYX>()
                },
                Options = EntityQueryOptions.FilterWriteGroup
            });
        }

        [BurstCompile]
        struct PostRotationEulerToPostRotation : IJobChunk
        {
            public ComponentTypeHandle<PostRotation> PostRotationTypeHandle;
            [ReadOnly] public ComponentTypeHandle<PostRotationEulerXYZ> PostRotationEulerXyzTypeHandle;
            [ReadOnly] public ComponentTypeHandle<PostRotationEulerXZY> PostRotationEulerXzyTypeHandle;
            [ReadOnly] public ComponentTypeHandle<PostRotationEulerYXZ> PostRotationEulerYxzTypeHandle;
            [ReadOnly] public ComponentTypeHandle<PostRotationEulerYZX> PostRotationEulerYzxTypeHandle;
            [ReadOnly] public ComponentTypeHandle<PostRotationEulerZXY> PostRotationEulerZxyTypeHandle;
            [ReadOnly] public ComponentTypeHandle<PostRotationEulerZYX> PostRotationEulerZyxTypeHandle;
            public uint LastSystemVersion;

            public void Execute(ArchetypeChunk chunk, int chunkIndex, int firstEntityIndex)
            {
                if (chunk.Has(PostRotationEulerXyzTypeHandle))
                {
                    if (!chunk.DidChange(PostRotationEulerXyzTypeHandle, LastSystemVersion))
                        return;

                    var chunkRotations = chunk.GetNativeArray(PostRotationTypeHandle);
                    var chunkPostRotationEulerXYZs = chunk.GetNativeArray(PostRotationEulerXyzTypeHandle);
                    for (var i = 0; i < chunk.Count; i++)
                    {
                        chunkRotations[i] = new PostRotation
                        {
                            Value = quaternion.EulerXYZ(chunkPostRotationEulerXYZs[i].Value)
                        };
                    }
                }
                else if (chunk.Has(PostRotationEulerXzyTypeHandle))
                {
                    if (!chunk.DidChange(PostRotationEulerXzyTypeHandle, LastSystemVersion))
                        return;

                    var chunkRotations = chunk.GetNativeArray(PostRotationTypeHandle);
                    var chunkPostRotationEulerXZYs = chunk.GetNativeArray(PostRotationEulerXzyTypeHandle);
                    for (var i = 0; i < chunk.Count; i++)
                    {
                        chunkRotations[i] = new PostRotation
                        {
                            Value = quaternion.EulerXZY(chunkPostRotationEulerXZYs[i].Value)
                        };
                    }
                }
                else if (chunk.Has(PostRotationEulerYxzTypeHandle))
                {
                    if (!chunk.DidChange(PostRotationEulerYxzTypeHandle, LastSystemVersion))
                        return;

                    var chunkRotations = chunk.GetNativeArray(PostRotationTypeHandle);
                    var chunkPostRotationEulerYXZs = chunk.GetNativeArray(PostRotationEulerYxzTypeHandle);
                    for (var i = 0; i < chunk.Count; i++)
                    {
                        chunkRotations[i] = new PostRotation
                        {
                            Value = quaternion.EulerYXZ(chunkPostRotationEulerYXZs[i].Value)
                        };
                    }
                }
                else if (chunk.Has(PostRotationEulerYzxTypeHandle))
                {
                    if (!chunk.DidChange(PostRotationEulerYzxTypeHandle, LastSystemVersion))
                        return;

                    var chunkRotations = chunk.GetNativeArray(PostRotationTypeHandle);
                    var chunkPostRotationEulerYZXs = chunk.GetNativeArray(PostRotationEulerYzxTypeHandle);
                    for (var i = 0; i < chunk.Count; i++)
                    {
                        chunkRotations[i] = new PostRotation
                        {
                            Value = quaternion.EulerYZX(chunkPostRotationEulerYZXs[i].Value)
                        };
                    }
                }
                else if (chunk.Has(PostRotationEulerZxyTypeHandle))
                {
                    if (!chunk.DidChange(PostRotationEulerZxyTypeHandle, LastSystemVersion))
                        return;

                    var chunkRotations = chunk.GetNativeArray(PostRotationTypeHandle);
                    var chunkPostRotationEulerZXYs = chunk.GetNativeArray(PostRotationEulerZxyTypeHandle);
                    for (var i = 0; i < chunk.Count; i++)
                    {
                        chunkRotations[i] = new PostRotation
                        {
                            Value = quaternion.EulerZXY(chunkPostRotationEulerZXYs[i].Value)
                        };
                    }
                }
                else if (chunk.Has(PostRotationEulerZyxTypeHandle))
                {
                    if (!chunk.DidChange(PostRotationEulerZyxTypeHandle, LastSystemVersion))
                        return;

                    var chunkRotations = chunk.GetNativeArray(PostRotationTypeHandle);
                    var chunkPostRotationEulerZYXs = chunk.GetNativeArray(PostRotationEulerZyxTypeHandle);
                    for (var i = 0; i < chunk.Count; i++)
                    {
                        chunkRotations[i] = new PostRotation
                        {
                            Value = quaternion.EulerZYX(chunkPostRotationEulerZYXs[i].Value)
                        };
                    }
                }
            }
        }

        protected override JobHandle OnUpdate(JobHandle inputDependencies)
        {
            var job = new PostRotationEulerToPostRotation()
            {
                PostRotationTypeHandle = GetComponentTypeHandle<PostRotation>(false),
                PostRotationEulerXyzTypeHandle = GetComponentTypeHandle<PostRotationEulerXYZ>(true),
                PostRotationEulerXzyTypeHandle = GetComponentTypeHandle<PostRotationEulerXZY>(true),
                PostRotationEulerYxzTypeHandle = GetComponentTypeHandle<PostRotationEulerYXZ>(true),
                PostRotationEulerYzxTypeHandle = GetComponentTypeHandle<PostRotationEulerYZX>(true),
                PostRotationEulerZxyTypeHandle = GetComponentTypeHandle<PostRotationEulerZXY>(true),
                PostRotationEulerZyxTypeHandle = GetComponentTypeHandle<PostRotationEulerZYX>(true),
                LastSystemVersion = LastSystemVersion
            };
            return job.Schedule(m_Group, inputDependencies);
        }
    }
}
