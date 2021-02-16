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
    [WriteGroup(typeof(Rotation))]
    public struct RotationEulerXYZ : IComponentData
    {
        public float3 Value;
    }

    [Serializable]
    [WriteGroup(typeof(Rotation))]
    public struct RotationEulerXZY : IComponentData
    {
        public float3 Value;
    }

    [Serializable]
    [WriteGroup(typeof(Rotation))]
    public struct RotationEulerYXZ : IComponentData
    {
        public float3 Value;
    }

    [Serializable]
    [WriteGroup(typeof(Rotation))]
    public struct RotationEulerYZX : IComponentData
    {
        public float3 Value;
    }

    [Serializable]
    [WriteGroup(typeof(Rotation))]
    public struct RotationEulerZXY : IComponentData
    {
        public float3 Value;
    }

    [Serializable]
    [WriteGroup(typeof(Rotation))]
    public struct RotationEulerZYX : IComponentData
    {
        public float3 Value;
    }

    // Rotation = RotationEulerXYZ
    // (or) Rotation = RotationEulerXZY
    // (or) Rotation = RotationEulerYXZ
    // (or) Rotation = RotationEulerYZX
    // (or) Rotation = RotationEulerZXY
    // (or) Rotation = RotationEulerZYX
    public abstract class RotationEulerSystem : JobComponentSystem
    {
        private EntityQuery m_Group;

        protected override void OnCreate()
        {
            m_Group = GetEntityQuery(new EntityQueryDesc
            {
                All = new ComponentType[]
                {
                    typeof(Rotation)
                },
                Any = new ComponentType[]
                {
                    ComponentType.ReadOnly<RotationEulerXYZ>(),
                    ComponentType.ReadOnly<RotationEulerXZY>(),
                    ComponentType.ReadOnly<RotationEulerYXZ>(),
                    ComponentType.ReadOnly<RotationEulerYZX>(),
                    ComponentType.ReadOnly<RotationEulerZXY>(),
                    ComponentType.ReadOnly<RotationEulerZYX>()
                },
                Options = EntityQueryOptions.FilterWriteGroup
            });
        }

        [BurstCompile]
        struct RotationEulerToRotation : IJobChunk
        {
            public ComponentTypeHandle<Rotation> RotationTypeHandle;
            [ReadOnly] public ComponentTypeHandle<RotationEulerXYZ> RotationEulerXyzTypeHandle;
            [ReadOnly] public ComponentTypeHandle<RotationEulerXZY> RotationEulerXzyTypeHandle;
            [ReadOnly] public ComponentTypeHandle<RotationEulerYXZ> RotationEulerYxzTypeHandle;
            [ReadOnly] public ComponentTypeHandle<RotationEulerYZX> RotationEulerYzxTypeHandle;
            [ReadOnly] public ComponentTypeHandle<RotationEulerZXY> RotationEulerZxyTypeHandle;
            [ReadOnly] public ComponentTypeHandle<RotationEulerZYX> RotationEulerZyxTypeHandle;
            public uint LastSystemVersion;

            public void Execute(ArchetypeChunk chunk, int chunkIndex, int firstEntityIndex)
            {
                if (chunk.Has(RotationEulerXyzTypeHandle))
                {
                    if (!chunk.DidChange(RotationEulerXyzTypeHandle, LastSystemVersion))
                        return;

                    var chunkRotations = chunk.GetNativeArray(RotationTypeHandle);
                    var chunkRotationEulerXYZs = chunk.GetNativeArray(RotationEulerXyzTypeHandle);
                    for (var i = 0; i < chunk.Count; i++)
                    {
                        chunkRotations[i] = new Rotation
                        {
                            Value = quaternion.EulerXYZ(chunkRotationEulerXYZs[i].Value)
                        };
                    }
                }
                else if (chunk.Has(RotationEulerXzyTypeHandle))
                {
                    if (!chunk.DidChange(RotationEulerXzyTypeHandle, LastSystemVersion))
                        return;

                    var chunkRotations = chunk.GetNativeArray(RotationTypeHandle);
                    var chunkRotationEulerXZYs = chunk.GetNativeArray(RotationEulerXzyTypeHandle);
                    for (var i = 0; i < chunk.Count; i++)
                    {
                        chunkRotations[i] = new Rotation
                        {
                            Value = quaternion.EulerXZY(chunkRotationEulerXZYs[i].Value)
                        };
                    }
                }
                else if (chunk.Has(RotationEulerYxzTypeHandle))
                {
                    if (!chunk.DidChange(RotationEulerYxzTypeHandle, LastSystemVersion))
                        return;

                    var chunkRotations = chunk.GetNativeArray(RotationTypeHandle);
                    var chunkRotationEulerYXZs = chunk.GetNativeArray(RotationEulerYxzTypeHandle);
                    for (var i = 0; i < chunk.Count; i++)
                    {
                        chunkRotations[i] = new Rotation
                        {
                            Value = quaternion.EulerYXZ(chunkRotationEulerYXZs[i].Value)
                        };
                    }
                }
                else if (chunk.Has(RotationEulerYzxTypeHandle))
                {
                    if (!chunk.DidChange(RotationEulerYzxTypeHandle, LastSystemVersion))
                        return;

                    var chunkRotations = chunk.GetNativeArray(RotationTypeHandle);
                    var chunkRotationEulerYZXs = chunk.GetNativeArray(RotationEulerYzxTypeHandle);
                    for (var i = 0; i < chunk.Count; i++)
                    {
                        chunkRotations[i] = new Rotation
                        {
                            Value = quaternion.EulerYZX(chunkRotationEulerYZXs[i].Value)
                        };
                    }
                }
                else if (chunk.Has(RotationEulerZxyTypeHandle))
                {
                    if (!chunk.DidChange(RotationEulerZxyTypeHandle, LastSystemVersion))
                        return;

                    var chunkRotations = chunk.GetNativeArray(RotationTypeHandle);
                    var chunkRotationEulerZXYs = chunk.GetNativeArray(RotationEulerZxyTypeHandle);
                    for (var i = 0; i < chunk.Count; i++)
                    {
                        chunkRotations[i] = new Rotation
                        {
                            Value = quaternion.EulerZXY(chunkRotationEulerZXYs[i].Value)
                        };
                    }
                }
                else if (chunk.Has(RotationEulerZyxTypeHandle))
                {
                    if (!chunk.DidChange(RotationEulerZyxTypeHandle, LastSystemVersion))
                        return;

                    var chunkRotations = chunk.GetNativeArray(RotationTypeHandle);
                    var chunkRotationEulerZYXs = chunk.GetNativeArray(RotationEulerZyxTypeHandle);
                    for (var i = 0; i < chunk.Count; i++)
                    {
                        chunkRotations[i] = new Rotation
                        {
                            Value = quaternion.EulerZYX(chunkRotationEulerZYXs[i].Value)
                        };
                    }
                }
            }
        }

        protected override JobHandle OnUpdate(JobHandle inputDependencies)
        {
            var job = new RotationEulerToRotation()
            {
                RotationTypeHandle = GetComponentTypeHandle<Rotation>(false),
                RotationEulerXyzTypeHandle = GetComponentTypeHandle<RotationEulerXYZ>(true),
                RotationEulerXzyTypeHandle = GetComponentTypeHandle<RotationEulerXZY>(true),
                RotationEulerYxzTypeHandle = GetComponentTypeHandle<RotationEulerYXZ>(true),
                RotationEulerYzxTypeHandle = GetComponentTypeHandle<RotationEulerYZX>(true),
                RotationEulerZxyTypeHandle = GetComponentTypeHandle<RotationEulerZXY>(true),
                RotationEulerZyxTypeHandle = GetComponentTypeHandle<RotationEulerZYX>(true),
                LastSystemVersion = LastSystemVersion
            };
            return job.Schedule(m_Group, inputDependencies);
        }
    }
}
