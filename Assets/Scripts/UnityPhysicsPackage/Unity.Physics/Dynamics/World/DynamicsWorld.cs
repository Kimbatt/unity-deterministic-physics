using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;

namespace UnityS.Physics
{
    // A collection of motion information used during physics simulation.
    [NoAlias]
    public struct DynamicsWorld : IDisposable
    {
        [NoAlias]
        private NativeArray<MotionData> m_MotionDatas;
        [NoAlias]
        private NativeArray<MotionVelocity> m_MotionVelocities;
        private int m_NumMotions; // number of motionDatas and motionVelocities currently in use

        [NoAlias]
        private NativeArray<Joint> m_Joints;
        private int m_NumJoints; // number of joints currently in use
        [NoAlias] internal NativeHashMap<Entity, int> EntityJointIndexMap;

        public NativeArray<MotionData> MotionDatas => m_MotionDatas.GetSubArray(0, m_NumMotions);
        public NativeArray<MotionVelocity> MotionVelocities => m_MotionVelocities.GetSubArray(0, m_NumMotions);
        public NativeArray<Joint> Joints => m_Joints.GetSubArray(0, m_NumJoints);

        public int NumMotions => m_NumMotions;

        public int NumJoints => m_NumJoints;

        // Construct a dynamics world with the given number of uninitialized motions
        public DynamicsWorld(int numMotions, int numJoints)
        {
            m_MotionDatas = new NativeArray<MotionData>(numMotions, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            m_MotionVelocities = new NativeArray<MotionVelocity>(numMotions, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            m_NumMotions = numMotions;

            m_Joints = new NativeArray<Joint>(numJoints, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            m_NumJoints = numJoints;

            EntityJointIndexMap = new NativeHashMap<Entity, int>(numJoints, Allocator.Persistent);
        }

        public void Reset(int numMotions, int numJoints)
        {
            m_NumMotions = numMotions;
            if (m_MotionDatas.Length < m_NumMotions)
            {
                m_MotionDatas.Dispose();
                m_MotionDatas = new NativeArray<MotionData>(m_NumMotions, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            }
            if (m_MotionVelocities.Length < m_NumMotions)
            {
                m_MotionVelocities.Dispose();
                m_MotionVelocities = new NativeArray<MotionVelocity>(m_NumMotions, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            }

            m_NumJoints = numJoints;
            if (m_Joints.Length < m_NumJoints)
            {
                m_Joints.Dispose();
                m_Joints = new NativeArray<Joint>(m_NumJoints, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                EntityJointIndexMap.Capacity = m_NumJoints;
            }
            EntityJointIndexMap.Clear();
        }

        // Free internal memory
        public void Dispose()
        {
            m_MotionDatas.Dispose();
            m_MotionVelocities.Dispose();
            m_Joints.Dispose();
            EntityJointIndexMap.Dispose();
        }

        // Clone the world
        public DynamicsWorld Clone()
        {
            DynamicsWorld clone = new DynamicsWorld
            {
                m_MotionDatas = new NativeArray<MotionData>(m_MotionDatas.Length, Allocator.Persistent, NativeArrayOptions.UninitializedMemory),
                m_MotionVelocities = new NativeArray<MotionVelocity>(m_MotionVelocities.Length, Allocator.Persistent, NativeArrayOptions.UninitializedMemory),
                m_NumMotions = m_NumMotions,
                m_Joints = new NativeArray<Joint>(m_Joints.Length, Allocator.Persistent, NativeArrayOptions.UninitializedMemory),
                m_NumJoints = m_NumJoints,
                EntityJointIndexMap = new NativeHashMap<Entity, int>(m_Joints.Length, Allocator.Persistent),
            };
            clone.m_MotionDatas.CopyFrom(m_MotionDatas);
            clone.m_MotionVelocities.CopyFrom(m_MotionVelocities);
            clone.m_Joints.CopyFrom(m_Joints);
            clone.UpdateJointIndexMap();
            return clone;
        }

        public void UpdateJointIndexMap()
        {
            EntityJointIndexMap.Clear();
            for (int i = 0; i < m_Joints.Length; i++)
            {
                EntityJointIndexMap.TryAdd(m_Joints[i].Entity, i);
            }
        }

        public int GetJointIndex(Entity entity)
        {
            return EntityJointIndexMap.TryGetValue(entity, out var index) ? index : -1;
        }
    }
}
