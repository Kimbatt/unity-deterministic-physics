using System;
using System.Collections.Generic;
using Unity.Jobs;
using UnityEngine.Assertions;

namespace UnityS.Physics
{
    // A container of user callbacks, to run during scheduling of the simulation jobs
    public class SimulationCallbacks
    {
        // Callbacks that are invoked after each simulation phase (no callback needed after last phase)
        public enum Phase
        {
            PostCreateDispatchPairs,
            PostCreateContacts,
            PostCreateContactJacobians,
            PostSolveJacobians
        }

        // this needs to match the number of phase values above
        private static readonly int k_NumCallbacks = 4;

        public delegate JobHandle Callback(ref ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps);


        private struct CallbackAndDependency
        {
            public Callback Callback;
            public JobHandle Dependency;
        }

        private readonly List<CallbackAndDependency>[] m_Callbacks = new List<CallbackAndDependency>[k_NumCallbacks];

        public SimulationCallbacks()
        {
#if !NET_DOTS
            Assert.AreEqual(Enum.GetValues(typeof(Phase)).Length, k_NumCallbacks);
#endif

            for (int i = 0; i < k_NumCallbacks; ++i)
            {
                m_Callbacks[i] = new List<CallbackAndDependency>(8);
            }
        }

        public void Enqueue(Phase phase, Callback cb, JobHandle dependency)
        {
            m_Callbacks[(int)phase].Add(new CallbackAndDependency { Callback = cb, Dependency = dependency });
        }

        internal bool Any(Phase phase)
        {
            return m_Callbacks[(int)phase].Count > 0;
        }

        internal JobHandle Execute(Phase phase, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps)
        {
            ref List<CallbackAndDependency> cbs = ref m_Callbacks[(int)phase];
            if (m_Callbacks[(int)phase].Count > 0)
            {
                foreach (CallbackAndDependency callback in cbs)
                    inputDeps = callback.Callback(ref simulation, ref world, JobHandle.CombineDependencies(inputDeps, callback.Dependency));

                return inputDeps;
            }
            return inputDeps;
        }

        public void Clear()
        {
            for (int i = 0; i < k_NumCallbacks; i++)
            {
                m_Callbacks[i].Clear();
            }
        }
    }
}
