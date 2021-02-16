using Unity.Jobs;

namespace UnityS.Physics.Systems
{
    // Defines an interface for all internal physics systems
    internal interface IPhysicsSystem
    {
        void AddInputDependency(JobHandle inputDep);

        JobHandle GetOutputDependency();
    }
}
