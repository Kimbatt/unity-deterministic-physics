#if UNITY_DATAFLOWGRAPH_EXISTS
using Unity.DataFlowGraph;
using Unity.Burst;

namespace Unity.Physics
{
    /// <summary>
    /// DataFlowGraph node that performs a Ray Cast query on a CollisionWorld.
    /// </summary>
    public class RaycastNode :
#if UNITY_DATAFLOWGRAPH_0_17_OR_NEWER
        KernelNodeDefinition<RaycastNode.KernelDefs>
#else
        NodeDefinition<RaycastNode.Data, RaycastNode.SimPorts, RaycastNode.KernelData, RaycastNode.KernelDefs, RaycastNode.Kernel>
#endif
    {
        public struct SimPorts : ISimulationPortDefinition
        {
        }

        public struct KernelDefs : IKernelPortDefinition
        {
            public DataInput<RaycastNode, RaycastInput> Input;
            public DataInput<RaycastNode, CollisionWorldProxy> CollisionWorld;

            public DataOutput<RaycastNode, RaycastHit> Hit;
            public DataOutput<RaycastNode, bool> HitSuccess;
        }

        public struct Data : INodeData
        {
        }

        public struct KernelData : IKernelData
        {
        }

        [BurstCompile]
        public struct Kernel : IGraphKernel<KernelData, KernelDefs>
        {
            public void Execute(RenderContext ctx, KernelData data, ref KernelDefs ports)
            {
                var collisionWorldProxy = ctx.Resolve(ports.CollisionWorld);
                if (!collisionWorldProxy.IsCreated)
                {
                    ctx.Resolve(ref ports.HitSuccess) = false;
                    return;
                }

                var collisionWorld = collisionWorldProxy.ToCollisionWorld();

                if (collisionWorld.NumBodies > 0)
                {
                    ctx.Resolve(ref ports.HitSuccess) = collisionWorld.CastRay(ctx.Resolve(ports.Input), out RaycastHit hit);
                    ctx.Resolve(ref ports.Hit) = hit;
                }
                else
                {
                    ctx.Resolve(ref ports.HitSuccess) = false;
                }
            }
        }
    }
}
#endif
