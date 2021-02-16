#if UNITY_DATAFLOWGRAPH_EXISTS
using Unity.Burst;
using Unity.DataFlowGraph;

namespace Unity.Physics
{
    /// <summary>
    /// DataFlowGraph node that performs a Collider Cast query on a CollisionWorld.
    /// </summary>
    public class ColliderCastNode :
#if UNITY_DATAFLOWGRAPH_0_17_OR_NEWER
        KernelNodeDefinition<ColliderCastNode.KernelDefs>
#else
        NodeDefinition<ColliderCastNode.Data, ColliderCastNode.SimPorts, ColliderCastNode.KernelData, ColliderCastNode.KernelDefs, ColliderCastNode.Kernel>
#endif
    {
        public struct SimPorts : ISimulationPortDefinition
        {
        }

        public struct KernelDefs : IKernelPortDefinition
        {
            public DataInput<ColliderCastNode, ColliderCastInput> Input;
            public DataInput<ColliderCastNode, CollisionWorldProxy> CollisionWorld;

            public DataOutput<ColliderCastNode, ColliderCastHit> Hit;
            public DataOutput<ColliderCastNode, bool> HitSuccess;
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
                    ctx.Resolve(ref ports.HitSuccess) = collisionWorld.CastCollider(ctx.Resolve(ports.Input), out ColliderCastHit hit);
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
