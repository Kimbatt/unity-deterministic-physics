using System;

namespace UnityS.Physics
{
    class PreserveAttribute : Attribute { }

    [Obsolete("Do not access this type. It is only included to hint AOT compilation. (DoNotRemove)", true)]
    static unsafe class AOTHint
    {
        [Preserve]
        static void HintAllImplementations()
        {
            AabbOverlapLeafProcessor_BoundingVolumeHierarchy_OverlapQueries_OverlapCollectors<Broadphase.BvhLeafProcessor, Broadphase.RigidBodyOverlapsCollector>();
            AabbOverlapLeafProcessor_BoundingVolumeHierarchy_OverlapQueries_OverlapCollectors<Broadphase.BvhLeafProcessor, ManifoldQueries.ConvexCompositeOverlapCollector>();
            AabbOverlapLeafProcessor_BoundingVolumeHierarchy_OverlapQueries_OverlapCollectors<OverlapQueries.CompoundLeafProcessor, Broadphase.RigidBodyOverlapsCollector>();
            AabbOverlapLeafProcessor_BoundingVolumeHierarchy_OverlapQueries_OverlapCollectors<OverlapQueries.CompoundLeafProcessor, ManifoldQueries.ConvexCompositeOverlapCollector>();
            AabbOverlapLeafProcessor_BoundingVolumeHierarchy_OverlapQueries_OverlapCollectors<OverlapQueries.MeshLeafProcessor, Broadphase.RigidBodyOverlapsCollector>();
            AabbOverlapLeafProcessor_BoundingVolumeHierarchy_OverlapQueries_OverlapCollectors<OverlapQueries.MeshLeafProcessor, ManifoldQueries.ConvexCompositeOverlapCollector>();

            ColliderCastLeafProcessor_ColliderCastHitCollectors<Broadphase.BvhLeafProcessor>();
            ColliderCastLeafProcessor_ColliderCastHitCollectors<ColliderCastQueries.ConvexCompoundLeafProcessor>();
            ColliderCastLeafProcessor_ColliderCastHitCollectors<ColliderCastQueries.ConvexMeshLeafProcessor>();

            ColliderDistanceLeafProcessor_DistanceCollectors<Broadphase.BvhLeafProcessor>();

            PointDistanceLeafProcessor_DistanceCollectors<Broadphase.BvhLeafProcessor>();

            RaycastLeafProcessor_RaycastHitCollectors<Broadphase.BvhLeafProcessor>();
        }

        static void AabbOverlapLeafProcessor_BoundingVolumeHierarchy_OverlapQueries_OverlapCollectors<TProcessor,
            TCollector>()
            where TProcessor : struct, BoundingVolumeHierarchy.IAabbOverlapLeafProcessor
            where TCollector : struct, IOverlapCollector
        {
            var collector = new TCollector();
            var p = new TProcessor();
            p.AabbLeaf(default, default, ref collector);
            var bvh = new BoundingVolumeHierarchy();
            bvh.AabbOverlap(default, ref p, ref collector, default);
            OverlapQueries.AabbCollider(default, null, ref collector);
        }

        static void ColliderCastLeafProcessor_ColliderCastHitCollectors<TProcessor>()
            where TProcessor : struct, BoundingVolumeHierarchy.IColliderCastLeafProcessor
        {
            var p = new TProcessor();
            var all = new AllHitsCollector<ColliderCastHit>();
            p.ColliderCastLeaf(default, default, ref all);
            var any = new AnyHitCollector<ColliderCastHit>();
            p.ColliderCastLeaf(default, default, ref any);
            var closest = new ClosestHitCollector<ColliderCastHit>();
            p.ColliderCastLeaf(default, default, ref closest);
        }

        static void ColliderDistanceLeafProcessor_DistanceCollectors<TProcessor>()
            where TProcessor : struct, BoundingVolumeHierarchy.IColliderDistanceLeafProcessor
        {
            var p = new TProcessor();
            var all = new AllHitsCollector<DistanceHit>();
            p.DistanceLeaf(default, default, ref all);
            var any = new AnyHitCollector<DistanceHit>();
            p.DistanceLeaf(default, default, ref any);
            var closest = new ClosestHitCollector<DistanceHit>();
            p.DistanceLeaf(default, default, ref closest);
        }

        static void PointDistanceLeafProcessor_DistanceCollectors<TProcessor>()
            where TProcessor : struct, BoundingVolumeHierarchy.IPointDistanceLeafProcessor
        {
            var p = new TProcessor();
            var all = new AllHitsCollector<DistanceHit>();
            p.DistanceLeaf(default, default, ref all);
            var any = new AnyHitCollector<DistanceHit>();
            p.DistanceLeaf(default, default, ref any);
            var closest = new ClosestHitCollector<DistanceHit>();
            p.DistanceLeaf(default, default, ref closest);
        }

        static void RaycastLeafProcessor_RaycastHitCollectors<TProcessor>()
            where TProcessor : struct, BoundingVolumeHierarchy.IRaycastLeafProcessor
        {
            var p = new TProcessor();
            var all = new AllHitsCollector<RaycastHit>();
            p.RayLeaf(default, default, ref all);
            var any = new AnyHitCollector<RaycastHit>();
            p.RayLeaf(default, default, ref any);
            var closest = new ClosestHitCollector<RaycastHit>();
            p.RayLeaf(default, default, ref closest);
        }
    }
}
