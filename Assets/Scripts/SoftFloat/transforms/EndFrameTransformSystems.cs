using Unity.Entities;

namespace UnityS.Transforms
{
    [UnityEngine.ExecuteAlways]
    public partial class TransformSystemGroup : ComponentSystemGroup
    {
    }

    [UnityEngine.ExecuteAlways]
    [UpdateInGroup(typeof(TransformSystemGroup))]
    public partial class EndFrameParentSystem : ParentSystem
    {
    }

    [UnityEngine.ExecuteAlways]
    [UpdateInGroup(typeof(TransformSystemGroup))]
    public partial class EndFrameCompositeScaleSystem : CompositeScaleSystem
    {
    }

    [UnityEngine.ExecuteAlways]
    [UpdateInGroup(typeof(TransformSystemGroup))]
    public partial class EndFrameRotationEulerSystem : RotationEulerSystem
    {
    }

    [UnityEngine.ExecuteAlways]
    [UpdateInGroup(typeof(TransformSystemGroup))]
    public partial class EndFramePostRotationEulerSystem : PostRotationEulerSystem
    {
    }

    [UnityEngine.ExecuteAlways]
    [UpdateInGroup(typeof(TransformSystemGroup))]
    [UpdateAfter(typeof(EndFrameRotationEulerSystem))]
    public partial class EndFrameCompositeRotationSystem : CompositeRotationSystem
    {
    }

    [UnityEngine.ExecuteAlways]
    [UpdateInGroup(typeof(TransformSystemGroup))]
    [UpdateAfter(typeof(EndFrameCompositeRotationSystem))]
    [UpdateAfter(typeof(EndFrameCompositeScaleSystem))]
    [UpdateBefore(typeof(EndFrameLocalToParentSystem))]
    public partial class EndFrameTRSToLocalToWorldSystem : TRSToLocalToWorldSystem
    {
    }

    [UnityEngine.ExecuteAlways]
    [UpdateInGroup(typeof(TransformSystemGroup))]
    [UpdateAfter(typeof(EndFrameParentSystem))]
    [UpdateAfter(typeof(EndFrameCompositeRotationSystem))]
    public partial class EndFrameParentScaleInverseSystem : ParentScaleInverseSystem
    {
    }

    [UnityEngine.ExecuteAlways]
    [UpdateInGroup(typeof(TransformSystemGroup))]
    [UpdateAfter(typeof(EndFrameCompositeRotationSystem))]
    [UpdateAfter(typeof(EndFrameCompositeScaleSystem))]
    [UpdateAfter(typeof(EndFrameParentScaleInverseSystem))]
    public partial class EndFrameTRSToLocalToParentSystem : TRSToLocalToParentSystem
    {
    }

    [UnityEngine.ExecuteAlways]
    [UpdateInGroup(typeof(TransformSystemGroup))]
    [UpdateAfter(typeof(EndFrameTRSToLocalToParentSystem))]
    public partial class EndFrameLocalToParentSystem : LocalToParentSystem
    {
    }

    [UnityEngine.ExecuteAlways]
    [UpdateInGroup(typeof(TransformSystemGroup))]
    [UpdateAfter(typeof(EndFrameTRSToLocalToWorldSystem))]
    [UpdateAfter(typeof(EndFrameLocalToParentSystem))]
    public partial class EndFrameWorldToLocalSystem : WorldToLocalSystem
    {
    }
}
