using System;
using Unity.Entities;

namespace UnityS.Transforms
{
    [Serializable]
    [WriteGroup(typeof(LocalToWorld))]
    public struct Parent : IComponentData
    {
        public Entity Value;
    }

    [Serializable]
    public struct PreviousParent : ICleanupComponentData
    {
        public Entity Value;
    }

    [Serializable]
    [InternalBufferCapacity(8)]
    [WriteGroup(typeof(ParentScaleInverse))]
    public struct Child : ICleanupBufferElementData
    {
        public Entity Value;
    }
}
