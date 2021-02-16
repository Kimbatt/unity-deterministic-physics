using System;
using Unity.Entities;
using UnityS.Mathematics;

namespace UnityS.Transforms
{
    [Serializable]
    [WriteGroup(typeof(LocalToWorld))]
    [WriteGroup(typeof(LocalToParent))]
    [WriteGroup(typeof(CompositeRotation))]
    public struct Rotation : IComponentData
    {
        public quaternion Value;
    }
}
