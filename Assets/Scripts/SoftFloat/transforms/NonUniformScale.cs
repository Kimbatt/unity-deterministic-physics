using System;
using Unity.Entities;
using UnityS.Mathematics;

namespace UnityS.Transforms
{
    [Serializable]
    [WriteGroup(typeof(LocalToWorld))]
    [WriteGroup(typeof(LocalToParent))]
    [WriteGroup(typeof(CompositeScale))]
    [WriteGroup(typeof(ParentScaleInverse))]
    public struct NonUniformScale : IComponentData
    {
        public float3 Value;
    }
}
