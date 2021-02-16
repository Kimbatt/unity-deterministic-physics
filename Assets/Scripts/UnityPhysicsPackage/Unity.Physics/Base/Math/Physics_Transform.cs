using System;
using System.Diagnostics;
using UnityS.Mathematics;

namespace UnityS.Physics
{
    public static partial class Math
    {
        [DebuggerStepThrough]
        public struct MTransform: IEquatable<MTransform> // TODO: Replace with Unity float4x4 ?
        {
            public float3x3 Rotation;
            public float3 Translation;

            public static MTransform Identity => new MTransform { Rotation = float3x3.identity, Translation = float3.zero };
            
            public MTransform(RigidTransform transform)
            {
                Rotation = new float3x3(transform.rot);
                Translation = transform.pos;
            }

            public MTransform(quaternion rotation, float3 translation)
            {
                Rotation = new float3x3(rotation);
                Translation = translation;
            }

            public MTransform(float3x3 rotation, float3 translation)
            {
                Rotation = rotation;
                Translation = translation;
            }

            public float3x3 InverseRotation => math.transpose(Rotation);

            public bool Equals(MTransform other)
            {
                return this.Rotation.Equals(other.Rotation) && this.Translation.Equals(other.Translation);
            }
        }

        public static float3 Mul(MTransform a, float3 x)
        {
            return math.mul(a.Rotation, x) + a.Translation;
        }

        // Returns cFromA = cFromB * bFromA
        public static MTransform Mul(MTransform cFromB, MTransform bFromA)
        {
            return new MTransform
            {
                Rotation = math.mul(cFromB.Rotation, bFromA.Rotation),
                Translation = math.mul(cFromB.Rotation, bFromA.Translation) + cFromB.Translation
            };
        }

        public static MTransform Inverse(MTransform a)
        {
            float3x3 inverseRotation = math.transpose(a.Rotation);
            return new MTransform
            {
                Rotation = inverseRotation,
                Translation = math.mul(inverseRotation, -a.Translation)
            };
        }
    }
}
