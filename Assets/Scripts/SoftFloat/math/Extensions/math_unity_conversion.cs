#if !UNITY_DOTSPLAYER
using UnityEngine;

#pragma warning disable 0660, 0661

namespace UnityS.Mathematics
{
    public partial struct float2
    {
        public static explicit operator Vector2(float2 v)     { return new Vector2((float)v.x, (float)v.y); }
        public static explicit operator float2(Vector2 v)     { return new float2((sfloat)v.x, (sfloat)v.y); }
    }

    public partial struct float3
    {
        public static explicit operator Vector3(float3 v)     { return new Vector3((float)v.x, (float)v.y, (float)v.z); }
        public static explicit operator float3(Vector3 v)     { return new float3((sfloat)v.x, (sfloat)v.y, (sfloat)v.z); }
    }

    public partial struct float4
    {
        public static explicit operator float4(Vector4 v)     { return new float4((sfloat)v.x, (sfloat)v.y, (sfloat)v.z, (sfloat)v.w); }
        public static explicit operator Vector4(float4 v)     { return new Vector4((float)v.x, (float)v.y, (float)v.z, (float)v.w); }
    }

    public partial struct quaternion
    {
        public static explicit operator Quaternion(quaternion q)  { return new Quaternion((float)q.value.x, (float)q.value.y, (float)q.value.z, (float)q.value.w); }
        public static explicit operator quaternion(Quaternion q)  { return new quaternion((sfloat)q.x, (sfloat)q.y, (sfloat)q.z, (sfloat)q.w); }
    }

    public partial struct float4x4
    {
        public static explicit operator float4x4(Matrix4x4 m) { return new float4x4((float4)m.GetColumn(0), (float4)m.GetColumn(1), (float4)m.GetColumn(2), (float4)m.GetColumn(3)); }
        public static explicit operator Matrix4x4(float4x4 m) { return new Matrix4x4((Vector4)m.c0, (Vector4)m.c1, (Vector4)m.c2, (Vector4)m.c3); }
    }
}
#endif
