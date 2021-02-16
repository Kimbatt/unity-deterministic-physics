using System;
using UnityS.Mathematics;

namespace UnityS.Physics
{
    public static partial class Math
    {
        // 4 transposed 3D vertices
        [Serializable]
        public struct FourTransposedPoints
        {
            private float4x3 m_TransposedPoints;

            public float4 X => m_TransposedPoints.c0;
            public float4 Y => m_TransposedPoints.c1;
            public float4 Z => m_TransposedPoints.c2;

            public FourTransposedPoints V0000 => new FourTransposedPoints { m_TransposedPoints = new float4x3(X.xxxx, Y.xxxx, Z.xxxx) };
            public FourTransposedPoints V1111 => new FourTransposedPoints { m_TransposedPoints = new float4x3(X.yyyy, Y.yyyy, Z.yyyy) };
            public FourTransposedPoints V2222 => new FourTransposedPoints { m_TransposedPoints = new float4x3(X.zzzz, Y.zzzz, Z.zzzz) };
            public FourTransposedPoints V3333 => new FourTransposedPoints { m_TransposedPoints = new float4x3(X.wwww, Y.wwww, Z.wwww) };
            public FourTransposedPoints V1230 => new FourTransposedPoints { m_TransposedPoints = new float4x3(X.yzwx, Y.yzwx, Z.yzwx) };
            public FourTransposedPoints V3012 => new FourTransposedPoints { m_TransposedPoints = new float4x3(X.wxyz, Y.wxyz, Z.wxyz) };
            public FourTransposedPoints V1203 => new FourTransposedPoints { m_TransposedPoints = new float4x3(X.yzxw, Y.yzxw, Z.yzxw) };

            public FourTransposedPoints(float3 v)
            {
                m_TransposedPoints.c0 = v.xxxx;
                m_TransposedPoints.c1 = v.yyyy;
                m_TransposedPoints.c2 = v.zzzz;
            }

            public FourTransposedPoints(float3 v0, float3 v1, float3 v2, float3 v3)
            {
                m_TransposedPoints.c0 = new float4(v0.x, v1.x, v2.x, v3.x);
                m_TransposedPoints.c1 = new float4(v0.y, v1.y, v2.y, v3.y);
                m_TransposedPoints.c2 = new float4(v0.z, v1.z, v2.z, v3.z);
            }

            public static FourTransposedPoints operator +(FourTransposedPoints lhs, FourTransposedPoints rhs)
            {
                return new FourTransposedPoints { m_TransposedPoints = lhs.m_TransposedPoints + rhs.m_TransposedPoints };
            }

            public static FourTransposedPoints operator -(FourTransposedPoints lhs, FourTransposedPoints rhs)
            {
                return new FourTransposedPoints { m_TransposedPoints = lhs.m_TransposedPoints - rhs.m_TransposedPoints };
            }

            public FourTransposedPoints MulT(float4 v)
            {
                return new FourTransposedPoints { m_TransposedPoints = new float4x3(X * v, Y * v, Z * v) };
            }

            public FourTransposedPoints Cross(FourTransposedPoints a)
            {
                return new FourTransposedPoints { m_TransposedPoints = new float4x3(Y * a.Z - Z * a.Y, Z * a.X - X * a.Z, X * a.Y - Y * a.X) };
            }

            public float4 Dot(float3 v) => X * v.x + Y * v.y + Z * v.z;
            public float4 Dot(FourTransposedPoints a) => X * a.X + Y * a.Y + Z * a.Z;

            public float3 GetPoint(int index) => new float3(X[index], Y[index], Z[index]);
            public float4 GetComponent(int index) => m_TransposedPoints[index];
        }
    }
}
