using System.Diagnostics;
using System.Runtime.CompilerServices;
using UnityS.Mathematics;
using UnityEngine.Assertions;

namespace UnityS.Physics
{
    // Common math helper functions
    [DebuggerStepThrough]
    public static partial class Math
    {
        // Constants
        [DebuggerStepThrough]
        public static class Constants
        {
            public static float4 One4F => new float4(1);
            public static float4 Min4F => new float4(sfloat.MinValue);
            public static float4 Max4F => new float4(sfloat.MaxValue);
            public static float3 Min3F => new float3(sfloat.MinValue);
            public static float3 Max3F => new float3(sfloat.MaxValue);

            // Smallest float such that 1.0 + eps != 1.0
            // Different from float.Epsilon which is the smallest value greater than zero.
            public static sfloat Eps => sfloat.FromRaw(0x34000000);

            // These constants are identical to the ones in the Unity Mathf library, to ensure identical behaviour
            internal static sfloat UnityEpsilonNormalSqrt => sfloat.FromRaw(0x26901d7d);
            internal static sfloat UnityEpsilon => sfloat.FromRaw(0x3727c5ac);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int NextMultipleOf16(int input) => ((input + 15) >> 4) << 4;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ulong NextMultipleOf16(ulong input) => ((input + 15) >> 4) << 4;

        /// Note that alignment must be a power of two for this to work.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int NextMultipleOf(int input, int alignment) => (input + (alignment - 1)) & (~(alignment - 1));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ulong NextMultipleOf(ulong input, ulong alignment) => (input + (alignment - 1)) & (~(alignment - 1));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int IndexOfMinComponent(float2 v) => v.x < v.y ? 0 : 1;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int IndexOfMinComponent(float3 v) => v.x < v.y ? ((v.x < v.z) ? 0 : 2) : ((v.y < v.z) ? 1 : 2);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int IndexOfMinComponent(float4 v) => math.cmax(math.select(new int4(0, 1, 2, 3), new int4(-1), math.cmin(v) < v));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int IndexOfMaxComponent(float2 v) => v.x > v.y ? 0 : 1;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int IndexOfMaxComponent(float3 v) => v.x > v.y ? ((v.x > v.z) ? 0 : 2) : ((v.y > v.z) ? 1 : 2);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int IndexOfMaxComponent(float4 v) => math.cmax(math.select(new int4(0, 1, 2, 3), new int4(-1), math.cmax(v) > v));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat HorizontalMul(float3 v) => v.x * v.y * v.z;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat HorizontalMul(float4 v) => (v.x * v.y) * (v.z * v.w);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat Dotxyz1(float4 lhs, float3 rhs) => math.dot(lhs, new float4(rhs, sfloat.One));


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat Det(float3 a, float3 b, float3 c) => math.dot(math.cross(a, b), c); // TODO: use math.determinant()?

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat RSqrtSafe(sfloat v) => math.select(math.rsqrt(v), sfloat.Zero, math.abs(v) < sfloat.FromRaw(0x2edbe6ff));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat NormalizeWithLength(float3 v, out float3 n)
        {
            sfloat lengthSq = math.lengthsq(v);
            sfloat invLength = math.rsqrt(lengthSq);
            n = v * invLength;
            return lengthSq * invLength;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsNormalized(float3 v)
        {
            sfloat lenZero = math.lengthsq(v) - sfloat.One;
            sfloat absLenZero = math.abs(lenZero);
            return absLenZero < Constants.UnityEpsilon;
        }

        // Return two normals perpendicular to the input vector
        public static void CalculatePerpendicularNormalized(float3 v, out float3 p, out float3 q)
        {
            float3 vSquared = v * v;
            float3 lengthsSquared = vSquared + vSquared.xxx; // y = ||j x v||^2, z = ||k x v||^2
            float3 invLengths = math.rsqrt(lengthsSquared);

            // select first direction, j x v or k x v, whichever has greater magnitude
            float3 dir0 = new float3(-v.y, v.x, sfloat.Zero);
            float3 dir1 = new float3(-v.z, sfloat.Zero, v.x);
            bool cmp = (lengthsSquared.y > lengthsSquared.z);
            float3 dir = math.select(dir1, dir0, cmp);

            // normalize and get the other direction
            sfloat invLength = math.select(invLengths.z, invLengths.y, cmp);
            p = dir * invLength;
            float3 cross = math.cross(v, dir);
            q = cross * invLength;
        }

        // Calculate the eigenvectors and eigenvalues of a symmetric 3x3 matrix
        public static void DiagonalizeSymmetricApproximation(float3x3 a, out float3x3 eigenVectors, out float3 eigenValues)
        {
            sfloat GetMatrixElement(float3x3 m, int row, int col)
            {
                switch (col)
                {
                    case 0: return m.c0[row];
                    case 1: return m.c1[row];
                    case 2: return m.c2[row];
                    default: UnityEngine.Assertions.Assert.IsTrue(false); return sfloat.Zero;
                }
            }

            void SetMatrixElement(ref float3x3 m, int row, int col, sfloat x)
            {
                switch (col)
                {
                    case 0: m.c0[row] = x; break;
                    case 1: m.c1[row] = x; break;
                    case 2: m.c2[row] = x; break;
                    default: UnityEngine.Assertions.Assert.IsTrue(false); break;
                }
            }

            eigenVectors = float3x3.identity;
            sfloat epsSq = sfloat.FromRaw(0x283424dc) * (math.lengthsq(a.c0) + math.lengthsq(a.c1) + math.lengthsq(a.c2));
            const int maxIterations = 10;
            for (int iteration = 0; iteration < maxIterations; iteration++)
            {
                // Find the row (p) and column (q) of the off-diagonal entry with greater magnitude
                int p = 0, q = 1;
                {
                    sfloat maxEntry = math.abs(a.c1[0]);
                    sfloat mag02 = math.abs(a.c2[0]);
                    sfloat mag12 = math.abs(a.c2[1]);
                    if (mag02 > maxEntry)
                    {
                        maxEntry = mag02;
                        p = 0;
                        q = 2;
                    }
                    if (mag12 > maxEntry)
                    {
                        maxEntry = mag12;
                        p = 1;
                        q = 2;
                    }

                    // Terminate if it's small enough
                    if (maxEntry * maxEntry < epsSq)
                    {
                        break;
                    }
                }

                // Calculate jacobia rotation
                float3x3 j = float3x3.identity;
                {
                    sfloat apq = GetMatrixElement(a, p, q);
                    sfloat tau = (GetMatrixElement(a, q, q) - GetMatrixElement(a, p, p)) / ((sfloat)2.0f * apq);
                    sfloat t = math.sqrt(sfloat.One + tau * tau);
                    if (tau > sfloat.Zero)
                    {
                        t = sfloat.One / (tau + t);
                    }
                    else
                    {
                        t = sfloat.One / (tau - t);
                    }
                    sfloat c = math.rsqrt(sfloat.One + t * t);
                    sfloat s = t * c;

                    SetMatrixElement(ref j, p, p, c);
                    SetMatrixElement(ref j, q, q, c);
                    SetMatrixElement(ref j, p, q, s);
                    SetMatrixElement(ref j, q, p, -s);
                }

                // Rotate a
                a = math.mul(math.transpose(j), math.mul(a, j));
                eigenVectors = math.mul(eigenVectors, j);
            }
            eigenValues = new float3(a.c0.x, a.c1.y, a.c2.z);
        }

        // Returns the twist angle of the swing-twist decomposition of q about i, j, or k corresponding to index = 0, 1, or 2 respectively.
        public static sfloat CalculateTwistAngle(quaternion q, int twistAxisIndex)
        {
            // q = swing * twist, twist = normalize(twistAxis * twistAxis dot q.xyz, q.w)
            sfloat dot = q.value[twistAxisIndex];
            sfloat w = q.value.w;
            sfloat lengthSq = dot * dot + w * w;
            sfloat invLength = RSqrtSafe(lengthSq);
            sfloat sinHalfAngle = dot * invLength;
            sfloat cosHalfAngle = w * invLength;
            sfloat halfAngle = math.atan2(sinHalfAngle, cosHalfAngle);
            return halfAngle + halfAngle;
        }

        // Returns a quaternion q with q * from = to
        public static quaternion FromToRotation(float3 from, float3 to)
        {
            Assert.IsTrue(math.abs(math.lengthsq(from) - sfloat.One) < sfloat.FromRaw(0x38d1b717));
            Assert.IsTrue(math.abs(math.lengthsq(to) - sfloat.One) < sfloat.FromRaw(0x38d1b717));
            float3 cross = math.cross(from, to);
            CalculatePerpendicularNormalized(from, out float3 safeAxis, out float3 unused); // for when angle ~= 180
            sfloat dot = math.dot(from, to);
            float3 squares = new float3((sfloat)0.5f - new float2(dot, -dot) * (sfloat)0.5f, math.lengthsq(cross));
            float3 inverses = math.select(math.rsqrt(squares), sfloat.Zero, squares < sfloat.FromRaw(0x2edbe6ff));
            float2 sinCosHalfAngle = squares.xy * inverses.xy;
            float3 axis = math.select(cross * inverses.z, safeAxis, squares.z < sfloat.FromRaw(0x2edbe6ff));
            return new quaternion(new float4(axis * sinCosHalfAngle.x, sinCosHalfAngle.y));
        }

        
        // Note: taken from Unity.Animation/Core/MathExtensions.cs, which will be moved to Unity.Mathematics at some point
        //       after that, this should be removed and the Mathematics version should be used
        #region toEuler
        static float3 toEuler(quaternion q, math.RotationOrder order = math.RotationOrder.Default)
        {
            //prepare the data
            var qv = q.value;
            var d1 = qv * qv.wwww * new float4((sfloat)2.0f); //xw, yw, zw, ww
            var d2 = qv * qv.yzxw * new float4((sfloat)2.0f); //xy, yz, zx, ww
            var d3 = qv * qv;
            var euler = new float3(sfloat.Zero);

            //const float epsilon = 1e-6f;
            //const float CUTOFF = (1.0f - 2.0f * epsilon) * (1.0f - 2.0f * epsilon);
            const uint CUTOFF_U32 = 0x3f7fffbd;
            sfloat CUTOFF = sfloat.FromRaw(CUTOFF_U32);

            switch (order)
            {
                case math.RotationOrder.ZYX:
                {
                    var y1 = d2.z + d1.y;
                    if (y1 * y1 < CUTOFF)
                    {
                        var x1 = -d2.x + d1.z;
                        var x2 = d3.x + d3.w - d3.y - d3.z;
                        var z1 = -d2.y + d1.x;
                        var z2 = d3.z + d3.w - d3.y - d3.x;
                        euler = new float3(math.atan2(x1, x2), math.asin(y1), math.atan2(z1, z2));
                    }
                    else //zxz
                    {
                        y1 = math.clamp(y1, -sfloat.One, sfloat.One);
                        var abcd = new float4(d2.z, d1.y, d2.y, d1.x);
                        var x1 = (sfloat)2.0f * (abcd.x * abcd.w + abcd.y * abcd.z); //2(ad+bc)
                        var x2 = math.csum(abcd * abcd * new float4(-sfloat.One, sfloat.One, -sfloat.One, sfloat.One));
                        euler = new float3(math.atan2(x1, x2), math.asin(y1), sfloat.Zero);
                    }

                    break;
                }

                case math.RotationOrder.ZXY:
                {
                    var y1 = d2.y - d1.x;
                    if (y1 * y1 < CUTOFF)
                    {
                        var x1 = d2.x + d1.z;
                        var x2 = d3.y + d3.w - d3.x - d3.z;
                        var z1 = d2.z + d1.y;
                        var z2 = d3.z + d3.w - d3.x - d3.y;
                        euler = new float3(math.atan2(x1, x2), -math.asin(y1), math.atan2(z1, z2));
                    }
                    else //zxz
                    {
                        y1 = math.clamp(y1, -sfloat.One, sfloat.One);
                        var abcd = new float4(d2.z, d1.y, d2.y, d1.x);
                        var x1 = (sfloat)2.0f * (abcd.x * abcd.w + abcd.y * abcd.z); //2(ad+bc)
                        var x2 = math.csum(abcd * abcd * new float4(-sfloat.One, sfloat.One, -sfloat.One, sfloat.One));
                        euler = new float3(math.atan2(x1, x2), -math.asin(y1), sfloat.Zero);
                    }

                    break;
                }

                case math.RotationOrder.YXZ:
                {
                    var y1 = d2.y + d1.x;
                    if (y1 * y1 < CUTOFF)
                    {
                        var x1 = -d2.z + d1.y;
                        var x2 = d3.z + d3.w - d3.x - d3.y;
                        var z1 = -d2.x + d1.z;
                        var z2 = d3.y + d3.w - d3.z - d3.x;
                        euler = new float3(math.atan2(x1, x2), math.asin(y1), math.atan2(z1, z2));
                    }
                    else //yzy
                    {
                        y1 = math.clamp(y1, -sfloat.One, sfloat.One);
                        var abcd = new float4(d2.x, d1.z, d2.y, d1.x);
                        var x1 = (sfloat)2.0f * (abcd.x * abcd.w + abcd.y * abcd.z); //2(ad+bc)
                        var x2 = math.csum(abcd * abcd * new float4(-sfloat.One, sfloat.One, -sfloat.One, sfloat.One));
                        euler = new float3(math.atan2(x1, x2), math.asin(y1), sfloat.Zero);
                    }

                    break;
                }

                case math.RotationOrder.YZX:
                {
                    var y1 = d2.x - d1.z;
                    if (y1 * y1 < CUTOFF)
                    {
                        var x1 = d2.z + d1.y;
                        var x2 = d3.x + d3.w - d3.z - d3.y;
                        var z1 = d2.y + d1.x;
                        var z2 = d3.y + d3.w - d3.x - d3.z;
                        euler = new float3(math.atan2(x1, x2), -math.asin(y1), math.atan2(z1, z2));
                    }
                    else //yxy
                    {
                        y1 = math.clamp(y1, -sfloat.One, sfloat.One);
                        var abcd = new float4(d2.x, d1.z, d2.y, d1.x);
                        var x1 = (sfloat)2.0f * (abcd.x * abcd.w + abcd.y * abcd.z); //2(ad+bc)
                        var x2 = math.csum(abcd * abcd * new float4(-sfloat.One, sfloat.One, -sfloat.One, sfloat.One));
                        euler = new float3(math.atan2(x1, x2), -math.asin(y1), sfloat.Zero);
                    }

                    break;
                }

                case math.RotationOrder.XZY:
                {
                    var y1 = d2.x + d1.z;
                    if (y1 * y1 < CUTOFF)
                    {
                        var x1 = -d2.y + d1.x;
                        var x2 = d3.y + d3.w - d3.z - d3.x;
                        var z1 = -d2.z + d1.y;
                        var z2 = d3.x + d3.w - d3.y - d3.z;
                        euler = new float3(math.atan2(x1, x2), math.asin(y1), math.atan2(z1, z2));
                    }
                    else //xyx
                    {
                        y1 = math.clamp(y1, -sfloat.One, sfloat.One);
                        var abcd = new float4(d2.x, d1.z, d2.z, d1.y);
                        var x1 = (sfloat)2.0f * (abcd.x * abcd.w + abcd.y * abcd.z); //2(ad+bc)
                        var x2 = math.csum(abcd * abcd * new float4(-sfloat.One, sfloat.One, -sfloat.One, sfloat.One));
                        euler = new float3(math.atan2(x1, x2), math.asin(y1), sfloat.Zero);
                    }

                    break;
                }

                case math.RotationOrder.XYZ:
                {
                    var y1 = d2.z - d1.y;
                    if (y1 * y1 < CUTOFF)
                    {
                        var x1 = d2.y + d1.x;
                        var x2 = d3.z + d3.w - d3.y - d3.x;
                        var z1 = d2.x + d1.z;
                        var z2 = d3.x + d3.w - d3.y - d3.z;
                        euler = new float3(math.atan2(x1, x2), -math.asin(y1), math.atan2(z1, z2));
                    } else //xzx
                    {
                        y1 = math.clamp(y1, -sfloat.One, sfloat.One);
                        var abcd = new float4(d2.z, d1.y, d2.x, d1.z);
                        var x1 = (sfloat)2.0f * (abcd.x * abcd.w + abcd.y * abcd.z); //2(ad+bc)
                        var x2 = math.csum(abcd * abcd * new float4(-sfloat.One, sfloat.One, -sfloat.One, sfloat.One));
                        euler = new float3(math.atan2(x1, x2), -math.asin(y1), sfloat.Zero);
                    }

                    break;
                }
            }

            return eulerReorderBack(euler, order);
        }

        static float3 eulerReorderBack(float3 euler, math.RotationOrder order)
        {
            switch (order)
            {
                case math.RotationOrder.XZY:
                    return euler.xzy;
                case math.RotationOrder.YZX:
                    return euler.zxy;
                case math.RotationOrder.YXZ:
                    return euler.yxz;
                case math.RotationOrder.ZXY:
                    return euler.yzx;
                case math.RotationOrder.ZYX:
                    return euler.zyx;
                case math.RotationOrder.XYZ:
                default:
                    return euler;
            }
        }
        #endregion

        /// <summary>
        /// Convert a quaternion orientation to Euler angles.
        /// Use this method to calculate angular velocity needed to achieve a target orientation.
        /// </summary>
        /// <param name="q">An orientation.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static float3 ToEulerAngles(this quaternion q, math.RotationOrder order = math.RotationOrder.XYZ)
        {
            return toEuler(q, order);
        }

        // Returns the angle in degrees between /from/ and /to/. This is always the smallest
        internal static sfloat Angle(float3 from, float3 to)
        {
            // sqrt(a) * sqrt(b) = sqrt(a * b) -- valid for real numbers
            var denominator = math.sqrt(math.lengthsq(from) * math.lengthsq(to));
            if (denominator < Constants.UnityEpsilonNormalSqrt)
                return sfloat.Zero;

            var dot = math.clamp(math.dot(from, to) / denominator, -sfloat.One, sfloat.One);
            return math.degrees(math.acos(dot));
        }

        // The smaller of the two possible angles between the two vectors is returned, therefore the result will never be greater than 180 degrees or smaller than -180 degrees.
        // If you imagine the from and to vectors as lines on a piece of paper, both originating from the same point, then the /axis/ vector would point up out of the paper.
        // The measured angle between the two vectors would be positive in a clockwise direction and negative in an anti-clockwise direction.
        internal static sfloat SignedAngle(float3 from, float3 to, float3 axis)
        {
            var unsignedAngle = Angle(from, to);
            var sign = math.sign(math.dot(math.cross(from, to), axis));
            return unsignedAngle * sign;
        }

        // Projects a vector onto a plane defined by a normal orthogonal to the plane.
        internal static float3 ProjectOnPlane(float3 vector, float3 planeNormal)
        {
            var sqrMag = math.dot(planeNormal, planeNormal);
            if (sqrMag < Constants.UnityEpsilon)
                return vector;

            var dot = math.dot(vector, planeNormal);
            return vector - planeNormal * (dot / sqrMag);
        }

        /// <summary>
        /// Physics internally represents all rigid bodies in world space.
        /// If a static body is in a hierarchy, its local-to-world matrix must be decomposed when building the physics world.
        /// This method returns a world-space RigidTransform that would be decomposed for such a rigid body.
        /// </summary>
        /// <returns>A world-space RigidTransform as used by physics.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static RigidTransform DecomposeRigidBodyTransform(in float4x4 localToWorld) =>
            new RigidTransform(DecomposeRigidBodyOrientation(localToWorld), localToWorld.c3.xyz);

        /// <summary>
        /// Physics internally represents all rigid bodies in world space.
        /// If a static body is in a hierarchy, its local-to-world matrix must be decomposed when building the physics world.
        /// This method returns a world-space orientation that would be decomposed for such a rigid body.
        /// </summary>
        /// <returns>A world-space orientation as used by physics.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion DecomposeRigidBodyOrientation(in float4x4 localToWorld) =>
            quaternion.LookRotationSafe(localToWorld.c2.xyz, localToWorld.c1.xyz);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static float3 DecomposeScale(this float4x4 matrix) =>
            new float3(math.length(matrix.c0.xyz), math.length(matrix.c1.xyz), math.length(matrix.c2.xyz));
    }
}
