using System.Runtime.CompilerServices;
using static UnityS.Mathematics.math;

namespace UnityS.Mathematics
{
    public partial struct float2x2
    {
        /// <summary>Returns a float2x2 matrix representing a counter-clockwise rotation of angle degrees.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2x2 Rotate(sfloat angle)
        {
            sfloat s, c;
            sincos(angle, out s, out c);
            return float2x2(c, -s,
                            s,  c);
        }

        /// <summary>Returns a float2x2 matrix representing a uniform scaling of both axes by s.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2x2 Scale(sfloat s)
        {
            return float2x2(s,    sfloat.Zero,
                            sfloat.Zero, s);
        }

        /// <summary>Returns a float2x2 matrix representing a non-uniform axis scaling by x and y.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2x2 Scale(sfloat x, sfloat y)
        {
            return float2x2(x, sfloat.Zero,
                            sfloat.Zero, y);
        }

        /// <summary>Returns a float2x2 matrix representing a non-uniform axis scaling by the components of the float2 vector v.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2x2 Scale(float2 v)
        {
            return Scale(v.x, v.y);
        }
    }

    public partial struct float3x3
    {
        /// <summary>
        /// Constructs a float3x3 from the upper left 3x3 of a float4x4.
        /// </summary>
        /// <param name="f4x4"><see cref="float4x4"/> to extract a float3x3 from.</param>
        public float3x3(float4x4 f4x4)
        {
            c0 = f4x4.c0.xyz;
            c1 = f4x4.c1.xyz;
            c2 = f4x4.c2.xyz;
        }

        /// <summary>Constructs a float3x3 matrix from a unit quaternion.</summary>
        public float3x3(quaternion q)
        {
            float4 v = q.value;
            float4 v2 = v + v;

            uint3 npn = uint3(0x80000000, 0x00000000, 0x80000000);
            uint3 nnp = uint3(0x80000000, 0x80000000, 0x00000000);
            uint3 pnn = uint3(0x00000000, 0x80000000, 0x80000000);
            c0 = v2.y * asfloat(asuint(v.yxw) ^ npn) - v2.z * asfloat(asuint(v.zwx) ^ pnn) + float3(sfloat.One, sfloat.Zero, sfloat.Zero);
            c1 = v2.z * asfloat(asuint(v.wzy) ^ nnp) - v2.x * asfloat(asuint(v.yxw) ^ npn) + float3(sfloat.Zero, sfloat.One, sfloat.Zero);
            c2 = v2.x * asfloat(asuint(v.zwx) ^ pnn) - v2.y * asfloat(asuint(v.wzy) ^ nnp) + float3(sfloat.Zero, sfloat.Zero, sfloat.One);
        }

        /// <summary>
        /// Returns a float3x3 matrix representing a rotation around a unit axis by an angle in radians.
        /// The rotation direction is clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x3 AxisAngle(float3 axis, sfloat angle)
        {
            sfloat sina, cosa;
            math.sincos(angle, out sina, out cosa);

            float3 u = axis;
            float3 u_yzx = u.yzx;
            float3 u_zxy = u.zxy;
            float3 u_inv_cosa = u - u * cosa;  // u * (1.0f - cosa);
            float4 t = float4(u * sina, cosa);

            uint3 ppn = uint3(0x00000000, 0x00000000, 0x80000000);
            uint3 npp = uint3(0x80000000, 0x00000000, 0x00000000);
            uint3 pnp = uint3(0x00000000, 0x80000000, 0x00000000);

            return float3x3(
                u.x * u_inv_cosa + asfloat(asuint(t.wzy) ^ ppn),
                u.y * u_inv_cosa + asfloat(asuint(t.zwx) ^ npp),
                u.z * u_inv_cosa + asfloat(asuint(t.yxw) ^ pnp)
                );
            /*
            return float3x3(
                cosa + u.x * u.x * (1.0f - cosa),       u.y * u.x * (1.0f - cosa) - u.z * sina, u.z * u.x * (1.0f - cosa) + u.y * sina,
                u.x * u.y * (1.0f - cosa) + u.z * sina, cosa + u.y * u.y * (1.0f - cosa),       u.y * u.z * (1.0f - cosa) - u.x * sina,
                u.x * u.z * (1.0f - cosa) - u.y * sina, u.y * u.z * (1.0f - cosa) + u.x * sina, cosa + u.z * u.z * (1.0f - cosa)
                );
                */
        }

        /// <summary>
        /// Returns a float3x3 rotation matrix constructed by first performing a rotation around the x-axis, then the y-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x3 EulerXYZ(float3 xyz)
        {
            // return mul(rotateZ(xyz.z), mul(rotateY(xyz.y), rotateX(xyz.x)));
            float3 s, c;
            sincos(xyz, out s, out c);
            return float3x3(
                c.y * c.z,  c.z * s.x * s.y - c.x * s.z,    c.x * c.z * s.y + s.x * s.z,
                c.y * s.z,  c.x * c.z + s.x * s.y * s.z,    c.x * s.y * s.z - c.z * s.x,
                -s.y,       c.y * s.x,                      c.x * c.y
                );
        }

        /// <summary>
        /// Returns a float3x3 rotation matrix constructed by first performing a rotation around the x-axis, then the z-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x3 EulerXZY(float3 xyz)
        {
            // return mul(rotateY(xyz.y), mul(rotateZ(xyz.z), rotateX(xyz.x))); }
            float3 s, c;
            sincos(xyz, out s, out c);
            return float3x3(
                c.y * c.z,  s.x * s.y - c.x * c.y * s.z,    c.x * s.y + c.y * s.x * s.z,
                s.z,        c.x * c.z,                      -c.z * s.x,
                -c.z * s.y, c.y * s.x + c.x * s.y * s.z,    c.x * c.y - s.x * s.y * s.z
                );
        }

        /// <summary>
        /// Returns a float3x3 rotation matrix constructed by first performing a rotation around the y-axis, then the x-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x3 EulerYXZ(float3 xyz)
        {
            // return mul(rotateZ(xyz.z), mul(rotateX(xyz.x), rotateY(xyz.y)));
            float3 s, c;
            sincos(xyz, out s, out c);
            return float3x3(
                c.y * c.z - s.x * s.y * s.z,    -c.x * s.z, c.z * s.y + c.y * s.x * s.z,
                c.z * s.x * s.y + c.y * s.z,    c.x * c.z,  s.y * s.z - c.y * c.z * s.x,
                -c.x * s.y,                     s.x,        c.x * c.y
                );
        }

        /// <summary>
        /// Returns a float3x3 rotation matrix constructed by first performing a rotation around the y-axis, then the z-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x3 EulerYZX(float3 xyz)
        {
            // return mul(rotateX(xyz.x), mul(rotateZ(xyz.z), rotateY(xyz.y)));
            float3 s, c;
            sincos(xyz, out s, out c);
            return float3x3(
                c.y * c.z,                      -s.z,       c.z * s.y,
                s.x * s.y + c.x * c.y * s.z,    c.x * c.z,  c.x * s.y * s.z - c.y * s.x,
                c.y * s.x * s.z - c.x * s.y,    c.z * s.x,  c.x * c.y + s.x * s.y * s.z
                );
        }

        /// <summary>
        /// Returns a float3x3 rotation matrix constructed by first performing a rotation around the z-axis, then the x-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// This is the default order rotation order in Unity.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x3 EulerZXY(float3 xyz)
        {
            // return mul(rotateY(xyz.y), mul(rotateX(xyz.x), rotateZ(xyz.z)));
            float3 s, c;
            sincos(xyz, out s, out c);
            return float3x3(
                c.y * c.z + s.x * s.y * s.z,    c.z * s.x * s.y - c.y * s.z,    c.x * s.y,
                c.x * s.z,                      c.x * c.z,                      -s.x,
                c.y * s.x * s.z - c.z * s.y,    c.y * c.z * s.x + s.y * s.z,    c.x * c.y
                );
        }

        /// <summary>
        /// Returns a float3x3 rotation matrix constructed by first performing a rotation around the z-axis, then the y-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x3 EulerZYX(float3 xyz)
        {
            // return mul(rotateX(xyz.x), mul(rotateY(xyz.y), rotateZ(xyz.z)));
            float3 s, c;
            sincos(xyz, out s, out c);
            return float3x3(
                c.y * c.z,                      -c.y * s.z,                     s.y,
                c.z * s.x * s.y + c.x * s.z,    c.x * c.z - s.x * s.y * s.z,    -c.y * s.x,
                s.x * s.z - c.x * c.z * s.y,    c.z * s.x + c.x * s.y * s.z,    c.x * c.y
                );
        }

        /// <summary>
        /// Returns a float3x3 rotation matrix constructed by first performing a rotation around the x-axis, then the y-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x3 EulerXYZ(sfloat x, sfloat y, sfloat z) { return EulerXYZ(float3(x, y, z)); }

        /// <summary>
        /// Returns a float3x3 rotation matrix constructed by first performing a rotation around the x-axis, then the z-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x3 EulerXZY(sfloat x, sfloat y, sfloat z) { return EulerXZY(float3(x, y, z)); }

        /// <summary>
        /// Returns a float3x3 rotation matrix constructed by first performing a rotation around the y-axis, then the x-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x3 EulerYXZ(sfloat x, sfloat y, sfloat z) { return EulerYXZ(float3(x, y, z)); }

        /// <summary>
        /// Returns a float3x3 rotation matrix constructed by first performing a rotation around the y-axis, then the z-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x3 EulerYZX(sfloat x, sfloat y, sfloat z) { return EulerYZX(float3(x, y, z)); }

        /// <summary>
        /// Returns a float3x3 rotation matrix constructed by first performing a rotation around the z-axis, then the x-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// This is the default order rotation order in Unity.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x3 EulerZXY(sfloat x, sfloat y, sfloat z) { return EulerZXY(float3(x, y, z)); }

        /// <summary>
        /// Returns a float3x3 rotation matrix constructed by first performing a rotation around the z-axis, then the y-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x3 EulerZYX(sfloat x, sfloat y, sfloat z) { return EulerZYX(float3(x, y, z)); }

        /// <summary>
        /// Returns a float3x3 rotation matrix constructed by first performing 3 rotations around the principal axes in a given order.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// When the rotation order is known at compile time, it is recommended for performance reasons to use specific
        /// Euler rotation constructors such as EulerZXY(...).
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <param name="order">The order in which the rotations are applied.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x3 Euler(float3 xyz, RotationOrder order = RotationOrder.Default)
        {
            switch (order)
            {
                case RotationOrder.XYZ:
                    return EulerXYZ(xyz);
                case RotationOrder.XZY:
                    return EulerXZY(xyz);
                case RotationOrder.YXZ:
                    return EulerYXZ(xyz);
                case RotationOrder.YZX:
                    return EulerYZX(xyz);
                case RotationOrder.ZXY:
                    return EulerZXY(xyz);
                case RotationOrder.ZYX:
                    return EulerZYX(xyz);
                default:
                    return float3x3.identity;
            }
        }

        /// <summary>
        /// Returns a float3x3 rotation matrix constructed by first performing 3 rotations around the principal axes in a given order.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// When the rotation order is known at compile time, it is recommended for performance reasons to use specific
        /// Euler rotation constructors such as EulerZXY(...).
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <param name="order">The order in which the rotations are applied.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x3 Euler(sfloat x, sfloat y, sfloat z, RotationOrder order = RotationOrder.Default)
        {
            return Euler(float3(x, y, z), order);
        }

        /// <summary>Returns a float4x4 matrix that rotates around the x-axis by a given number of radians.</summary>
        /// <param name="angle">The clockwise rotation angle when looking along the x-axis towards the origin in radians.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x3 RotateX(sfloat angle)
        {
            // {{1, 0, 0}, {0, c_0, -s_0}, {0, s_0, c_0}}
            sfloat s, c;
            sincos(angle, out s, out c);
            return float3x3(sfloat.One, sfloat.Zero, sfloat.Zero,
                            sfloat.Zero, c,    -s,
                            sfloat.Zero, s,    c);
        }

        /// <summary>Returns a float4x4 matrix that rotates around the y-axis by a given number of radians.</summary>
        /// <param name="angle">The clockwise rotation angle when looking along the y-axis towards the origin in radians.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x3 RotateY(sfloat angle)
        {
            // {{c_1, 0, s_1}, {0, 1, 0}, {-s_1, 0, c_1}}
            sfloat s, c;
            sincos(angle, out s, out c);
            return float3x3(c, sfloat.Zero, s,
                            sfloat.Zero, sfloat.One, sfloat.Zero,
                            -s, sfloat.Zero, c);
        }

        /// <summary>Returns a float4x4 matrix that rotates around the z-axis by a given number of radians.</summary>
        /// <param name="angle">The clockwise rotation angle when looking along the z-axis towards the origin in radians.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x3 RotateZ(sfloat angle)
        {
            // {{c_2, -s_2, 0}, {s_2, c_2, 0}, {0, 0, 1}}
            sfloat s, c;
            sincos(angle, out s, out c);
            return float3x3(c,    -s, sfloat.Zero,
                            s,    c, sfloat.Zero,
                            sfloat.Zero, sfloat.Zero, sfloat.One);
        }

        //<summary>Returns a float3x3 matrix representing a uniform scaling of all axes by s.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x3 Scale(sfloat s)
        {
            return float3x3(s, sfloat.Zero, sfloat.Zero,
                            sfloat.Zero, s, sfloat.Zero,
                            sfloat.Zero, sfloat.Zero, s);
        }

        /// <summary>Returns a float3x3 matrix representing a non-uniform axis scaling by x, y and z.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x3 Scale(sfloat x, sfloat y, sfloat z)
        {
            return float3x3(x, sfloat.Zero, sfloat.Zero,
                            sfloat.Zero, y, sfloat.Zero,
                            sfloat.Zero, sfloat.Zero, z);
        }

        /// <summary>Returns a float3x3 matrix representing a non-uniform axis scaling by the components of the float3 vector v.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x3 Scale(float3 v)
        {
            return Scale(v.x, v.y, v.z);
        }

        /// <summary>
        /// Returns a float3x3 view rotation matrix given a unit length forward vector and a unit length up vector.
        /// The two input vectors are assumed to be unit length and not collinear.
        /// If these assumptions are not met use float3x3.LookRotationSafe instead.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x3 LookRotation(float3 forward, float3 up)
        {
            float3 t = normalize(cross(up, forward));
            return float3x3(t, cross(forward, t), forward);
        }

        /// <summary>
        /// Returns a float3x3 view rotation matrix given a forward vector and an up vector.
        /// The two input vectors are not assumed to be unit length.
        /// If the magnitude of either of the vectors is so extreme that the calculation cannot be carried out reliably or the vectors are collinear,
        /// the identity will be returned instead.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x3 LookRotationSafe(float3 forward, float3 up)
        {
            sfloat forwardLengthSq = dot(forward, forward);
            sfloat upLengthSq = dot(up, up);

            forward *= rsqrt(forwardLengthSq);
            up *= rsqrt(upLengthSq);

            float3 t = cross(up, forward);
            sfloat tLengthSq = dot(t, t);
            t *= rsqrt(tLengthSq);

            sfloat mn = min(min(forwardLengthSq, upLengthSq), tLengthSq);
            sfloat mx = max(max(forwardLengthSq, upLengthSq), tLengthSq);

            const uint bigValue = 0x799a130c;
            const uint smallValue = 0x0554ad2e;

            bool accept = mn > sfloat.FromRaw(smallValue) && mx < sfloat.FromRaw(bigValue) && isfinite(forwardLengthSq) && isfinite(upLengthSq) && isfinite(tLengthSq);
            return float3x3(
                select(float3(sfloat.One, sfloat.Zero, sfloat.Zero), t, accept),
                select(float3(sfloat.Zero, sfloat.One, sfloat.Zero), cross(forward, t), accept),
                select(float3(sfloat.Zero, sfloat.Zero, sfloat.One), forward, accept));
        }

        public static explicit operator float3x3(float4x4 f4x4) => new float3x3(f4x4);
    }

    public partial struct float4x4
    {
        /// <summary>Constructs a float4x4 from a float3x3 rotation matrix and a float3 translation vector.</summary>
        public float4x4(float3x3 rotation, float3 translation)
        {
            c0 = float4(rotation.c0, sfloat.Zero);
            c1 = float4(rotation.c1, sfloat.Zero);
            c2 = float4(rotation.c2, sfloat.Zero);
            c3 = float4(translation, sfloat.One);
        }

        /// <summary>Constructs a float4x4 from a quaternion and a float3 translation vector.</summary>
        public float4x4(quaternion rotation, float3 translation)
        {
            float3x3 rot = float3x3(rotation);
            c0 = float4(rot.c0, sfloat.Zero);
            c1 = float4(rot.c1, sfloat.Zero);
            c2 = float4(rot.c2, sfloat.Zero);
            c3 = float4(translation, sfloat.One);
        }

        /// <summary>Constructs a float4x4 from a RigidTransform.</summary>
        public float4x4(RigidTransform transform)
        {
            float3x3 rot = float3x3(transform.rot);
            c0 = float4(rot.c0, sfloat.Zero);
            c1 = float4(rot.c1, sfloat.Zero);
            c2 = float4(rot.c2, sfloat.Zero);
            c3 = float4(transform.pos, sfloat.One);
        }

        /// <summary>
        /// Returns a float4x4 matrix representing a rotation around a unit axis by an angle in radians.
        /// The rotation direction is clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 AxisAngle(float3 axis, sfloat angle)
        {
            sfloat sina, cosa;
            math.sincos(angle, out sina, out cosa);

            float4 u = float4(axis, sfloat.Zero);
            float4 u_yzx = u.yzxx;
            float4 u_zxy = u.zxyx;
            float4 u_inv_cosa = u - u * cosa;  // u * (1.0f - cosa);
            float4 t = float4(u.xyz * sina, cosa);

            uint4 ppnp = uint4(0x00000000, 0x00000000, 0x80000000, 0x00000000);
            uint4 nppp = uint4(0x80000000, 0x00000000, 0x00000000, 0x00000000);
            uint4 pnpp = uint4(0x00000000, 0x80000000, 0x00000000, 0x00000000);
            uint4 mask = uint4(0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x00000000);

            return float4x4(
                u.x * u_inv_cosa + asfloat((asuint(t.wzyx) ^ ppnp) & mask),
                u.y * u_inv_cosa + asfloat((asuint(t.zwxx) ^ nppp) & mask),
                u.z * u_inv_cosa + asfloat((asuint(t.yxwx) ^ pnpp) & mask),
                float4(sfloat.Zero, sfloat.Zero, sfloat.Zero, sfloat.One)
                );

        }

        /// <summary>
        /// Returns a float4x4 rotation matrix constructed by first performing a rotation around the x-axis, then the y-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 EulerXYZ(float3 xyz)
        {
            // return mul(rotateZ(xyz.z), mul(rotateY(xyz.y), rotateX(xyz.x)));
            float3 s, c;
            sincos(xyz, out s, out c);
            return float4x4(
                c.y * c.z,  c.z * s.x * s.y - c.x * s.z,    c.x * c.z * s.y + s.x * s.z,    sfloat.Zero,
                c.y * s.z,  c.x * c.z + s.x * s.y * s.z,    c.x * s.y * s.z - c.z * s.x,    sfloat.Zero,
                -s.y,       c.y * s.x,                      c.x * c.y,                      sfloat.Zero,
                sfloat.Zero,sfloat.Zero,                    sfloat.Zero,                    sfloat.One
                );
        }

        /// <summary>
        /// Returns a float4x4 rotation matrix constructed by first performing a rotation around the x-axis, then the z-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 EulerXZY(float3 xyz)
        {
            // return mul(rotateY(xyz.y), mul(rotateZ(xyz.z), rotateX(xyz.x))); }
            float3 s, c;
            sincos(xyz, out s, out c);
            return float4x4(
                c.y * c.z,  s.x * s.y - c.x * c.y * s.z,    c.x * s.y + c.y * s.x * s.z,    sfloat.Zero,
                s.z,        c.x * c.z,                      -c.z * s.x,                     sfloat.Zero,
                -c.z * s.y, c.y * s.x + c.x * s.y * s.z,    c.x * c.y - s.x * s.y * s.z,    sfloat.Zero,
                sfloat.Zero,sfloat.Zero,                    sfloat.Zero,                    sfloat.One
                );
        }

        /// <summary>
        /// Returns a float4x4 rotation matrix constructed by first performing a rotation around the y-axis, then the x-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 EulerYXZ(float3 xyz)
        {
            // return mul(rotateZ(xyz.z), mul(rotateX(xyz.x), rotateY(xyz.y)));
            float3 s, c;
            sincos(xyz, out s, out c);
            return float4x4(
                c.y * c.z - s.x * s.y * s.z,    -c.x * s.z, c.z * s.y + c.y * s.x * s.z,    sfloat.Zero,
                c.z * s.x * s.y + c.y * s.z,    c.x * c.z,  s.y * s.z - c.y * c.z * s.x,    sfloat.Zero,
                -c.x * s.y,                     s.x,        c.x * c.y,                      sfloat.Zero,
                sfloat.Zero,                    sfloat.Zero,sfloat.Zero,                    sfloat.One
                );
        }

        /// <summary>
        /// Returns a float4x4 rotation matrix constructed by first performing a rotation around the y-axis, then the z-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 EulerYZX(float3 xyz)
        {
            // return mul(rotateX(xyz.x), mul(rotateZ(xyz.z), rotateY(xyz.y)));
            float3 s, c;
            sincos(xyz, out s, out c);
            return float4x4(
                c.y * c.z,                      -s.z,       c.z * s.y,                      sfloat.Zero,
                s.x * s.y + c.x * c.y * s.z,    c.x * c.z,  c.x * s.y * s.z - c.y * s.x,    sfloat.Zero,
                c.y * s.x * s.z - c.x * s.y,    c.z * s.x,  c.x * c.y + s.x * s.y * s.z,    sfloat.Zero,
                sfloat.Zero,                    sfloat.Zero,sfloat.Zero,                    sfloat.One
                );
        }

        /// <summary>
        /// Returns a float4x4 rotation matrix constructed by first performing a rotation around the z-axis, then the x-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// This is the default order rotation order in Unity.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 EulerZXY(float3 xyz)
        {
            // return mul(rotateY(xyz.y), mul(rotateX(xyz.x), rotateZ(xyz.z)));
            float3 s, c;
            sincos(xyz, out s, out c);
            return float4x4(
                c.y * c.z + s.x * s.y * s.z,    c.z * s.x * s.y - c.y * s.z,    c.x * s.y,  sfloat.Zero,
                c.x * s.z,                      c.x * c.z,                      -s.x,       sfloat.Zero,
                c.y * s.x * s.z - c.z * s.y,    c.y * c.z * s.x + s.y * s.z,    c.x * c.y,  sfloat.Zero,
                sfloat.Zero,                    sfloat.Zero,                    sfloat.Zero,sfloat.One
                );
        }

        /// <summary>
        /// Returns a float4x4 rotation matrix constructed by first performing a rotation around the z-axis, then the y-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 EulerZYX(float3 xyz)
        {
            // return mul(rotateX(xyz.x), mul(rotateY(xyz.y), rotateZ(xyz.z)));
            float3 s, c;
            sincos(xyz, out s, out c);
            return float4x4(
                c.y * c.z,                      -c.y * s.z,                     s.y,        sfloat.Zero,
                c.z * s.x * s.y + c.x * s.z,    c.x * c.z - s.x * s.y * s.z,    -c.y * s.x, sfloat.Zero,
                s.x * s.z - c.x * c.z * s.y,    c.z * s.x + c.x * s.y * s.z,    c.x * c.y,  sfloat.Zero,
                sfloat.Zero,                    sfloat.Zero,                    sfloat.Zero,sfloat.One
                );
        }

        /// <summary>
        /// Returns a float4x4 rotation matrix constructed by first performing a rotation around the x-axis, then the y-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 EulerXYZ(sfloat x, sfloat y, sfloat z) { return EulerXYZ(float3(x, y, z)); }

        /// <summary>
        /// Returns a float4x4 rotation matrix constructed by first performing a rotation around the x-axis, then the z-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 EulerXZY(sfloat x, sfloat y, sfloat z) { return EulerXZY(float3(x, y, z)); }

        /// <summary>
        /// Returns a float4x4 rotation matrix constructed by first performing a rotation around the y-axis, then the x-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 EulerYXZ(sfloat x, sfloat y, sfloat z) { return EulerYXZ(float3(x, y, z)); }

        /// <summary>
        /// Returns a float4x4 rotation matrix constructed by first performing a rotation around the y-axis, then the z-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 EulerYZX(sfloat x, sfloat y, sfloat z) { return EulerYZX(float3(x, y, z)); }

        /// <summary>
        /// Returns a float4x4 rotation matrix constructed by first performing a rotation around the z-axis, then the x-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// This is the default order rotation order in Unity.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 EulerZXY(sfloat x, sfloat y, sfloat z) { return EulerZXY(float3(x, y, z)); }

        /// <summary>
        /// Returns a float4x4 rotation matrix constructed by first performing a rotation around the z-axis, then the y-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 EulerZYX(sfloat x, sfloat y, sfloat z) { return EulerZYX(float3(x, y, z)); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 Euler(float3 xyz, RotationOrder order = RotationOrder.Default)
        {
            switch (order)
            {
                case RotationOrder.XYZ:
                    return EulerXYZ(xyz);
                case RotationOrder.XZY:
                    return EulerXZY(xyz);
                case RotationOrder.YXZ:
                    return EulerYXZ(xyz);
                case RotationOrder.YZX:
                    return EulerYZX(xyz);
                case RotationOrder.ZXY:
                    return EulerZXY(xyz);
                case RotationOrder.ZYX:
                    return EulerZYX(xyz);
                default:
                    return float4x4.identity;
            }
        }

        /// <summary>
        /// Returns a float4x4 rotation matrix constructed by first performing 3 rotations around the principal axes in a given order.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// When the rotation order is known at compile time, it is recommended for performance reasons to use specific
        /// Euler rotation constructors such as EulerZXY(...).
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <param name="order">The order in which the rotations are applied.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 Euler(sfloat x, sfloat y, sfloat z, RotationOrder order = RotationOrder.Default)
        {
            return Euler(float3(x, y, z), order);
        }

        /// <summary>Returns a float4x4 matrix that rotates around the x-axis by a given number of radians.</summary>
        /// <param name="angle">The clockwise rotation angle when looking along the x-axis towards the origin in radians.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 RotateX(sfloat angle)
        {
            // {{1, 0, 0}, {0, c_0, -s_0}, {0, s_0, c_0}}
            sfloat s, c;
            sincos(angle, out s, out c);
            return float4x4(sfloat.One, sfloat.Zero, sfloat.Zero, sfloat.Zero,
                            sfloat.Zero, c, -s, sfloat.Zero,
                            sfloat.Zero, s, c, sfloat.Zero,
                            sfloat.Zero, sfloat.Zero, sfloat.Zero, sfloat.One);

        }

        /// <summary>Returns a float4x4 matrix that rotates around the y-axis by a given number of radians.</summary>
        /// <param name="angle">The clockwise rotation angle when looking along the y-axis towards the origin in radians.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 RotateY(sfloat angle)
        {
            // {{c_1, 0, s_1}, {0, 1, 0}, {-s_1, 0, c_1}}
            sfloat s, c;
            sincos(angle, out s, out c);
            return float4x4(c, sfloat.Zero, s, sfloat.Zero,
                            sfloat.Zero, sfloat.One, sfloat.Zero, sfloat.Zero,
                            -s, sfloat.Zero, c, sfloat.Zero,
                            sfloat.Zero, sfloat.Zero, sfloat.Zero, sfloat.One);

        }

        /// <summary>Returns a float4x4 matrix that rotates around the z-axis by a given number of radians.</summary>
        /// <param name="angle">The clockwise rotation angle when looking along the z-axis towards the origin in radians.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 RotateZ(sfloat angle)
        {
            // {{c_2, -s_2, 0}, {s_2, c_2, 0}, {0, 0, 1}}
            sfloat s, c;
            sincos(angle, out s, out c);
            return float4x4(c, -s, sfloat.Zero, sfloat.Zero,
                            s, c, sfloat.Zero, sfloat.Zero,
                            sfloat.Zero, sfloat.Zero, sfloat.One, sfloat.Zero,
                            sfloat.Zero, sfloat.Zero, sfloat.Zero, sfloat.One);

        }

        /// <summary>Returns a float4x4 scale matrix given 3 axis scales.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 Scale(sfloat s)
        {
            return float4x4(s, sfloat.Zero, sfloat.Zero, sfloat.Zero,
                            sfloat.Zero, s, sfloat.Zero, sfloat.Zero,
                            sfloat.Zero, sfloat.Zero, s, sfloat.Zero,
                            sfloat.Zero, sfloat.Zero, sfloat.Zero, sfloat.One);
        }

        /// <summary>Returns a float4x4 scale matrix given a float3 vector containing the 3 axis scales.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 Scale(sfloat x, sfloat y, sfloat z)
        {
            return float4x4(x, sfloat.Zero, sfloat.Zero, sfloat.Zero,
                            sfloat.Zero, y, sfloat.Zero, sfloat.Zero,
                            sfloat.Zero, sfloat.Zero, z, sfloat.Zero,
                            sfloat.Zero, sfloat.Zero, sfloat.Zero, sfloat.One);
        }

        /// <summary>Returns a float4x4 scale matrix given a float3 vector containing the 3 axis scales.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 Scale(float3 scales)
        {
            return Scale(scales.x, scales.y, scales.z);
        }

        /// <summary>Returns a float4x4 translation matrix given a float3 translation vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 Translate(float3 vector)
        {
            return float4x4(float4(sfloat.One, sfloat.Zero, sfloat.Zero, sfloat.Zero),
                            float4(sfloat.Zero, sfloat.One, sfloat.Zero, sfloat.Zero),
                            float4(sfloat.Zero, sfloat.Zero, sfloat.One, sfloat.Zero),
                            float4(vector.x, vector.y, vector.z, sfloat.One));
        }

        /// <summary>
        /// Returns a float4x4 view matrix given an eye position, a target point and a unit length up vector.
        /// The up vector is assumed to be unit length, the eye and target points are assumed to be distinct and
        /// the vector between them is assumes to be collinear with the up vector.
        /// If these assumptions are not met use float4x4.LookRotationSafe instead.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 LookAt(float3 eye, float3 target, float3 up)
        {
            float3x3 rot = float3x3.LookRotation(normalize(target - eye), up);

            float4x4 matrix;
            matrix.c0 = float4(rot.c0, sfloat.Zero);
            matrix.c1 = float4(rot.c1, sfloat.Zero);
            matrix.c2 = float4(rot.c2, sfloat.Zero);
            matrix.c3 = float4(eye, sfloat.One);
            return matrix;
        }

        /// <summary>
        /// Returns a float4x4 centered orthographic projection matrix.
        /// </summary>
        /// <param name="width">The width of the view volume.</param>
        /// <param name="height">The height of the view volume.</param>
        /// <param name="near">The distance to the near plane.</param>
        /// <param name="far">The distance to the far plane.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 Ortho(sfloat width, sfloat height, sfloat near, sfloat far)
        {
            sfloat rcpdx = sfloat.One / width;
            sfloat rcpdy = sfloat.One / height;
            sfloat rcpdz = sfloat.One / (far - near);

            return float4x4(
                (sfloat)2.0f * rcpdx, sfloat.Zero, sfloat.Zero, sfloat.Zero,
                sfloat.Zero, (sfloat)2.0f * rcpdy, sfloat.Zero, sfloat.Zero,
                sfloat.Zero, sfloat.Zero, (sfloat)(-2.0f) * rcpdz, -(far + near) * rcpdz,
                sfloat.Zero, sfloat.Zero, sfloat.Zero, sfloat.One
                );
        }

        /// <summary>
        /// Returns a float4x4 off-center orthographic projection matrix.
        /// </summary>
        /// <param name="left">The minimum x-coordinate of the view volume.</param>
        /// <param name="right">The maximum x-coordinate of the view volume.</param>
        /// <param name="bottom">The minimum y-coordinate of the view volume.</param>
        /// <param name="top">The minimum y-coordinate of the view volume.</param>
        /// <param name="near">The distance to the near plane.</param>
        /// <param name="far">The distance to the far plane.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 OrthoOffCenter(sfloat left, sfloat right, sfloat bottom, sfloat top, sfloat near, sfloat far)
        {
            sfloat rcpdx = sfloat.One / (right - left);
            sfloat rcpdy = sfloat.One / (top - bottom);
            sfloat rcpdz = sfloat.One / (far - near);

            return float4x4(
                (sfloat)2.0f * rcpdx, sfloat.Zero, sfloat.Zero, -(right + left) * rcpdx,
                sfloat.Zero, (sfloat)2.0f * rcpdy, sfloat.Zero, -(top + bottom) * rcpdy,
                sfloat.Zero, sfloat.Zero, (sfloat)(-2.0f) * rcpdz, -(far + near) * rcpdz,
                sfloat.Zero, sfloat.Zero, sfloat.Zero, sfloat.One
                );
        }

        /// <summary>
        /// Returns a float4x4 perspective projection matrix based on field of view.
        /// </summary>
        /// <param name="verticalFov">Vertical Field of view in radians.</param>
        /// <param name="aspect">X:Y aspect ratio.</param>
        /// <param name="near">Distance to near plane. Must be greater than zero.</param>
        /// <param name="far">Distance to far plane. Must be greater than zero.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 PerspectiveFov(sfloat verticalFov, sfloat aspect, sfloat near, sfloat far)
        {
            sfloat cotangent = sfloat.One / tan(verticalFov * (sfloat)0.5f);
            sfloat rcpdz = sfloat.One / (near - far);

            return float4x4(
                cotangent / aspect, sfloat.Zero, sfloat.Zero, sfloat.Zero,
                sfloat.Zero, cotangent, sfloat.Zero, sfloat.Zero,
                sfloat.Zero, sfloat.Zero, (far + near) * rcpdz, (sfloat)2.0f * near * far * rcpdz,
                sfloat.Zero, sfloat.Zero, -sfloat.One, sfloat.Zero
                );
        }

        /// <summary>
        /// Returns a float4x4 off-center perspective projection matrix.
        /// </summary>
        /// <param name="left">The x-coordinate of the left side of the clipping frustum at the near plane.</param>
        /// <param name="right">The x-coordinate of the right side of the clipping frustum at the near plane.</param>
        /// <param name="bottom">The y-coordinate of the bottom side of the clipping frustum at the near plane.</param>
        /// <param name="top">The y-coordinate of the top side of the clipping frustum at the near plane.</param>
        /// <param name="near">Distance to the near plane. Must be greater than zero.</param>
        /// <param name="far">Distance to the far plane. Must be greater than zero.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 PerspectiveOffCenter(sfloat left, sfloat right, sfloat bottom, sfloat top, sfloat near, sfloat far)
        {
            sfloat rcpdz = sfloat.One / (near - far);
            sfloat rcpWidth = sfloat.One / (right - left);
            sfloat rcpHeight = sfloat.One / (top - bottom);

            return float4x4(
                (sfloat)2.0f * near * rcpWidth, sfloat.Zero, (left + right) * rcpWidth, sfloat.Zero,
                sfloat.Zero, (sfloat)2.0f * near * rcpHeight, (bottom + top) * rcpHeight, sfloat.Zero,
                sfloat.Zero, sfloat.Zero, (far + near) * rcpdz, (sfloat)2.0f * near * far * rcpdz,
                sfloat.Zero, sfloat.Zero, -sfloat.One, sfloat.Zero
                );
        }

        /// <summary>
        /// Returns a float4x4 matrix representing a combined scale-, rotation- and translation transform.
        /// Equivalent to mul(translationTransform, mul(rotationTransform, scaleTransform)).
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 TRS(float3 translation, quaternion rotation, float3 scale)
        {
            float3x3 r = float3x3(rotation);
            return float4x4(  float4(r.c0 * scale.x, sfloat.Zero),
                              float4(r.c1 * scale.y, sfloat.Zero),
                              float4(r.c2 * scale.z, sfloat.Zero),
                              float4(translation, sfloat.One));
        }
    }

    partial class math
    {
        /// <summary>
        /// Extracts a float3x3 from the upper left 3x3 of a float4x4.
        /// </summary>
        /// <param name="f4x4"><see cref="float4x4"/> to extract a float3x3 from.</param>
        /// <returns>Upper left 3x3 matrix as float3x3.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x3 float3x3(float4x4 f4x4)
        {
            return new float3x3(f4x4);
        }

        /// <summary>Returns a float3x3 matrix constructed from a quaternion.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x3 float3x3(quaternion rotation)
        {
            return new float3x3(rotation);
        }

        /// <summary>Returns a float4x4 constructed from a float3x3 rotation matrix and a float3 translation vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 float4x4(float3x3 rotation, float3 translation)
        {
            return new float4x4(rotation, translation);
        }

        /// <summary>Returns a float4x4 constructed from a quaternion and a float3 translation vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 float4x4(quaternion rotation, float3 translation)
        {
            return new float4x4(rotation, translation);
        }

        /// <summary>Returns a float4x4 constructed from a RigidTransform.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 float4x4(RigidTransform transform)
        {
            return new float4x4(transform);
        }

        /// <summary>Returns an orthonormalized version of a float3x3 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x3 orthonormalize(float3x3 i)
        {
            float3x3 o;

            float3 u = i.c0;
            float3 v = i.c1 - i.c0 * math.dot(i.c1, i.c0);

            sfloat lenU = math.length(u);
            sfloat lenV = math.length(v);

            const uint smallValue = 0x0da24260;
            sfloat epsilon = sfloat.FromRaw(smallValue);

            bool c = lenU > epsilon && lenV > epsilon;

            o.c0 = math.select(float3(sfloat.One, sfloat.Zero, sfloat.Zero), u / lenU, c);
            o.c1 = math.select(float3(sfloat.Zero, sfloat.One, sfloat.Zero), v / lenV, c);
            o.c2 = math.cross(o.c0, o.c1);

            return o;
        }
    }
}
