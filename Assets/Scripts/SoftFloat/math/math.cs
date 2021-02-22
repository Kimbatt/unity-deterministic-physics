using System;
using System.Runtime.InteropServices;
using System.Runtime.CompilerServices;

namespace UnityS.Mathematics
{
    public static partial class math
    {
        /// <summary>Extrinsic rotation order. Specifies in which order rotations around the principal axes (x, y and z) are to be applied.</summary>
        public enum RotationOrder : byte
        {
            /// <summary>Extrinsic rotation around the x axis, then around the y axis and finally around the z axis.</summary>
            XYZ,
            /// <summary>Extrinsic rotation around the x axis, then around the z axis and finally around the y axis.</summary>
            XZY,
            /// <summary>Extrinsic rotation around the y axis, then around the x axis and finally around the z axis.</summary>
            YXZ,
            /// <summary>Extrinsic rotation around the y axis, then around the z axis and finally around the x axis.</summary>
            YZX,
            /// <summary>Extrinsic rotation around the z axis, then around the x axis and finally around the y axis.</summary>
            ZXY,
            /// <summary>Extrinsic rotation around the z axis, then around the y axis and finally around the x axis.</summary>
            ZYX,
            /// <summary>Unity default rotation order. Extrinsic Rotation around the z axis, then around the x axis and finally around the y axis.</summary>
            Default = ZXY
        };

        /// <summary>Specifies a shuffle component.</summary>
        public enum ShuffleComponent : byte
        {
            /// <summary>Specified the x component of the left vector.</summary>
            LeftX,
            /// <summary>Specified the y component of the left vector.</summary>
            LeftY,
            /// <summary>Specified the z component of the left vector.</summary>
            LeftZ,
            /// <summary>Specified the w component of the left vector.</summary>
            LeftW,

            /// <summary>Specified the x component of the right vector.</summary>
            RightX,
            /// <summary>Specified the y component of the right vector.</summary>
            RightY,
            /// <summary>Specified the z component of the right vector.</summary>
            RightZ,
            /// <summary>Specified the w component of the right vector.</summary>
            RightW
        };

        /// <summary>The smallest positive normal number representable in a float.</summary>
        public static sfloat FLT_MIN_NORMAL => sfloat.FromRaw(0x00800000);

        /// <summary>The mathematical constant e also known as Euler's number. Approximately 2.72.</summary>
        public static sfloat E => sfloat.FromRaw(0x402df854);

        /// <summary>The base 2 logarithm of e. Approximately 1.44.</summary>
        public static sfloat LOG2E => sfloat.FromRaw(0x3fb8aa3b);

        /// <summary>The base 10 logarithm of e. Approximately 0.43.</summary>
        public static sfloat LOG10E => sfloat.FromRaw(0x3ede5bd9);

        /// <summary>The natural logarithm of 2. Approximately 0.69.</summary>
        public static sfloat LN2 => sfloat.FromRaw(0x3f317218);

        /// <summary>The natural logarithm of 10. Approximately 2.30.</summary>
        public static sfloat LN10 => sfloat.FromRaw(0x40135d8e);

        /// <summary>The mathematical constant pi. Approximately 3.14.</summary>
        public static sfloat PI => sfloat.FromRaw(0x40490fdb);

        /// <summary>pi / 2. Approximately 1.57.</summary>
        public static sfloat PI_HALF => sfloat.FromRaw(0x3fc90fdb);

        /// <summary>pi / 4. Approximately 0.79.</summary>
        public static sfloat PI_OVER_4 => sfloat.FromRaw(0x3f490fdb);

        /// <summary>pi * 2. Approximately 6.28.</summary>
        public static sfloat TWO_PI => sfloat.FromRaw(0x40c90fdb);

        /// <summary>The square root 2. Approximately 1.41.</summary>
        public static sfloat SQRT2 => sfloat.FromRaw(0x3fb504f3);

        /// <summary>
        /// The difference between 1.0f and the next representable f32/single precision number.
        ///
        /// Beware:
        /// This value is different from System.Single.Epsilon, which is the smallest, positive, denormalized f32/single.
        /// </summary>
        public static sfloat EPSILON => sfloat.FromRaw(0x34000000);

        /// <summary>
        /// Single precision constant for positive infinity.
        /// </summary>
        public static sfloat INFINITY => sfloat.PositiveInfinity;

        /// <summary>
        /// Single precision constant for Not a Number.
        ///
        /// NAN is considered unordered, which means all comparisons involving it are false except for not equal (operator !=).
        /// As a consequence, NAN == NAN is false but NAN != NAN is true.
        ///
        /// Additionally, there are multiple bit representations for Not a Number, so if you must test if your value
        /// is NAN, use isnan().
        /// </summary>
        public static sfloat NAN => sfloat.NaN;

        /// <summary>Returns the bit pattern of a uint as an int.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int asint(uint x) { return (int)x; }

        /// <summary>Returns the bit pattern of a uint2 as an int2.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int2 asint(uint2 x) { return int2((int)x.x, (int)x.y); }

        /// <summary>Returns the bit pattern of a uint3 as an int3.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int3 asint(uint3 x) { return int3((int)x.x, (int)x.y, (int)x.z); }

        /// <summary>Returns the bit pattern of a uint4 as an int4.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int4 asint(uint4 x) { return int4((int)x.x, (int)x.y, (int)x.z, (int)x.w); }


        /// <summary>Returns the bit pattern of a float as an int.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int asint(sfloat x) {
            return (int)x.RawValue;
        }

        /// <summary>Returns the bit pattern of a float2 as an int2.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int2 asint(float2 x) { return int2(asint(x.x), asint(x.y)); }

        /// <summary>Returns the bit pattern of a float3 as an int3.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int3 asint(float3 x) { return int3(asint(x.x), asint(x.y), asint(x.z)); }

        /// <summary>Returns the bit pattern of a float4 as an int4.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int4 asint(float4 x) { return int4(asint(x.x), asint(x.y), asint(x.z), asint(x.w)); }


        /// <summary>Returns the bit pattern of an int as a uint.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint asuint(int x) { return (uint)x; }

        /// <summary>Returns the bit pattern of an int2 as a uint2.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint2 asuint(int2 x) { return uint2((uint)x.x, (uint)x.y); }

        /// <summary>Returns the bit pattern of an int3 as a uint3.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint3 asuint(int3 x) { return uint3((uint)x.x, (uint)x.y, (uint)x.z); }

        /// <summary>Returns the bit pattern of an int4 as a uint4.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint4 asuint(int4 x) { return uint4((uint)x.x, (uint)x.y, (uint)x.z, (uint)x.w); }


        /// <summary>Returns the bit pattern of a float as a uint.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint asuint(sfloat x) { return (uint)asint(x); }

        /// <summary>Returns the bit pattern of a float2 as a uint2.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint2 asuint(float2 x) { return uint2(asuint(x.x), asuint(x.y)); }

        /// <summary>Returns the bit pattern of a float3 as a uint3.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint3 asuint(float3 x) { return uint3(asuint(x.x), asuint(x.y), asuint(x.z)); }

        /// <summary>Returns the bit pattern of a float4 as a uint4.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint4 asuint(float4 x) { return uint4(asuint(x.x), asuint(x.y), asuint(x.z), asuint(x.w)); }


        /// <summary>Returns the bit pattern of a ulong as a long.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static long aslong(ulong x) { return (long)x; }


        /// <summary>Returns the bit pattern of a long as a ulong.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ulong asulong(long x) { return (ulong)x; }


        /// <summary>Returns the bit pattern of an int as a float.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat asfloat(int x)
        {
            return sfloat.FromRaw((uint)x);
        }

        /// <summary>Returns the bit pattern of an int2 as a float2.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 asfloat(int2 x) { return float2(asfloat(x.x), asfloat(x.y)); }

        /// <summary>Returns the bit pattern of an int3 as a float3.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 asfloat(int3 x) { return float3(asfloat(x.x), asfloat(x.y), asfloat(x.z)); }

        /// <summary>Returns the bit pattern of an int4 as a float4.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 asfloat(int4 x) { return float4(asfloat(x.x), asfloat(x.y), asfloat(x.z), asfloat(x.w)); }


        /// <summary>Returns the bit pattern of a uint as a float.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat asfloat(uint x) { return asfloat((int)x); }

        /// <summary>Returns the bit pattern of a uint2 as a float2.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 asfloat(uint2 x) { return float2(asfloat(x.x), asfloat(x.y)); }

        /// <summary>Returns the bit pattern of a uint3 as a float3.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 asfloat(uint3 x) { return float3(asfloat(x.x), asfloat(x.y), asfloat(x.z)); }

        /// <summary>Returns the bit pattern of a uint4 as a float4.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 asfloat(uint4 x) { return float4(asfloat(x.x), asfloat(x.y), asfloat(x.z), asfloat(x.w)); }

        /// <summary>
        /// Returns a bitmask representation of a bool4. Storing one 1 bit per component
        /// in LSB order, from lower to higher bits (so 4 bits in total).
        /// The component x is stored at bit 0,
        /// The component y is stored at bit 1,
        /// The component z is stored at bit 2,
        /// The component w is stored at bit 3
        /// The bool4(x = true, y = true, z = false, w = true) would produce the value 1011 = 0xB
        /// </summary>
        /// <param name="value">The input bool4 to calculate the bitmask for</param>
        /// <returns>A bitmask representation of the bool4, in LSB order</returns>
        public static int bitmask(bool4 value)
        {
            int mask = 0;
            if (value.x) mask |= 0x01;
            if (value.y) mask |= 0x02;
            if (value.z) mask |= 0x04;
            if (value.w) mask |= 0x08;
            return mask;
        }


        /// <summary>Returns true if the input float is a finite floating point value, false otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool isfinite(sfloat x) { return x.IsFinite(); }

        /// <summary>Returns a bool2 indicating for each component of a float2 whether it is a finite floating point value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool2 isfinite(float2 x) { return new bool2(isfinite(x.x), isfinite(x.y)); }

        /// <summary>Returns a bool3 indicating for each component of a float3 whether it is a finite floating point value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool3 isfinite(float3 x) { return new bool3(isfinite(x.x), isfinite(x.y), isfinite(x.z)); }

        /// <summary>Returns a bool4 indicating for each component of a float4 whether it is a finite floating point value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4 isfinite(float4 x) { return new bool4(isfinite(x.x), isfinite(x.y), isfinite(x.z), isfinite(x.w)); }


        /// <summary>Returns true if the input float is an infinite floating point value, false otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool isinf(sfloat x) { return !x.IsFinite(); }

        /// <summary>Returns a bool2 indicating for each component of a float2 whether it is an infinite floating point value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool2 isinf(float2 x) { return new bool2(isinf(x.x), isinf(x.y)); }

        /// <summary>Returns a bool3 indicating for each component of a float3 whether it is an infinite floating point value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool3 isinf(float3 x) { return new bool3(isinf(x.x), isinf(x.y), isinf(x.z)); }

        /// <summary>Returns a bool4 indicating for each component of a float4 whether it is an infinite floating point value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4 isinf(float4 x) { return new bool4(isinf(x.x), isinf(x.y), isinf(x.z), isinf(x.w)); }


        /// <summary>Returns true if the input float is a NaN (not a number) floating point value, false otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool isnan(sfloat x) { return (asuint(x) & 0x7FFFFFFF) > 0x7F800000; }

        /// <summary>Returns a bool2 indicating for each component of a float2 whether it is a NaN (not a number) floating point value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool2 isnan(float2 x) { return (asuint(x) & 0x7FFFFFFF) > 0x7F800000; }

        /// <summary>Returns a bool3 indicating for each component of a float3 whether it is a NaN (not a number) floating point value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool3 isnan(float3 x) { return (asuint(x) & 0x7FFFFFFF) > 0x7F800000; }

        /// <summary>Returns a bool4 indicating for each component of a float4 whether it is a NaN (not a number) floating point value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4 isnan(float4 x) { return (asuint(x) & 0x7FFFFFFF) > 0x7F800000; }


        /// <summary>
        /// Checks if the input is a power of two.
        /// </summary>
        /// <remarks>If x is less than or equal to zero, then this function returns false.</remarks>
        /// <param name="x">Integer input.</param>
        /// <returns>bool where true indicates that input was a power of two.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool ispow2(int x)
        {
            return x > 0 && ((x & (x - 1)) == 0);
        }

        /// <summary>
        /// Checks if each component of the input is a power of two.
        /// </summary>
        /// <remarks>If a component of x is less than or equal to zero, then this function returns false in that component.</remarks>
        /// <param name="x"><see cref="int2"/> input</param>
        /// <returns><see cref="bool2"/> where true in a component indicates the same component in the input was a power of two.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool2 ispow2(int2 x)
        {
            return new bool2(ispow2(x.x), ispow2(x.y));
        }

        /// <summary>
        /// Checks if each component of the input is a power of two.
        /// </summary>
        /// <remarks>If a component of x is less than or equal to zero, then this function returns false in that component.</remarks>
        /// <param name="x"><see cref="int3"/> input</param>
        /// <returns><see cref="bool3"/> where true in a component indicates the same component in the input was a power of two.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool3 ispow2(int3 x)
        {
            return new bool3(ispow2(x.x), ispow2(x.y), ispow2(x.z));
        }

        /// <summary>
        /// Checks if each component of the input is a power of two.
        /// </summary>
        /// <remarks>If a component of x is less than or equal to zero, then this function returns false in that component.</remarks>
        /// <param name="x"><see cref="int4"/> input</param>
        /// <returns><see cref="bool4"/> where true in a component indicates the same component in the input was a power of two.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4 ispow2(int4 x)
        {
            return new bool4(ispow2(x.x), ispow2(x.y), ispow2(x.z), ispow2(x.w));
        }

        /// <summary>
        /// Checks if the input is a power of two.
        /// </summary>
        /// <remarks>If x is less than or equal to zero, then this function returns false.</remarks>
        /// <param name="x">Unsigned integer input.</param>
        /// <returns>bool where true indicates that input was a power of two.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool ispow2(uint x)
        {
            return x > 0 && ((x & (x - 1)) == 0);
        }

        /// <summary>
        /// Checks if each component of the input is a power of two.
        /// </summary>
        /// <remarks>If a component of x is less than or equal to zero, then this function returns false in that component.</remarks>
        /// <param name="x"><see cref="uint2"/> input</param>
        /// <returns><see cref="bool2"/> where true in a component indicates the same component in the input was a power of two.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool2 ispow2(uint2 x)
        {
            return new bool2(ispow2(x.x), ispow2(x.y));
        }

        /// <summary>
        /// Checks if each component of the input is a power of two.
        /// </summary>
        /// <remarks>If a component of x is less than or equal to zero, then this function returns false in that component.</remarks>
        /// <param name="x"><see cref="uint3"/> input</param>
        /// <returns><see cref="bool3"/> where true in a component indicates the same component in the input was a power of two.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool3 ispow2(uint3 x)
        {
            return new bool3(ispow2(x.x), ispow2(x.y), ispow2(x.z));
        }

        /// <summary>
        /// Checks if each component of the input is a power of two.
        /// </summary>
        /// <remarks>If a component of x is less than or equal to zero, then this function returns false in that component.</remarks>
        /// <param name="x"><see cref="uint4"/> input</param>
        /// <returns><see cref="bool4"/> where true in a component indicates the same component in the input was a power of two.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4 ispow2(uint4 x)
        {
            return new bool4(ispow2(x.x), ispow2(x.y), ispow2(x.z), ispow2(x.w));
        }

        /// <summary>Returns the minimum of two int values.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int min(int x, int y) { return x < y ? x : y; }

        /// <summary>Returns the componentwise minimum of two int2 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int2 min(int2 x, int2 y) { return new int2(min(x.x, y.x), min(x.y, y.y)); }

        /// <summary>Returns the componentwise minimum of two int3 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int3 min(int3 x, int3 y) { return new int3(min(x.x, y.x), min(x.y, y.y), min(x.z, y.z)); }

        /// <summary>Returns the componentwise minimum of two int4 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int4 min(int4 x, int4 y) { return new int4(min(x.x, y.x), min(x.y, y.y), min(x.z, y.z), min(x.w, y.w)); }


        /// <summary>Returns the minimum of two uint values.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint min(uint x, uint y) { return x < y ? x : y; }

        /// <summary>Returns the componentwise minimum of two uint2 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint2 min(uint2 x, uint2 y) { return new uint2(min(x.x, y.x), min(x.y, y.y)); }

        /// <summary>Returns the componentwise minimum of two uint3 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint3 min(uint3 x, uint3 y) { return new uint3(min(x.x, y.x), min(x.y, y.y), min(x.z, y.z)); }

        /// <summary>Returns the componentwise minimum of two uint4 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint4 min(uint4 x, uint4 y) { return new uint4(min(x.x, y.x), min(x.y, y.y), min(x.z, y.z), min(x.w, y.w)); }


        /// <summary>Returns the minimum of two long values.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static long min(long x, long y) { return x < y ? x : y; }


        /// <summary>Returns the minimum of two ulong values.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ulong min(ulong x, ulong y) { return x < y ? x : y; }


        /// <summary>Returns the minimum of two float values.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat min(sfloat x, sfloat y) { return y.IsNaN() || x < y ? x : y; }

        /// <summary>Returns the componentwise minimum of two float2 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 min(float2 x, float2 y) { return new float2(min(x.x, y.x), min(x.y, y.y)); }

        /// <summary>Returns the componentwise minimum of two float3 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 min(float3 x, float3 y) { return new float3(min(x.x, y.x), min(x.y, y.y), min(x.z, y.z)); }

        /// <summary>Returns the componentwise minimum of two float4 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 min(float4 x, float4 y) { return new float4(min(x.x, y.x), min(x.y, y.y), min(x.z, y.z), min(x.w, y.w)); }


        /// <summary>Returns the maximum of two int values.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int max(int x, int y) { return x > y ? x : y; }

        /// <summary>Returns the componentwise maximum of two int2 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int2 max(int2 x, int2 y) { return new int2(max(x.x, y.x), max(x.y, y.y)); }

        /// <summary>Returns the componentwise maximum of two int3 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int3 max(int3 x, int3 y) { return new int3(max(x.x, y.x), max(x.y, y.y), max(x.z, y.z)); }

        /// <summary>Returns the componentwise maximum of two int4 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int4 max(int4 x, int4 y) { return new int4(max(x.x, y.x), max(x.y, y.y), max(x.z, y.z), max(x.w, y.w)); }


        /// <summary>Returns the maximum of two uint values.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint max(uint x, uint y) { return x > y ? x : y; }

        /// <summary>Returns the componentwise maximum of two uint2 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint2 max(uint2 x, uint2 y) { return new uint2(max(x.x, y.x), max(x.y, y.y)); }

        /// <summary>Returns the componentwise maximum of two uint3 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint3 max(uint3 x, uint3 y) { return new uint3(max(x.x, y.x), max(x.y, y.y), max(x.z, y.z)); }

        /// <summary>Returns the componentwise maximum of two uint4 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint4 max(uint4 x, uint4 y) { return new uint4(max(x.x, y.x), max(x.y, y.y), max(x.z, y.z), max(x.w, y.w)); }


        /// <summary>Returns the maximum of two long values.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static long max(long x, long y) { return x > y ? x : y; }


        /// <summary>Returns the maximum of two ulong values.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ulong max(ulong x, ulong y) { return x > y ? x : y; }


        /// <summary>Returns the maximum of two float values.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat max(sfloat x, sfloat y) { return y.IsNaN() || x > y ? x : y; }

        /// <summary>Returns the componentwise maximum of two float2 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 max(float2 x, float2 y) { return new float2(max(x.x, y.x), max(x.y, y.y)); }

        /// <summary>Returns the componentwise maximum of two float3 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 max(float3 x, float3 y) { return new float3(max(x.x, y.x), max(x.y, y.y), max(x.z, y.z)); }

        /// <summary>Returns the componentwise maximum of two float4 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 max(float4 x, float4 y) { return new float4(max(x.x, y.x), max(x.y, y.y), max(x.z, y.z), max(x.w, y.w)); }



        /// <summary>Returns the result of linearly interpolating from x to y using the interpolation parameter s.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat lerp(sfloat x, sfloat y, sfloat s) { return x + s * (y - x); }

        /// <summary>Returns the result of a componentwise linear interpolating from x to y using the interpolation parameter s.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 lerp(float2 x, float2 y, sfloat s) { return x + s * (y - x); }

        /// <summary>Returns the result of a componentwise linear interpolating from x to y using the interpolation parameter s.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 lerp(float3 x, float3 y, sfloat s) { return x + s * (y - x); }

        /// <summary>Returns the result of a componentwise linear interpolating from x to y using the interpolation parameter s.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 lerp(float4 x, float4 y, sfloat s) { return x + s * (y - x); }


        /// <summary>Returns the result of a componentwise linear interpolating from x to y using the corresponding components of the interpolation parameter s.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 lerp(float2 x, float2 y, float2 s) { return x + s * (y - x); }

        /// <summary>Returns the result of a componentwise linear interpolating from x to y using the corresponding components of the interpolation parameter s.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 lerp(float3 x, float3 y, float3 s) { return x + s * (y - x); }

        /// <summary>Returns the result of a componentwise linear interpolating from x to y using the corresponding components of the interpolation parameter s.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 lerp(float4 x, float4 y, float4 s) { return x + s * (y - x); }


        /// <summary>Returns the result of normalizing a floating point value x to a range [a, b]. The opposite of lerp. Equivalent to (x - a) / (b - a).</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat unlerp(sfloat a, sfloat b, sfloat x) { return (x - a) / (b - a); }

        /// <summary>Returns the componentwise result of normalizing a floating point value x to a range [a, b]. The opposite of lerp. Equivalent to (x - a) / (b - a).</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 unlerp(float2 a, float2 b, float2 x) { return (x - a) / (b - a); }

        /// <summary>Returns the componentwise result of normalizing a floating point value x to a range [a, b]. The opposite of lerp. Equivalent to (x - a) / (b - a).</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 unlerp(float3 a, float3 b, float3 x) { return (x - a) / (b - a); }

        /// <summary>Returns the componentwise result of normalizing a floating point value x to a range [a, b]. The opposite of lerp. Equivalent to (x - a) / (b - a).</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 unlerp(float4 a, float4 b, float4 x) { return (x - a) / (b - a); }


        /// <summary>Returns the result of a non-clamping linear remapping of a value x from [a, b] to [c, d].</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat remap(sfloat a, sfloat b, sfloat c, sfloat d, sfloat x) { return lerp(c, d, unlerp(a, b, x)); }

        /// <summary>Returns the componentwise result of a non-clamping linear remapping of a value x from [a, b] to [c, d].</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 remap(float2 a, float2 b, float2 c, float2 d, float2 x) { return lerp(c, d, unlerp(a, b, x)); }

        /// <summary>Returns the componentwise result of a non-clamping linear remapping of a value x from [a, b] to [c, d].</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 remap(float3 a, float3 b, float3 c, float3 d, float3 x) { return lerp(c, d, unlerp(a, b, x)); }

        /// <summary>Returns the componentwise result of a non-clamping linear remapping of a value x from [a, b] to [c, d].</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 remap(float4 a, float4 b, float4 c, float4 d, float4 x) { return lerp(c, d, unlerp(a, b, x)); }


        /// <summary>Returns the result of a multiply-add operation (a * b + c) on 3 int values.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int mad(int a, int b, int c) { return a * b + c; }

        /// <summary>Returns the result of a componentwise multiply-add operation (a * b + c) on 3 int2 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int2 mad(int2 a, int2 b, int2 c) { return a * b + c; }

        /// <summary>Returns the result of a componentwise multiply-add operation (a * b + c) on 3 int3 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int3 mad(int3 a, int3 b, int3 c) { return a * b + c; }

        /// <summary>Returns the result of a componentwise multiply-add operation (a * b + c) on 3 int4 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int4 mad(int4 a, int4 b, int4 c) { return a * b + c; }


        /// <summary>Returns the result of a multiply-add operation (a * b + c) on 3 uint values.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint mad(uint a, uint b, uint c) { return a * b + c; }

        /// <summary>Returns the result of a componentwise multiply-add operation (a * b + c) on 3 uint2 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint2 mad(uint2 a, uint2 b, uint2 c) { return a * b + c; }

        /// <summary>Returns the result of a componentwise multiply-add operation (a * b + c) on 3 uint3 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint3 mad(uint3 a, uint3 b, uint3 c) { return a * b + c; }

        /// <summary>Returns the result of a componentwise multiply-add operation (a * b + c) on 3 uint4 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint4 mad(uint4 a, uint4 b, uint4 c) { return a * b + c; }


        /// <summary>Returns the result of a multiply-add operation (a * b + c) on 3 long values.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static long mad(long a, long b, long c) { return a * b + c; }


        /// <summary>Returns the result of a multiply-add operation (a * b + c) on 3 ulong values.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ulong mad(ulong a, ulong b, ulong c) { return a * b + c; }


        /// <summary>Returns the result of a multiply-add operation (a * b + c) on 3 float values.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float mad(float a, float b, float c) { return a * b + c; }

        /// <summary>Returns the result of a componentwise multiply-add operation (a * b + c) on 3 float2 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 mad(float2 a, float2 b, float2 c) { return a * b + c; }

        /// <summary>Returns the result of a componentwise multiply-add operation (a * b + c) on 3 float3 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 mad(float3 a, float3 b, float3 c) { return a * b + c; }

        /// <summary>Returns the result of a componentwise multiply-add operation (a * b + c) on 3 float4 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 mad(float4 a, float4 b, float4 c) { return a * b + c; }


        /// <summary>Returns the result of clamping the value x into the interval [a, b], where x, a and b are int values.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int clamp(int x, int a, int b) { return max(a, min(b, x)); }

        /// <summary>Returns the result of a componentwise clamping of the int2 x into the interval [a, b], where a and b are int2 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int2 clamp(int2 x, int2 a, int2 b) { return max(a, min(b, x)); }

        /// <summary>Returns the result of a componentwise clamping of the int3 x into the interval [a, b], where x, a and b are int3 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int3 clamp(int3 x, int3 a, int3 b) { return max(a, min(b, x)); }

        /// <summary>Returns the result of a componentwise clamping of the value x into the interval [a, b], where x, a and b are int4 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int4 clamp(int4 x, int4 a, int4 b) { return max(a, min(b, x)); }


        /// <summary>Returns the result of clamping the value x into the interval [a, b], where x, a and b are uint values.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint clamp(uint x, uint a, uint b) { return max(a, min(b, x)); }

        /// <summary>Returns the result of a componentwise clamping of the value x into the interval [a, b], where x, a and b are uint2 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint2 clamp(uint2 x, uint2 a, uint2 b) { return max(a, min(b, x)); }

        /// <summary>Returns the result of a componentwise clamping of the value x into the interval [a, b], where x, a and b are uint3 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint3 clamp(uint3 x, uint3 a, uint3 b) { return max(a, min(b, x)); }

        /// <summary>Returns the result of a componentwise clamping of the value x into the interval [a, b], where x, a and b are uint4 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint4 clamp(uint4 x, uint4 a, uint4 b) { return max(a, min(b, x)); }


        /// <summary>Returns the result of clamping the value x into the interval [a, b], where x, a and b are long values.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static long clamp(long x, long a, long b) { return max(a, min(b, x)); }

        /// <summary>Returns the result of clamping the value x into the interval [a, b], where x, a and b are ulong values.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ulong clamp(ulong x, ulong a, ulong b) { return max(a, min(b, x)); }


        /// <summary>Returns the result of clamping the value x into the interval [a, b], where x, a and b are float values.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat clamp(sfloat x, sfloat a, sfloat b) { return max(a, min(b, x)); }

        /// <summary>Returns the result of a componentwise clamping of the value x into the interval [a, b], where x, a and b are float2 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 clamp(float2 x, float2 a, float2 b) { return max(a, min(b, x)); }

        /// <summary>Returns the result of a componentwise clamping of the value x into the interval [a, b], where x, a and b are float3 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 clamp(float3 x, float3 a, float3 b) { return max(a, min(b, x)); }

        /// <summary>Returns the result of a componentwise clamping of the value x into the interval [a, b], where x, a and b are float4 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 clamp(float4 x, float4 a, float4 b) { return max(a, min(b, x)); }


        /// <summary>Returns the result of clamping the float value x into the interval [0, 1].</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat saturate(sfloat x) { return clamp(x, sfloat.Zero, sfloat.One); }

        /// <summary>Returns the result of a componentwise clamping of the float2 vector x into the interval [0, 1].</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 saturate(float2 x) { return clamp(x, new float2(sfloat.Zero), new float2(sfloat.One)); }

        /// <summary>Returns the result of a componentwise clamping of the float3 vector x into the interval [0, 1].</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 saturate(float3 x) { return clamp(x, new float3(sfloat.Zero), new float3(sfloat.One)); }

        /// <summary>Returns the result of a componentwise clamping of the float4 vector x into the interval [0, 1].</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 saturate(float4 x) { return clamp(x, new float4(sfloat.Zero), new float4(sfloat.One)); }


        /// <summary>Returns the absolute value of a int value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int abs(int x) { return max(-x, x); }

        /// <summary>Returns the componentwise absolute value of a int2 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int2 abs(int2 x) { return max(-x, x); }

        /// <summary>Returns the componentwise absolute value of a int3 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int3 abs(int3 x) { return max(-x, x); }

        /// <summary>Returns the componentwise absolute value of a int4 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int4 abs(int4 x) { return max(-x, x); }

        /// <summary>Returns the absolute value of a long value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static long abs(long x) { return max(-x, x); }


        /// <summary>Returns the absolute value of a float value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat abs(sfloat x) { return sfloat.Abs(x); }

        /// <summary>Returns the componentwise absolute value of a float2 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 abs(float2 x) { return new float2(abs(x.x), abs(x.y)); }

        /// <summary>Returns the componentwise absolute value of a float3 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 abs(float3 x) { return new float3(abs(x.x), abs(x.y), abs(x.z)); }

        /// <summary>Returns the componentwise absolute value of a float4 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 abs(float4 x) { return new float4(abs(x.x), abs(x.y), abs(x.z), abs(x.w)); }


        /// <summary>Returns the dot product of two int values. Equivalent to multiplication.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int dot(int x, int y) { return x * y; }

        /// <summary>Returns the dot product of two int2 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int dot(int2 x, int2 y) { return x.x * y.x + x.y * y.y; }

        /// <summary>Returns the dot product of two int3 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int dot(int3 x, int3 y) { return x.x * y.x + x.y * y.y + x.z * y.z; }

        /// <summary>Returns the dot product of two int4 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int dot(int4 x, int4 y) { return x.x * y.x + x.y * y.y + x.z * y.z + x.w * y.w; }


        /// <summary>Returns the dot product of two uint values. Equivalent to multiplication.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint dot(uint x, uint y) { return x * y; }

        /// <summary>Returns the dot product of two uint2 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint dot(uint2 x, uint2 y) { return x.x * y.x + x.y * y.y; }

        /// <summary>Returns the dot product of two uint3 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint dot(uint3 x, uint3 y) { return x.x * y.x + x.y * y.y + x.z * y.z; }

        /// <summary>Returns the dot product of two uint4 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint dot(uint4 x, uint4 y) { return x.x * y.x + x.y * y.y + x.z * y.z + x.w * y.w; }


        /// <summary>Returns the dot product of two float values. Equivalent to multiplication.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat dot(sfloat x, sfloat y) { return x * y; }

        /// <summary>Returns the dot product of two float2 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat dot(float2 x, float2 y) { return x.x * y.x + x.y * y.y; }

        /// <summary>Returns the dot product of two float3 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat dot(float3 x, float3 y) { return x.x * y.x + x.y * y.y + x.z * y.z; }

        /// <summary>Returns the dot product of two float4 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat dot(float4 x, float4 y) { return x.x * y.x + x.y * y.y + x.z * y.z + x.w * y.w; }


        /// <summary>Returns the tangent of a float value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat tan(sfloat x) { return libm.tanf(x); }

        /// <summary>Returns the componentwise tangent of a float2 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 tan(float2 x) { return new float2(tan(x.x), tan(x.y)); }

        /// <summary>Returns the componentwise tangent of a float3 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 tan(float3 x) { return new float3(tan(x.x), tan(x.y), tan(x.z)); }

        /// <summary>Returns the componentwise tangent of a float4 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 tan(float4 x) { return new float4(tan(x.x), tan(x.y), tan(x.z), tan(x.w)); }


        /// <summary>Returns the arctangent of a float value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat atan(sfloat x) { return libm.atanf(x); }

        /// <summary>Returns the componentwise arctangent of a float2 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 atan(float2 x) { return new float2(atan(x.x), atan(x.y)); }

        /// <summary>Returns the componentwise arctangent of a float3 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 atan(float3 x) { return new float3(atan(x.x), atan(x.y), atan(x.z)); }

        /// <summary>Returns the componentwise arctangent of a float4 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 atan(float4 x) { return new float4(atan(x.x), atan(x.y), atan(x.z), atan(x.w)); }


        /// <summary>Returns the 2-argument arctangent of a pair of float values.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat atan2(sfloat y, sfloat x) { return libm.atan2f(y, x); }

        /// <summary>Returns the componentwise 2-argument arctangent of a pair of floats2 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 atan2(float2 y, float2 x) { return new float2(atan2(y.x, x.x), atan2(y.y, x.y)); }

        /// <summary>Returns the componentwise 2-argument arctangent of a pair of floats3 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 atan2(float3 y, float3 x) { return new float3(atan2(y.x, x.x), atan2(y.y, x.y), atan2(y.z, x.z)); }

        /// <summary>Returns the componentwise 2-argument arctangent of a pair of floats4 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 atan2(float4 y, float4 x) { return new float4(atan2(y.x, x.x), atan2(y.y, x.y), atan2(y.z, x.z), atan2(y.w, x.w)); }


        /// <summary>Returns the cosine of a float value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat cos(sfloat x) { return libm.cosf(x); }

        /// <summary>Returns the componentwise cosine of a float2 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 cos(float2 x) { return new float2(cos(x.x), cos(x.y)); }

        /// <summary>Returns the componentwise cosine of a float3 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 cos(float3 x) { return new float3(cos(x.x), cos(x.y), cos(x.z)); }

        /// <summary>Returns the componentwise cosine of a float4 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 cos(float4 x) { return new float4(cos(x.x), cos(x.y), cos(x.z), cos(x.w)); }


        /// <summary>Returns the arccosine of a float value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat acos(sfloat x) { return libm.acosf(x); }

        /// <summary>Returns the componentwise arccosine of a float2 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 acos(float2 x) { return new float2(acos(x.x), acos(x.y)); }

        /// <summary>Returns the componentwise arccosine of a float3 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 acos(float3 x) { return new float3(acos(x.x), acos(x.y), acos(x.z)); }

        /// <summary>Returns the componentwise arccosine of a float4 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 acos(float4 x) { return new float4(acos(x.x), acos(x.y), acos(x.z), acos(x.w)); }


        /// <summary>Returns the sine of a float value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat sin(sfloat x) { return libm.sinf(x); }

        /// <summary>Returns the componentwise sine of a float2 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 sin(float2 x) { return new float2(sin(x.x), sin(x.y)); }

        /// <summary>Returns the componentwise sine of a float3 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 sin(float3 x) { return new float3(sin(x.x), sin(x.y), sin(x.z)); }

        /// <summary>Returns the componentwise sine of a float4 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 sin(float4 x) { return new float4(sin(x.x), sin(x.y), sin(x.z), sin(x.w)); }


        /// <summary>Returns the arcsine of a float value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat asin(sfloat x) { return libm.asinf(x); }

        /// <summary>Returns the componentwise arcsine of a float2 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 asin(float2 x) { return new float2(asin(x.x), asin(x.y)); }

        /// <summary>Returns the componentwise arcsine of a float3 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 asin(float3 x) { return new float3(asin(x.x), asin(x.y), asin(x.z)); }

        /// <summary>Returns the componentwise arcsine of a float4 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 asin(float4 x) { return new float4(asin(x.x), asin(x.y), asin(x.z), asin(x.w)); }


        /// <summary>Returns the result of rounding a float value up to the nearest integral value less or equal to the original value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat floor(sfloat x) { return libm.floorf(x); }

        /// <summary>Returns the result of rounding each component of a float2 vector value down to the nearest value less or equal to the original value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 floor(float2 x) { return new float2(floor(x.x), floor(x.y)); }

        /// <summary>Returns the result of rounding each component of a float3 vector value down to the nearest value less or equal to the original value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 floor(float3 x) { return new float3(floor(x.x), floor(x.y), floor(x.z)); }

        /// <summary>Returns the result of rounding each component of a float4 vector value down to the nearest value less or equal to the original value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 floor(float4 x) { return new float4(floor(x.x), floor(x.y), floor(x.z), floor(x.w)); }


        /// <summary>Returns the result of rounding a float value up to the nearest integral value greater or equal to the original value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat ceil(sfloat x) { return libm.ceilf(x); }

        /// <summary>Returns the result of rounding each component of a float2 vector value up to the nearest value greater or equal to the original value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 ceil(float2 x) { return new float2(ceil(x.x), ceil(x.y)); }

        /// <summary>Returns the result of rounding each component of a float3 vector value up to the nearest value greater or equal to the original value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 ceil(float3 x) { return new float3(ceil(x.x), ceil(x.y), ceil(x.z)); }

        /// <summary>Returns the result of rounding each component of a float4 vector value up to the nearest value greater or equal to the original value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 ceil(float4 x) { return new float4(ceil(x.x), ceil(x.y), ceil(x.z), ceil(x.w)); }


        /// <summary>Returns the result of rounding a float value to the nearest integral value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat round(sfloat x) { return libm.roundf(x); }

        /// <summary>Returns the result of rounding each component of a float2 vector value to the nearest integral value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 round(float2 x) { return new float2(round(x.x), round(x.y)); }

        /// <summary>Returns the result of rounding each component of a float3 vector value to the nearest integral value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 round(float3 x) { return new float3(round(x.x), round(x.y), round(x.z)); }

        /// <summary>Returns the result of rounding each component of a float4 vector value to the nearest integral value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 round(float4 x) { return new float4(round(x.x), round(x.y), round(x.z), round(x.w)); }


        /// <summary>Returns the result of truncating a float value to an integral float value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat trunc(sfloat x) { return libm.truncf(x); }

        /// <summary>Returns the result of a componentwise truncation of a float2 value to an integral float2 value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 trunc(float2 x) { return new float2(trunc(x.x), trunc(x.y)); }

        /// <summary>Returns the result of a componentwise truncation of a float3 value to an integral float3 value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 trunc(float3 x) { return new float3(trunc(x.x), trunc(x.y), trunc(x.z)); }

        /// <summary>Returns the result of a componentwise truncation of a float4 value to an integral float4 value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 trunc(float4 x) { return new float4(trunc(x.x), trunc(x.y), trunc(x.z), trunc(x.w)); }


        /// <summary>Returns the fractional part of a float value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat frac(sfloat x) { return x - floor(x); }

        /// <summary>Returns the componentwise fractional parts of a float2 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 frac(float2 x) { return x - floor(x); }

        /// <summary>Returns the componentwise fractional parts of a float3 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 frac(float3 x) { return x - floor(x); }

        /// <summary>Returns the componentwise fractional parts of a float4 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 frac(float4 x) { return x - floor(x); }


        /// <summary>Returns the reciprocal a float value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat rcp(sfloat x) { return sfloat.One / x; }

        /// <summary>Returns the componentwise reciprocal a float2 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 rcp(float2 x) { return sfloat.One / x; }

        /// <summary>Returns the componentwise reciprocal a float3 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 rcp(float3 x) { return sfloat.One / x; }

        /// <summary>Returns the componentwise reciprocal a float4 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 rcp(float4 x) { return sfloat.One / x; }


        /// <summary>Returns the sign of a float value. -1.0f if it is less than zero, 0.0f if it is zero and 1.0f if it greater than zero.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat sign(sfloat x) { return x > sfloat.Zero ? sfloat.One : x < sfloat.Zero ? -sfloat.One : sfloat.Zero; }

        /// <summary>Returns the componentwise sign of a float2 value. 1.0f for positive components, 0.0f for zero components and -1.0f for negative components.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 sign(float2 x) { return new float2(sign(x.x), sign(x.y)); }

        /// <summary>Returns the componentwise sign of a float3 value. 1.0f for positive components, 0.0f for zero components and -1.0f for negative components.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 sign(float3 x) { return new float3(sign(x.x), sign(x.y), sign(x.z)); }

        /// <summary>Returns the componentwise sign of a float4 value. 1.0f for positive components, 0.0f for zero components and -1.0f for negative components.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 sign(float4 x) { return new float4(sign(x.x), sign(x.y), sign(x.z), sign(x.w)); }


        /// <summary>Returns x raised to the power y.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat pow(sfloat x, sfloat y) { return libm.powf(x, y); }

        /// <summary>Returns the componentwise result of raising x to the power y.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 pow(float2 x, float2 y) { return new float2(pow(x.x, y.x), pow(x.y, y.y)); }

        /// <summary>Returns the componentwise result of raising x to the power y.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 pow(float3 x, float3 y) { return new float3(pow(x.x, y.x), pow(x.y, y.y), pow(x.z, y.z)); }

        /// <summary>Returns the componentwise result of raising x to the power y.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 pow(float4 x, float4 y) { return new float4(pow(x.x, y.x), pow(x.y, y.y), pow(x.z, y.z), pow(x.w, y.w)); }


        /// <summary>Returns the base-e exponential of x.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat exp(sfloat x) { return libm.expf(x); }

        /// <summary>Returns the componentwise base-e exponential of x.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 exp(float2 x) { return new float2(exp(x.x), exp(x.y)); }

        /// <summary>Returns the componentwise base-e exponential of x.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 exp(float3 x) { return new float3(exp(x.x), exp(x.y), exp(x.z)); }

        /// <summary>Returns the componentwise base-e exponential of x.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 exp(float4 x) { return new float4(exp(x.x), exp(x.y), exp(x.z), exp(x.w)); }


        /// <summary>Returns the base-2 exponential of x.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat exp2(sfloat x) { return libm.expf(x) * sfloat.FromRaw(0x3f317218); }

        /// <summary>Returns the componentwise base-2 exponential of x.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 exp2(float2 x) { return new float2(exp2(x.x), exp2(x.y)); }

        /// <summary>Returns the componentwise base-2 exponential of x.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 exp2(float3 x) { return new float3(exp2(x.x), exp2(x.y), exp2(x.z)); }

        /// <summary>Returns the componentwise base-2 exponential of x.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 exp2(float4 x) { return new float4(exp2(x.x), exp2(x.y), exp2(x.z), exp2(x.w)); }


        /// <summary>Returns the base-10 exponential of x.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat exp10(sfloat x) { return libm.expf(x) * sfloat.FromRaw(0x40135d8e); }

        /// <summary>Returns the componentwise base-10 exponential of x.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 exp10(float2 x) { return new float2(exp10(x.x), exp10(x.y)); }

        /// <summary>Returns the componentwise base-10 exponential of x.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 exp10(float3 x) { return new float3(exp10(x.x), exp10(x.y), exp10(x.z)); }

        /// <summary>Returns the componentwise base-10 exponential of x.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 exp10(float4 x) { return new float4(exp10(x.x), exp10(x.y), exp10(x.z), exp10(x.w)); }


        /// <summary>Returns the natural logarithm of a float value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat log(sfloat x) { return libm.logf(x); }

        /// <summary>Returns the componentwise natural logarithm of a float2 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 log(float2 x) { return new float2(log(x.x), log(x.y)); }

        /// <summary>Returns the componentwise natural logarithm of a float3 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 log(float3 x) { return new float3(log(x.x), log(x.y), log(x.z)); }

        /// <summary>Returns the componentwise natural logarithm of a float4 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 log(float4 x) { return new float4(log(x.x), log(x.y), log(x.z), log(x.w)); }


        /// <summary>Returns the base-2 logarithm of a float value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat log2(sfloat x) { return libm.log2f(x); }

        /// <summary>Returns the componentwise base-2 logarithm of a float2 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 log2(float2 x) { return new float2(log2(x.x), log2(x.y)); }

        /// <summary>Returns the componentwise base-2 logarithm of a float3 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 log2(float3 x) { return new float3(log2(x.x), log2(x.y), log2(x.z)); }

        /// <summary>Returns the componentwise base-2 logarithm of a float4 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 log2(float4 x) { return new float4(log2(x.x), log2(x.y), log2(x.z), log2(x.w)); }


        /// <summary>Returns the base-10 logarithm of a float value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat log10(sfloat x) { return libm.logf(x) * LOG10E; }

        /// <summary>Returns the componentwise base-10 logarithm of a float2 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 log10(float2 x) { return new float2(log10(x.x), log10(x.y)); }

        /// <summary>Returns the componentwise base-10 logarithm of a float3 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 log10(float3 x) { return new float3(log10(x.x), log10(x.y), log10(x.z)); }

        /// <summary>Returns the componentwise base-10 logarithm of a float4 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 log10(float4 x) { return new float4(log10(x.x), log10(x.y), log10(x.z), log10(x.w)); }


        /// <summary>Returns the floating point remainder of x/y.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat fmod(sfloat x, sfloat y) { return x % y; }

        /// <summary>Returns the componentwise floating point remainder of x/y.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 fmod(float2 x, float2 y) { return new float2(x.x % y.x, x.y % y.y); }

        /// <summary>Returns the componentwise floating point remainder of x/y.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 fmod(float3 x, float3 y) { return new float3(x.x % y.x, x.y % y.y, x.z % y.z); }

        /// <summary>Returns the componentwise floating point remainder of x/y.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 fmod(float4 x, float4 y) { return new float4(x.x % y.x, x.y % y.y, x.z % y.z, x.w % y.w); }


        /// <summary>Splits a float value into an integral part i and a fractional part that gets returned. Both parts take the sign of the input.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat modf(sfloat x, out sfloat i) { i = trunc(x); return x - i; }

        /// <summary>
        /// Performs a componentwise split of a float2 vector into an integral part i and a fractional part that gets returned.
        /// Both parts take the sign of the corresponding input component.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 modf(float2 x, out float2 i) { i = trunc(x); return x - i; }

        /// <summary>
        /// Performs a componentwise split of a float3 vector into an integral part i and a fractional part that gets returned.
        /// Both parts take the sign of the corresponding input component.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 modf(float3 x, out float3 i) { i = trunc(x); return x - i; }

        /// <summary>
        /// Performs a componentwise split of a float4 vector into an integral part i and a fractional part that gets returned.
        /// Both parts take the sign of the corresponding input component.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 modf(float4 x, out float4 i) { i = trunc(x); return x - i; }


        /// <summary>Returns the square root of a float value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat sqrt(sfloat x) { return libm.sqrtf(x); }

        /// <summary>Returns the componentwise square root of a float2 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 sqrt(float2 x) { return new float2(sqrt(x.x), sqrt(x.y)); }

        /// <summary>Returns the componentwise square root of a float3 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 sqrt(float3 x) { return new float3(sqrt(x.x), sqrt(x.y), sqrt(x.z)); }

        /// <summary>Returns the componentwise square root of a float4 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 sqrt(float4 x) { return new float4(sqrt(x.x), sqrt(x.y), sqrt(x.z), sqrt(x.w)); }


        /// <summary>Returns the reciprocal square root of a float value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat rsqrt(sfloat x) { return sfloat.One / sqrt(x); }

        /// <summary>Returns the componentwise reciprocal square root of a float2 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 rsqrt(float2 x) { return sfloat.One / sqrt(x); }

        /// <summary>Returns the componentwise reciprocal square root of a float3 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 rsqrt(float3 x) { return sfloat.One / sqrt(x); }

        /// <summary>Returns the componentwise reciprocal square root of a float4 vector</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 rsqrt(float4 x) { return sfloat.One / sqrt(x); }


        /// <summary>Returns a normalized version of the float2 vector x by scaling it by 1 / length(x).</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 normalize(float2 x) { return rsqrt(dot(x, x)) * x; }

        /// <summary>Returns a normalized version of the float3 vector x by scaling it by 1 / length(x).</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 normalize(float3 x) { return rsqrt(dot(x, x)) * x; }

        /// <summary>Returns a normalized version of the float4 vector x by scaling it by 1 / length(x).</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 normalize(float4 x) { return rsqrt(dot(x, x)) * x; }


        /// <summary>
        /// Returns a safe normalized version of the float2 vector x by scaling it by 1 / length(x).
        /// Returns the given default value when 1 / length(x) does not produce a finite number.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static public float2 normalizesafe(float2 x, float2 defaultvalue = new float2())
        {
            sfloat len = math.dot(x, x);
            return math.select(defaultvalue, x * math.rsqrt(len), len > FLT_MIN_NORMAL);
        }

        /// <summary>
        /// Returns a safe normalized version of the float3 vector x by scaling it by 1 / length(x).
        /// Returns the given default value when 1 / length(x) does not produce a finite number.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static public float3 normalizesafe(float3 x, float3 defaultvalue = new float3())
        {
            sfloat len = math.dot(x, x);
            return math.select(defaultvalue, x * math.rsqrt(len), len > FLT_MIN_NORMAL);
        }

        /// <summary>
        /// Returns a safe normalized version of the float4 vector x by scaling it by 1 / length(x).
        /// Returns the given default value when 1 / length(x) does not produce a finite number.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static public float4 normalizesafe(float4 x, float4 defaultvalue = new float4())
        {
            sfloat len = math.dot(x, x);
            return math.select(defaultvalue, x * math.rsqrt(len), len > FLT_MIN_NORMAL);
        }


        /// <summary>Returns the length of a float value. Equivalent to the absolute value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat length(sfloat x) { return abs(x); }

        /// <summary>Returns the length of a float2 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat length(float2 x) { return sqrt(dot(x, x)); }

        /// <summary>Returns the length of a float3 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat length(float3 x) { return sqrt(dot(x, x)); }

        /// <summary>Returns the length of a float4 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat length(float4 x) { return sqrt(dot(x, x)); }


        /// <summary>Returns the squared length of a float value. Equivalent to squaring the value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat lengthsq(sfloat x) { return x*x; }

        /// <summary>Returns the squared length of a float2 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat lengthsq(float2 x) { return dot(x, x); }

        /// <summary>Returns the squared length of a float3 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat lengthsq(float3 x) { return dot(x, x); }

        /// <summary>Returns the squared length of a float4 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat lengthsq(float4 x) { return dot(x, x); }


        /// <summary>Returns the distance between two float values.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat distance(sfloat x, sfloat y) { return abs(y - x); }

        /// <summary>Returns the distance between two float2 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat distance(float2 x, float2 y) { return length(y - x); }

        /// <summary>Returns the distance between two float3 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat distance(float3 x, float3 y) { return length(y - x); }

        /// <summary>Returns the distance between two float4 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat distance(float4 x, float4 y) { return length(y - x); }


        /// <summary>Returns the distance between two float values.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat distancesq(sfloat x, sfloat y) { sfloat ymx = y - x; return ymx * ymx; }

        /// <summary>Returns the distance between two float2 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat distancesq(float2 x, float2 y) { return lengthsq(y - x); }

        /// <summary>Returns the distance between two float3 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat distancesq(float3 x, float3 y) { return lengthsq(y - x); }

        /// <summary>Returns the distance between two float4 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat distancesq(float4 x, float4 y) { return lengthsq(y - x); }


        /// <summary>Returns the cross product of two float3 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 cross(float3 x, float3 y) { return (x * y.yzx - x.yzx * y).yzx; }


        /// <summary>Returns a smooth Hermite interpolation between 0.0f and 1.0f when x is in [a, b].</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat smoothstep(sfloat a, sfloat b, sfloat x)
        {
            var t = saturate((x - a) / (b - a));
            return t * t * ((sfloat)3.0f - ((sfloat)2.0f * t));
        }

        /// <summary>Returns a componentwise smooth Hermite interpolation between 0.0f and 1.0f when x is in [a, b].</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 smoothstep(float2 a, float2 b, float2 x)
        {
            var t = saturate((x - a) / (b - a));
            return t * t * ((sfloat)3.0f - ((sfloat)2.0f * t));
        }

        /// <summary>Returns a componentwise smooth Hermite interpolation between 0.0f and 1.0f when x is in [a, b].</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 smoothstep(float3 a, float3 b, float3 x)
        {
            var t = saturate((x - a) / (b - a));
            return t * t * ((sfloat)3.0f - ((sfloat)2.0f * t));
        }

        /// <summary>Returns a componentwise smooth Hermite interpolation between 0.0f and 1.0f when x is in [a, b].</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 smoothstep(float4 a, float4 b, float4 x)
        {
            var t = saturate((x - a) / (b - a));
            return t * t * ((sfloat)3.0f - ((sfloat)2.0f * t));
        }


        /// <summary>Returns true if any component of the input bool2 vector is true, false otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool any(bool2 x) { return x.x || x.y; }

        /// <summary>Returns true if any component of the input bool3 vector is true, false otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool any(bool3 x) { return x.x || x.y || x.z; }

        /// <summary>Returns true if any components of the input bool4 vector is true, false otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool any(bool4 x) { return x.x || x.y || x.z || x.w; }


        /// <summary>Returns true if any component of the input int2 vector is non-zero, false otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool any(int2 x) { return x.x != 0 || x.y != 0; }

        /// <summary>Returns true if any component of the input int3 vector is non-zero, false otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool any(int3 x) { return x.x != 0 || x.y != 0 || x.z != 0; }

        /// <summary>Returns true if any components of the input int4 vector is non-zero, false otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool any(int4 x) { return x.x != 0 || x.y != 0 || x.z != 0 || x.w != 0; }


        /// <summary>Returns true if any component of the input uint2 vector is non-zero, false otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool any(uint2 x) { return x.x != 0 || x.y != 0; }

        /// <summary>Returns true if any component of the input uint3 vector is non-zero, false otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool any(uint3 x) { return x.x != 0 || x.y != 0 || x.z != 0; }

        /// <summary>Returns true if any components of the input uint4 vector is non-zero, false otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool any(uint4 x) { return x.x != 0 || x.y != 0 || x.z != 0 || x.w != 0; }


        /// <summary>Returns true if any component of the input float2 vector is non-zero, false otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool any(float2 x) { return !x.x.IsZero() || !x.y.IsZero(); }

        /// <summary>Returns true if any component of the input float3 vector is non-zero, false otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool any(float3 x) { return !x.x.IsZero() || !x.y.IsZero() || !x.z.IsZero(); }

        /// <summary>Returns true if any component of the input float4 vector is non-zero, false otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool any(float4 x) { return !x.x.IsZero() || !x.y.IsZero() || !x.z.IsZero() || !x.w.IsZero(); }


        /// <summary>Returns true if all components of the input bool2 vector are true, false otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool all(bool2 x) { return x.x && x.y; }

        /// <summary>Returns true if all components of the input bool3 vector are true, false otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool all(bool3 x) { return x.x && x.y && x.z; }

        /// <summary>Returns true if all components of the input bool4 vector are true, false otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool all(bool4 x) { return x.x && x.y && x.z && x.w; }


        /// <summary>Returns true if all components of the input int2 vector are non-zero, false otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool all(int2 x) { return x.x != 0 && x.y != 0; }

        /// <summary>Returns true if all components of the input int3 vector are non-zero, false otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool all(int3 x) { return x.x != 0 && x.y != 0 && x.z != 0; }

        /// <summary>Returns true if all components of the input int4 vector are non-zero, false otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool all(int4 x) { return x.x != 0 && x.y != 0 && x.z != 0 && x.w != 0; }


        /// <summary>Returns true if all components of the input uint2 vector are non-zero, false otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool all(uint2 x) { return x.x != 0 && x.y != 0; }

        /// <summary>Returns true if all components of the input uint3 vector are non-zero, false otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool all(uint3 x) { return x.x != 0 && x.y != 0 && x.z != 0; }

        /// <summary>Returns true if all components of the input uint4 vector are non-zero, false otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool all(uint4 x) { return x.x != 0 && x.y != 0 && x.z != 0 && x.w != 0; }


        /// <summary>Returns true if all components of the input float2 vector are non-zero, false otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool all(float2 x) { return !x.x.IsZero() && !x.y.IsZero(); }

        /// <summary>Returns true if all components of the input float3 vector are non-zero, false otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool all(float3 x) { return !x.x.IsZero() && !x.y.IsZero() && !x.z.IsZero(); }

        /// <summary>Returns true if all components of the input float4 vector are non-zero, false otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool all(float4 x) { return !x.x.IsZero() && !x.y.IsZero() && !x.z.IsZero() && !x.w.IsZero(); }


        /// <summary>Returns b if c is true, a otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int select(int a, int b, bool c)    { return c ? b : a; }

        /// <summary>Returns b if c is true, a otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int2 select(int2 a, int2 b, bool c) { return c ? b : a; }

        /// <summary>Returns b if c is true, a otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int3 select(int3 a, int3 b, bool c) { return c ? b : a; }

        /// <summary>Returns b if c is true, a otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int4 select(int4 a, int4 b, bool c) { return c ? b : a; }


        /// <summary>
        /// Returns a componentwise selection between two int2 vectors a and b based on a bool2 selection mask c.
        /// Per component, the component from b is selected when c is true, otherwise the component from a is selected.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int2 select(int2 a, int2 b, bool2 c) { return new int2(c.x ? b.x : a.x, c.y ? b.y : a.y); }

        /// <summary>
        /// Returns a componentwise selection between two int3 vectors a and b based on a bool3 selection mask c.
        /// Per component, the component from b is selected when c is true, otherwise the component from a is selected.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int3 select(int3 a, int3 b, bool3 c) { return new int3(c.x ? b.x : a.x, c.y ? b.y : a.y, c.z ? b.z : a.z); }

        /// <summary>
        /// Returns a componentwise selection between two int4 vectors a and b based on a bool4 selection mask c.
        /// Per component, the component from b is selected when c is true, otherwise the component from a is selected.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int4 select(int4 a, int4 b, bool4 c) { return new int4(c.x ? b.x : a.x, c.y ? b.y : a.y, c.z ? b.z : a.z, c.w ? b.w : a.w); }


        /// <summary>Returns b if c is true, a otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint select(uint a, uint b, bool c) { return c ? b : a; }

        /// <summary>Returns b if c is true, a otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint2 select(uint2 a, uint2 b, bool c) { return c ? b : a; }

        /// <summary>Returns b if c is true, a otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint3 select(uint3 a, uint3 b, bool c) { return c ? b : a; }

        /// <summary>Returns b if c is true, a otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint4 select(uint4 a, uint4 b, bool c) { return c ? b : a; }


        /// <summary>
        /// Returns a componentwise selection between two uint2 vectors a and b based on a bool2 selection mask c.
        /// Per component, the component from b is selected when c is true, otherwise the component from a is selected.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint2 select(uint2 a, uint2 b, bool2 c) { return new uint2(c.x ? b.x : a.x, c.y ? b.y : a.y); }

        /// <summary>
        /// Returns a componentwise selection between two uint3 vectors a and b based on a bool3 selection mask c.
        /// Per component, the component from b is selected when c is true, otherwise the component from a is selected.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint3 select(uint3 a, uint3 b, bool3 c) { return new uint3(c.x ? b.x : a.x, c.y ? b.y : a.y, c.z ? b.z : a.z); }

        /// <summary>
        /// Returns a componentwise selection between two uint4 vectors a and b based on a bool4 selection mask c.
        /// Per component, the component from b is selected when c is true, otherwise the component from a is selected.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint4 select(uint4 a, uint4 b, bool4 c) { return new uint4(c.x ? b.x : a.x, c.y ? b.y : a.y, c.z ? b.z : a.z, c.w ? b.w : a.w); }


        /// <summary>Returns b if c is true, a otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static long select(long a, long b, bool c) { return c ? b : a; }

        /// <summary>Returns b if c is true, a otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ulong select(ulong a, ulong b, bool c) { return c ? b : a; }


        /// <summary>Returns b if c is true, a otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat select(sfloat a, sfloat b, bool c)    { return c ? b : a; }

        /// <summary>Returns b if c is true, a otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 select(float2 a, float2 b, bool c) { return c ? b : a; }

        /// <summary>Returns b if c is true, a otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 select(float3 a, float3 b, bool c) { return c ? b : a; }

        /// <summary>Returns b if c is true, a otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 select(float4 a, float4 b, bool c) { return c ? b : a; }


        /// <summary>
        /// Returns a componentwise selection between two float2 vectors a and b based on a bool2 selection mask c.
        /// Per component, the component from b is selected when c is true, otherwise the component from a is selected.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 select(float2 a, float2 b, bool2 c) { return new float2(c.x ? b.x : a.x, c.y ? b.y : a.y); }

        /// <summary>
        /// Returns a componentwise selection between two float3 vectors a and b based on a bool3 selection mask c.
        /// Per component, the component from b is selected when c is true, otherwise the component from a is selected.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 select(float3 a, float3 b, bool3 c) { return new float3(c.x ? b.x : a.x, c.y ? b.y : a.y, c.z ? b.z : a.z); }

        /// <summary>
        /// Returns a componentwise selection between two float4 vectors a and b based on a bool4 selection mask c.
        /// Per component, the component from b is selected when c is true, otherwise the component from a is selected.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 select(float4 a, float4 b, bool4 c) { return new float4(c.x ? b.x : a.x, c.y ? b.y : a.y, c.z ? b.z : a.z, c.w ? b.w : a.w); }


        /// <summary>Computes a step function. Returns 1.0f when x >= y, 0.0f otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat step(sfloat y, sfloat x) { return select(sfloat.Zero, sfloat.One, x >= y); }

        /// <summary>Returns the result of a componentwise step function where each component is 1.0f when x >= y and 0.0f otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 step(float2 y, float2 x) { return select(float2(sfloat.Zero), float2(sfloat.One), x >= y); }

        /// <summary>Returns the result of a componentwise step function where each component is 1.0f when x >= y and 0.0f otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 step(float3 y, float3 x) { return select(float3(sfloat.Zero), float3(sfloat.One), x >= y); }

        /// <summary>Returns the result of a componentwise step function where each component is 1.0f when x >= y and 0.0f otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 step(float4 y, float4 x) { return select(float4(sfloat.Zero), float4(sfloat.One), x >= y); }


        /// <summary>Given an incident vector i and a normal vector n, returns the reflection vector r = i - 2.0f * dot(i, n) * n.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 reflect(float2 i, float2 n) { return i - (sfloat)2.0f * n * dot(i, n); }

        /// <summary>Given an incident vector i and a normal vector n, returns the reflection vector r = i - 2.0f * dot(i, n) * n.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 reflect(float3 i, float3 n) { return i - (sfloat)2.0f * n * dot(i, n); }

        /// <summary>Given an incident vector i and a normal vector n, returns the reflection vector r = i - 2.0f * dot(i, n) * n.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 reflect(float4 i, float4 n) { return i - (sfloat)2.0f * n * dot(i, n); }


        /// <summary>Returns the refraction vector given the incident vector i, the normal vector n and the refraction index eta.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 refract(float2 i, float2 n, sfloat eta)
        {
            sfloat ni = dot(n, i);
            sfloat k = sfloat.One - eta * eta * (sfloat.One - ni * ni);
            return select(sfloat.Zero, eta * i - (eta * ni + sqrt(k)) * n, k >= sfloat.Zero);
        }

        /// <summary>Returns the refraction vector given the incident vector i, the normal vector n and the refraction index eta.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 refract(float3 i, float3 n, sfloat eta)
        {
            sfloat ni = dot(n, i);
            sfloat k = sfloat.One - eta * eta * (sfloat.One - ni * ni);
            return select(sfloat.Zero, eta * i - (eta * ni + sqrt(k)) * n, k >= sfloat.Zero);
        }

        /// <summary>Returns the refraction vector given the incident vector i, the normal vector n and the refraction index eta.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 refract(float4 i, float4 n, sfloat eta)
        {
            sfloat ni = dot(n, i);
            sfloat k = sfloat.One - eta * eta * (sfloat.One - ni * ni);
            return select(sfloat.Zero, eta * i - (eta * ni + sqrt(k)) * n, k >= sfloat.Zero);
        }


        /// <summary>
        /// Compute vector projection of a onto b.
        /// </summary>
        /// <remarks>
        /// Some finite vectors a and b could generate a non-finite result. This is most likely when a's components
        /// are very large (close to Single.MaxValue) or when b's components are very small (close to FLT_MIN_NORMAL).
        /// In these cases, you can call <see cref="projectsafe(Unity.Mathematics.float2,Unity.Mathematics.float2,Unity.Mathematics.float2)"/>
        /// which will use a given default value if the result is not finite.
        /// </remarks>
        /// <param name="a">Vector to project.</param>
        /// <param name="b">Non-zero vector to project onto.</param>
        /// <returns>Vector projection of a onto b.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 project(float2 a, float2 b)
        {
            return (dot(a, b) / dot(b, b)) * b;
        }

        /// <summary>
        /// Compute vector projection of a onto b.
        /// </summary>
        /// <remarks>
        /// Some finite vectors a and b could generate a non-finite result. This is most likely when a's components
        /// are very large (close to Single.MaxValue) or when b's components are very small (close to FLT_MIN_NORMAL).
        /// In these cases, you can call <see cref="projectsafe(Unity.Mathematics.float3,Unity.Mathematics.float3,Unity.Mathematics.float3)"/>
        /// which will use a given default value if the result is not finite.
        /// </remarks>
        /// <param name="a">Vector to project.</param>
        /// <param name="b">Non-zero vector to project onto.</param>
        /// <returns>Vector projection of a onto b.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 project(float3 a, float3 b)
        {
            return (dot(a, b) / dot(b, b)) * b;
        }

        /// <summary>
        /// Compute vector projection of a onto b.
        /// </summary>
        /// <remarks>
        /// Some finite vectors a and b could generate a non-finite result. This is most likely when a's components
        /// are very large (close to Single.MaxValue) or when b's components are very small (close to FLT_MIN_NORMAL).
        /// In these cases, you can call <see cref="projectsafe(Unity.Mathematics.float4,Unity.Mathematics.float4,Unity.Mathematics.float4)"/>
        /// which will use a given default value if the result is not finite.
        /// </remarks>
        /// <param name="a">Vector to project.</param>
        /// <param name="b">Non-zero vector to project onto.</param>
        /// <returns>Vector projection of a onto b.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 project(float4 a, float4 b)
        {
            return (dot(a, b) / dot(b, b)) * b;
        }

        /// <summary>
        /// Compute vector projection of a onto b. If result is not finite, then return the default value instead.
        /// </summary>
        /// <remarks>
        /// This function performs extra checks to see if the result of projecting a onto b is finite. If you know that
        /// your inputs will generate a finite result or you don't care if the result is finite, then you can call
        /// <see cref="project(Unity.Mathematics.float2,Unity.Mathematics.float2)"/> instead which is faster than this
        /// function.
        /// </remarks>
        /// <param name="a">Vector to project.</param>
        /// <param name="b">Non-zero vector to project onto.</param>
        /// <param name="defaultValue">Default value to return if projection is not finite.</param>
        /// <returns>Vector projection of a onto b or the default value.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 projectsafe(float2 a, float2 b, float2 defaultValue = new float2())
        {
            var proj = project(a, b);

            return select(defaultValue, proj, all(isfinite(proj)));
        }

        /// <summary>
        /// Compute vector projection of a onto b. If result is not finite, then return the default value instead.
        /// </summary>
        /// <remarks>
        /// This function performs extra checks to see if the result of projecting a onto b is finite. If you know that
        /// your inputs will generate a finite result or you don't care if the result is finite, then you can call
        /// <see cref="project(Unity.Mathematics.float3,Unity.Mathematics.float3)"/> instead which is faster than this
        /// function.
        /// </remarks>
        /// <param name="a">Vector to project.</param>
        /// <param name="b">Non-zero vector to project onto.</param>
        /// <param name="defaultValue">Default value to return if projection is not finite.</param>
        /// <returns>Vector projection of a onto b or the default value.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 projectsafe(float3 a, float3 b, float3 defaultValue = new float3())
        {
            var proj = project(a, b);

            return select(defaultValue, proj, all(isfinite(proj)));
        }

        /// <summary>
        /// Compute vector projection of a onto b. If result is not finite, then return the default value instead.
        /// </summary>
        /// <remarks>
        /// This function performs extra checks to see if the result of projecting a onto b is finite. If you know that
        /// your inputs will generate a finite result or you don't care if the result is finite, then you can call
        /// <see cref="project(Unity.Mathematics.float4,Unity.Mathematics.float4)"/> instead which is faster than this
        /// function.
        /// </remarks>
        /// <param name="a">Vector to project.</param>
        /// <param name="b">Non-zero vector to project onto.</param>
        /// <param name="defaultValue">Default value to return if projection is not finite.</param>
        /// <returns>Vector projection of a onto b or the default value.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 projectsafe(float4 a, float4 b, float4 defaultValue = new float4())
        {
            var proj = project(a, b);

            return select(defaultValue, proj, all(isfinite(proj)));
        }


        /// <summary>Conditionally flips a vector n to face in the direction of i. Returns n if dot(i, ng) &lt; 0, -n otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 faceforward(float2 n, float2 i, float2 ng) { return select(n, -n, dot(ng, i) >= sfloat.Zero); }

        /// <summary>Conditionally flips a vector n to face in the direction of i. Returns n if dot(i, ng) &lt; 0, -n otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 faceforward(float3 n, float3 i, float3 ng) { return select(n, -n, dot(ng, i) >= sfloat.Zero); }

        /// <summary>Conditionally flips a vector n to face in the direction of i. Returns n if dot(i, ng) &lt; 0, -n otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 faceforward(float4 n, float4 i, float4 ng) { return select(n, -n, dot(ng, i) >= sfloat.Zero); }


        /// <summary>Returns the sine and cosine of the input float value x through the out parameters s and c.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void sincos(sfloat x, out sfloat s, out sfloat c) { s = sin(x); c = cos(x); }

        /// <summary>Returns the componentwise sine and cosine of the input float2 vector x through the out parameters s and c.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void sincos(float2 x, out float2 s, out float2 c) { s = sin(x); c = cos(x); }

        /// <summary>Returns the componentwise sine and cosine of the input float3 vector x through the out parameters s and c.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void sincos(float3 x, out float3 s, out float3 c) { s = sin(x); c = cos(x); }

        /// <summary>Returns the componentwise sine and cosine of the input float4 vector x through the out parameters s and c.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void sincos(float4 x, out float4 s, out float4 c) { s = sin(x); c = cos(x); }


        /// <summary>Returns number of 1-bits in the binary representation of an int value. Also known as the Hamming weight, popcnt on x86, and vcnt on ARM.</summary>
        /// <param name="x">int value in which to count bits set to 1.</param>
        /// <returns>Number of bits set to 1 within x.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int countbits(int x) { return countbits((uint)x); }

        /// <summary>Returns component-wise number of 1-bits in the binary representation of an int2 vector. Also known as the Hamming weight, popcnt on x86, and vcnt on ARM.</summary>
        /// <param name="x">int2 value in which to count bits for each component.</param>
        /// <returns>int2 containing number of bits set to 1 within each component of x.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int2 countbits(int2 x) { return countbits((uint2)x); }

        /// <summary>Returns component-wise number of 1-bits in the binary representation of an int3 vector. Also known as the Hamming weight, popcnt on x86, and vcnt on ARM.</summary>
        /// <param name="x">Number in which to count bits.</param>
        /// <returns>int3 containing number of bits set to 1 within each component of x.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int3 countbits(int3 x) { return countbits((uint3)x); }

        /// <summary>Returns component-wise number of 1-bits in the binary representation of an int4 vector. Also known as the Hamming weight, popcnt on x86, and vcnt on ARM.</summary>
        /// <param name="x">Number in which to count bits.</param>
        /// <returns>int4 containing number of bits set to 1 within each component of x.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int4 countbits(int4 x) { return countbits((uint4)x); }


        /// <summary>Returns number of 1-bits in the binary representation of a uint value. Also known as the Hamming weight, popcnt on x86, and vcnt on ARM.</summary>
        /// <param name="x">Number in which to count bits.</param>
        /// <returns>Number of bits set to 1 within x.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int countbits(uint x)
        {
            x = x - ((x >> 1) & 0x55555555);
            x = (x & 0x33333333) + ((x >> 2) & 0x33333333);
            return (int)((((x + (x >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24);
        }

        /// <summary>Returns component-wise number of 1-bits in the binary representation of a uint2 vector. Also known as the Hamming weight, popcnt on x86, and vcnt on ARM.</summary>
        /// <param name="x">Number in which to count bits.</param>
        /// <returns>int2 containing number of bits set to 1 within each component of x.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int2 countbits(uint2 x)
        {
            x = x - ((x >> 1) & 0x55555555);
            x = (x & 0x33333333) + ((x >> 2) & 0x33333333);
            return int2((((x + (x >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24);
        }

        /// <summary>Returns component-wise number of 1-bits in the binary representation of a uint3 vector. Also known as the Hamming weight, popcnt on x86, and vcnt on ARM.</summary>
        /// <param name="x">Number in which to count bits.</param>
        /// <returns>int3 containing number of bits set to 1 within each component of x.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int3 countbits(uint3 x)
        {
            x = x - ((x >> 1) & 0x55555555);
            x = (x & 0x33333333) + ((x >> 2) & 0x33333333);
            return int3((((x + (x >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24);
        }

        /// <summary>Returns component-wise number of 1-bits in the binary representation of a uint4 vector. Also known as the Hamming weight, popcnt on x86, and vcnt on ARM.</summary>
        /// <param name="x">Number in which to count bits.</param>
        /// <returns>int4 containing number of bits set to 1 within each component of x.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int4 countbits(uint4 x)
        {
            x = x - ((x >> 1) & 0x55555555);
            x = (x & 0x33333333) + ((x >> 2) & 0x33333333);
            return int4((((x + (x >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24);
        }

        /// <summary>Returns number of 1-bits in the binary representation of a ulong value. Also known as the Hamming weight, popcnt on x86, and vcnt on ARM.</summary>
        /// <param name="x">Number in which to count bits.</param>
        /// <returns>Number of bits set to 1 within x.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int countbits(ulong x)
        {
            x = x - ((x >> 1) & 0x5555555555555555);
            x = (x & 0x3333333333333333) + ((x >> 2) & 0x3333333333333333);
            return (int)((((x + (x >> 4)) & 0x0F0F0F0F0F0F0F0F) * 0x0101010101010101) >> 56);
        }

        /// <summary>Returns number of 1-bits in the binary representation of a long value. Also known as the Hamming weight, popcnt on x86, and vcnt on ARM.</summary>
        /// <param name="x">Number in which to count bits.</param>
        /// <returns>Number of bits set to 1 within x.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int countbits(long x) { return countbits((ulong)x); }


        /// <summary>Returns the componentwise number of leading zeros in the binary representations of an int vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int lzcnt(int x) { return lzcnt((uint)x); }

        /// <summary>Returns the componentwise number of leading zeros in the binary representations of an int2 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int2 lzcnt(int2 x) { return int2(lzcnt(x.x), lzcnt(x.y)); }

        /// <summary>Returns the componentwise number of leading zeros in the binary representations of an int3 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int3 lzcnt(int3 x) { return int3(lzcnt(x.x), lzcnt(x.y), lzcnt(x.z)); }

        /// <summary>Returns the componentwise number of leading zeros in the binary representations of an int4 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int4 lzcnt(int4 x) { return int4(lzcnt(x.x), lzcnt(x.y), lzcnt(x.z), lzcnt(x.w)); }


        private static readonly int[] debruijn32 = new int[32]
        {
            0, 31, 9, 30, 3, 8, 13, 29, 2, 5, 7, 21, 12, 24, 28, 19,
            1, 10, 4, 14, 6, 22, 25, 20, 11, 15, 23, 26, 16, 27, 17, 18
        };

        /// <summary>Returns number of leading zeros in the binary representations of a uint value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int lzcnt(uint x)
        {
            if (x == 0)
            {
                return 32;
            }

            x |= x >> 1;
            x |= x >> 2;
            x |= x >> 4;
            x |= x >> 8;
            x |= x >> 16;
            x++;

            return debruijn32[x * 0x076be629 >> 27];
        }


        /// <summary>Returns the componentwise number of leading zeros in the binary representations of a uint2 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int2 lzcnt(uint2 x) { return int2(lzcnt(x.x), lzcnt(x.y)); }

        /// <summary>Returns the componentwise number of leading zeros in the binary representations of a uint3 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int3 lzcnt(uint3 x) { return int3(lzcnt(x.x), lzcnt(x.y), lzcnt(x.z)); }

        /// <summary>Returns the componentwise number of leading zeros in the binary representations of a uint4 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int4 lzcnt(uint4 x) { return int4(lzcnt(x.x), lzcnt(x.y), lzcnt(x.z), lzcnt(x.w)); }


        /// <summary>Returns number of leading zeros in the binary representations of a long value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int lzcnt(long x) { return lzcnt((ulong)x); }

        private static readonly int[] debruijn64 = new int[64] {
            63,  0, 58,  1, 59, 47, 53,  2,
            60, 39, 48, 27, 54, 33, 42,  3,
            61, 51, 37, 40, 49, 18, 28, 20,
            55, 30, 34, 11, 43, 14, 22,  4,
            62, 57, 46, 52, 38, 26, 32, 41,
            50, 36, 17, 19, 29, 10, 13, 21,
            56, 45, 25, 31, 35, 16,  9, 12,
            44, 24, 15,  8, 23,  7,  6,  5
        };

        /// <summary>Returns number of leading zeros in the binary representations of a ulong value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int lzcnt(ulong x)
        {
            if (x == 0)
            {
                return 64;
            }

            x |= x >> 1;
            x |= x >> 2;
            x |= x >> 4;
            x |= x >> 8;
            x |= x >> 16;
            x |= x >> 32;
            return debruijn64[(x - (x >> 1)) * 0x07EDD5E59A4E28C2u >> 58];
        }


        /// <summary>Returns number of trailing zeros in the binary representations of an int value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int tzcnt(int x) { return tzcnt((uint)x); }

        /// <summary>Returns the componentwise number of leading zeros in the binary representations of an int2 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int2 tzcnt(int2 x) { return int2(tzcnt(x.x), tzcnt(x.y)); }

        /// <summary>Returns the componentwise number of leading zeros in the binary representations of an int3 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int3 tzcnt(int3 v) { return int3(tzcnt(v.x), tzcnt(v.y), tzcnt(v.z)); }

        /// <summary>Returns the componentwise number of leading zeros in the binary representations of an int4 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int4 tzcnt(int4 v) { return int4(tzcnt(v.x), tzcnt(v.y), tzcnt(v.z), tzcnt(v.w)); }



        private static readonly int[] debruijn_ctz32 = new int[32]
        {
            31, 0, 27, 1, 28, 13, 23, 2, 29, 21, 19, 14, 24, 16, 3, 7,
            30, 26, 12, 22, 20, 18, 15, 6, 25, 11, 17, 5, 10, 4, 9, 8
        };

        /// <summary>Returns number of trailing zeros in the binary representations of a uint value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int tzcnt(uint x)
        {
            return (x == 0 ? 1 : 0) + debruijn_ctz32[((uint)((int)x & -(int)x) * 0x0EF96A62u) >> 27];
        }

        /// <summary>Returns the componentwise number of leading zeros in the binary representations of an uint2 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int2 tzcnt(uint2 x) { return int2(tzcnt(x.x), tzcnt(x.y)); }

        /// <summary>Returns the componentwise number of leading zeros in the binary representations of an uint3 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int3 tzcnt(uint3 x) { return int3(tzcnt(x.x), tzcnt(x.y), tzcnt(x.z)); }

        /// <summary>Returns the componentwise number of leading zeros in the binary representations of an uint4 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int4 tzcnt(uint4 x) { return int4(tzcnt(x.x), tzcnt(x.y), tzcnt(x.z), tzcnt(x.w)); }


        /// <summary>Returns number of trailing zeros in the binary representations of a long value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int tzcnt(long x) { return tzcnt((ulong)x); }


        private static readonly int[] debruijn_ctz64 = new int[64]
        {
            63, 0, 1, 52, 2, 6, 53, 26, 3, 37, 40, 7, 33, 54, 47, 27,
            61, 4, 38, 45, 43, 41, 21, 8, 23, 34, 58, 55, 48, 17, 28, 10,
            62, 51, 5, 25, 36, 39, 32, 46, 60, 44, 42, 20, 22, 57, 16, 9,
            50, 24, 35, 31, 59, 19, 56, 15, 49, 30, 18, 14, 29, 13, 12, 11
        };

        /// <summary>Returns number of trailing zeros in the binary representations of a ulong value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int tzcnt(ulong x)
        {
            return (x == 0 ? 1 : 0) + debruijn_ctz64[((ulong)((long)x & -(long)x) * 0x045FBAC7992A70DAul) >> 58];
        }


        /// <summary>Returns the result of performing a reversal of the bit pattern of an int value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int reversebits(int x) { return (int)reversebits((uint)x); }

        /// <summary>Returns the result of performing a componentwise reversal of the bit pattern of an int2 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int2 reversebits(int2 x) { return (int2)reversebits((uint2)x); }

        /// <summary>Returns the result of performing a componentwise reversal of the bit pattern of an int3 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int3 reversebits(int3 x) { return (int3)reversebits((uint3)x); }

        /// <summary>Returns the result of performing a componentwise reversal of the bit pattern of an int4 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int4 reversebits(int4 x) { return (int4)reversebits((uint4)x); }


        /// <summary>Returns the result of performing a reversal of the bit pattern of a uint value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint reversebits(uint x) {
            x = ((x >> 1) & 0x55555555) | ((x & 0x55555555) << 1);
            x = ((x >> 2) & 0x33333333) | ((x & 0x33333333) << 2);
            x = ((x >> 4) & 0x0F0F0F0F) | ((x & 0x0F0F0F0F) << 4);
            x = ((x >> 8) & 0x00FF00FF) | ((x & 0x00FF00FF) << 8);
            return (x >> 16) | (x << 16);
        }

        /// <summary>Returns the result of performing a componentwise reversal of the bit pattern of an uint2 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint2 reversebits(uint2 x)
        {
            x = ((x >> 1) & 0x55555555) | ((x & 0x55555555) << 1);
            x = ((x >> 2) & 0x33333333) | ((x & 0x33333333) << 2);
            x = ((x >> 4) & 0x0F0F0F0F) | ((x & 0x0F0F0F0F) << 4);
            x = ((x >> 8) & 0x00FF00FF) | ((x & 0x00FF00FF) << 8);
            return (x >> 16) | (x << 16);
        }

        /// <summary>Returns the result of performing a componentwise reversal of the bit pattern of an uint3 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint3 reversebits(uint3 x)
        {
            x = ((x >> 1) & 0x55555555) | ((x & 0x55555555) << 1);
            x = ((x >> 2) & 0x33333333) | ((x & 0x33333333) << 2);
            x = ((x >> 4) & 0x0F0F0F0F) | ((x & 0x0F0F0F0F) << 4);
            x = ((x >> 8) & 0x00FF00FF) | ((x & 0x00FF00FF) << 8);
            return (x >> 16) | (x << 16);
        }

        /// <summary>Returns the result of performing a componentwise reversal of the bit pattern of an uint4 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint4 reversebits(uint4 x)
        {
            x = ((x >> 1) & 0x55555555) | ((x & 0x55555555) << 1);
            x = ((x >> 2) & 0x33333333) | ((x & 0x33333333) << 2);
            x = ((x >> 4) & 0x0F0F0F0F) | ((x & 0x0F0F0F0F) << 4);
            x = ((x >> 8) & 0x00FF00FF) | ((x & 0x00FF00FF) << 8);
            return (x >> 16) | (x << 16);
        }


        /// <summary>Returns the result of performing a reversal of the bit pattern of a long value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static long reversebits(long x) { return (long)reversebits((ulong)x); }


        /// <summary>Returns the result of performing a reversal of the bit pattern of a ulong value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ulong reversebits(ulong x)
        {
            x = ((x >> 1) & 0x5555555555555555ul) | ((x & 0x5555555555555555ul) << 1);
            x = ((x >> 2) & 0x3333333333333333ul) | ((x & 0x3333333333333333ul) << 2);
            x = ((x >> 4) & 0x0F0F0F0F0F0F0F0Ful) | ((x & 0x0F0F0F0F0F0F0F0Ful) << 4);
            x = ((x >> 8) & 0x00FF00FF00FF00FFul) | ((x & 0x00FF00FF00FF00FFul) << 8);
            x = ((x >> 16) & 0x0000FFFF0000FFFFul) | ((x & 0x0000FFFF0000FFFFul) << 16);
            return (x >> 32) | (x << 32);
        }


        /// <summary>Returns the result of rotating the bits of an int left by bits n.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int rol(int x, int n) { return (int)rol((uint)x, n); }

        /// <summary>Returns the componentwise result of rotating the bits of an int2 left by bits n.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int2 rol(int2 x, int n) { return (int2)rol((uint2)x, n); }

        /// <summary>Returns the componentwise result of rotating the bits of an int3 left by bits n.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int3 rol(int3 x, int n) { return (int3)rol((uint3)x, n); }

        /// <summary>Returns the componentwise result of rotating the bits of an int4 left by bits n.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int4 rol(int4 x, int n) { return (int4)rol((uint4)x, n); }


        /// <summary>Returns the result of rotating the bits of a uint left by bits n.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint rol(uint x, int n) { return (x << n) | (x >> (32 - n)); }

        /// <summary>Returns the componentwise result of rotating the bits of a uint2 left by bits n.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint2 rol(uint2 x, int n) { return (x << n) | (x >> (32 - n)); }

        /// <summary>Returns the componentwise result of rotating the bits of a uint3 left by bits n.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint3 rol(uint3 x, int n) { return (x << n) | (x >> (32 - n)); }

        /// <summary>Returns the componentwise result of rotating the bits of a uint4 left by bits n.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint4 rol(uint4 x, int n) { return (x << n) | (x >> (32 - n)); }


        /// <summary>Returns the result of rotating the bits of a long left by bits n.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static long rol(long x, int n) { return (long)rol((ulong)x, n); }


        /// <summary>Returns the result of rotating the bits of a ulong left by bits n.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ulong rol(ulong x, int n) { return (x << n) | (x >> (64 - n)); }


        /// <summary>Returns the result of rotating the bits of an int right by bits n.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int ror(int x, int n) { return (int)ror((uint)x, n); }

        /// <summary>Returns the componentwise result of rotating the bits of an int2 right by bits n.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int2 ror(int2 x, int n) { return (int2)ror((uint2)x, n); }

        /// <summary>Returns the componentwise result of rotating the bits of an int3 right by bits n.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int3 ror(int3 x, int n) { return (int3)ror((uint3)x, n); }

        /// <summary>Returns the componentwise result of rotating the bits of an int4 right by bits n.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int4 ror(int4 x, int n) { return (int4)ror((uint4)x, n); }


        /// <summary>Returns the result of rotating the bits of a uint right by bits n.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint ror(uint x, int n) { return (x >> n) | (x << (32 - n)); }

        /// <summary>Returns the componentwise result of rotating the bits of a uint2 right by bits n.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint2 ror(uint2 x, int n) { return (x >> n) | (x << (32 - n)); }

        /// <summary>Returns the componentwise result of rotating the bits of a uint3 right by bits n.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint3 ror(uint3 x, int n) { return (x >> n) | (x << (32 - n)); }

        /// <summary>Returns the componentwise result of rotating the bits of a uint4 right by bits n.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint4 ror(uint4 x, int n) { return (x >> n) | (x << (32 - n)); }


        /// <summary>Returns the result of rotating the bits of a long right by bits n.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static long ror(long x, int n) { return (long)ror((ulong)x, n); }


        /// <summary>Returns the result of rotating the bits of a ulong right by bits n.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ulong ror(ulong x, int n) { return (x >> n) | (x << (64 - n)); }


        /// <summary>Returns the smallest power of two greater than or equal to the input.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int ceilpow2(int x)
        {
            x -= 1;
            x |= x >> 1;
            x |= x >> 2;
            x |= x >> 4;
            x |= x >> 8;
            x |= x >> 16;
            return x + 1;
        }

        /// <summary>Returns the result of a componentwise calculation of the smallest power of two greater than or equal to the input.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int2 ceilpow2(int2 x)
        {
            x -= 1;
            x |= x >> 1;
            x |= x >> 2;
            x |= x >> 4;
            x |= x >> 8;
            x |= x >> 16;
            return x + 1;
        }

        /// <summary>Returns the result of a componentwise calculation of the smallest power of two greater than or equal to the input.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int3 ceilpow2(int3 x)
        {
            x -= 1;
            x |= x >> 1;
            x |= x >> 2;
            x |= x >> 4;
            x |= x >> 8;
            x |= x >> 16;
            return x + 1;
        }

        /// <summary>Returns the result of a componentwise calculation of the smallest power of two greater than or equal to the input.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int4 ceilpow2(int4 x)
        {
            x -= 1;
            x |= x >> 1;
            x |= x >> 2;
            x |= x >> 4;
            x |= x >> 8;
            x |= x >> 16;
            return x + 1;
        }


        /// <summary>Returns the smallest power of two greater than or equal to the input.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint ceilpow2(uint x)
        {
            x -= 1;
            x |= x >> 1;
            x |= x >> 2;
            x |= x >> 4;
            x |= x >> 8;
            x |= x >> 16;
            return x + 1;
        }

        /// <summary>Returns the result of a componentwise calculation of the smallest power of two greater than or equal to the input.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint2 ceilpow2(uint2 x)
        {
            x -= 1;
            x |= x >> 1;
            x |= x >> 2;
            x |= x >> 4;
            x |= x >> 8;
            x |= x >> 16;
            return x + 1;
        }

        /// <summary>Returns the result of a componentwise calculation of the smallest power of two greater than or equal to the input.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint3 ceilpow2(uint3 x)
        {
            x -= 1;
            x |= x >> 1;
            x |= x >> 2;
            x |= x >> 4;
            x |= x >> 8;
            x |= x >> 16;
            return x + 1;
        }

        /// <summary>Returns the result of a componentwise calculation of the smallest power of two greater than or equal to the input.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint4 ceilpow2(uint4 x)
        {
            x -= 1;
            x |= x >> 1;
            x |= x >> 2;
            x |= x >> 4;
            x |= x >> 8;
            x |= x >> 16;
            return x + 1;
        }


        /// <summary>Returns the smallest power of two greater than or equal to the input.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static long ceilpow2(long x)
        {
            x -= 1;
            x |= x >> 1;
            x |= x >> 2;
            x |= x >> 4;
            x |= x >> 8;
            x |= x >> 16;
            x |= x >> 32;
            return x + 1;
        }


        /// <summary>Returns the smallest power of two greater than or equal to the input.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ulong ceilpow2(ulong x)
        {
            x -= 1;
            x |= x >> 1;
            x |= x >> 2;
            x |= x >> 4;
            x |= x >> 8;
            x |= x >> 16;
            x |= x >> 32;
            return x + 1;
        }

        /// <summary>
        /// Computes the ceiling of the base-2 logarithm of x.
        /// </summary>
        /// <remarks>
        /// x must be greater than 0, otherwise the result is undefined.
        /// </remarks>
        /// <param name="x">Integer to be used as input.</param>
        /// <returns>Ceiling of the base-2 logarithm of x, as an integer.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int ceillog2(int x)
        {
            return 32 - lzcnt((uint)x - 1);
        }

        /// <summary>
        /// Computes the componentwise ceiling of the base-2 logarithm of x.
        /// </summary>
        /// <remarks>
        /// Components of x must be greater than 0, otherwise the result for that component is undefined.
        /// </remarks>
        /// <param name="x"><see cref="int2"/> to be used as input.</param>
        /// <returns>Componentwise ceiling of the base-2 logarithm of x.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int2 ceillog2(int2 x)
        {
            return new int2(ceillog2(x.x), ceillog2(x.y));
        }

        /// <summary>
        /// Computes the componentwise ceiling of the base-2 logarithm of x.
        /// </summary>
        /// <remarks>
        /// Components of x must be greater than 0, otherwise the result for that component is undefined.
        /// </remarks>
        /// <param name="x"><see cref="int3"/> to be used as input.</param>
        /// <returns>Componentwise ceiling of the base-2 logarithm of x.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int3 ceillog2(int3 x)
        {
            return new int3(ceillog2(x.x), ceillog2(x.y), ceillog2(x.z));
        }

        /// <summary>
        /// Computes the componentwise ceiling of the base-2 logarithm of x.
        /// </summary>
        /// <remarks>
        /// Components of x must be greater than 0, otherwise the result for that component is undefined.
        /// </remarks>
        /// <param name="x"><see cref="int4"/> to be used as input.</param>
        /// <returns>Componentwise ceiling of the base-2 logarithm of x.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int4 ceillog2(int4 x)
        {
            return new int4(ceillog2(x.x), ceillog2(x.y), ceillog2(x.z), ceillog2(x.w));
        }

        /// <summary>
        /// Computes the ceiling of the base-2 logarithm of x.
        /// </summary>
        /// <remarks>
        /// x must be greater than 0, otherwise the result is undefined.
        /// </remarks>
        /// <param name="x">Unsigned integer to be used as input.</param>
        /// <returns>Ceiling of the base-2 logarithm of x, as an integer.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int ceillog2(uint x)
        {
            return 32 - lzcnt(x - 1);
        }

        /// <summary>
        /// Computes the componentwise ceiling of the base-2 logarithm of x.
        /// </summary>
        /// <remarks>
        /// Components of x must be greater than 0, otherwise the result for that component is undefined.
        /// </remarks>
        /// <param name="x"><see cref="uint2"/> to be used as input.</param>
        /// <returns>Componentwise ceiling of the base-2 logarithm of x.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int2 ceillog2(uint2 x)
        {
            return new int2(ceillog2(x.x), ceillog2(x.y));
        }

        /// <summary>
        /// Computes the componentwise ceiling of the base-2 logarithm of x.
        /// </summary>
        /// <remarks>
        /// Components of x must be greater than 0, otherwise the result for that component is undefined.
        /// </remarks>
        /// <param name="x"><see cref="uint3"/> to be used as input.</param>
        /// <returns>Componentwise ceiling of the base-2 logarithm of x.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int3 ceillog2(uint3 x)
        {
            return new int3(ceillog2(x.x), ceillog2(x.y), ceillog2(x.z));
        }

        /// <summary>
        /// Computes the componentwise ceiling of the base-2 logarithm of x.
        /// </summary>
        /// <remarks>
        /// Components of x must be greater than 0, otherwise the result for that component is undefined.
        /// </remarks>
        /// <param name="x"><see cref="uint4"/> to be used as input.</param>
        /// <returns>Componentwise ceiling of the base-2 logarithm of x.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int4 ceillog2(uint4 x)
        {
            return new int4(ceillog2(x.x), ceillog2(x.y), ceillog2(x.z), ceillog2(x.w));
        }

        /// <summary>
        /// Computes the floor of the base-2 logarithm of x.
        /// </summary>
        /// <remarks>x must be greater than zero, otherwise the result is undefined.</remarks>
        /// <param name="x">Integer to be used as input.</param>
        /// <returns>Floor of base-2 logarithm of x.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int floorlog2(int x)
        {
            return 31 - lzcnt((uint)x);
        }

        /// <summary>
        /// Computes the componentwise floor of the base-2 logarithm of x.
        /// </summary>
        /// <remarks>Components of x must be greater than zero, otherwise the result of the component is undefined.</remarks>
        /// <param name="x"><see cref="int2"/> to be used as input.</param>
        /// <returns>Componentwise floor of base-2 logarithm of x.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int2 floorlog2(int2 x)
        {
            return new int2(floorlog2(x.x), floorlog2(x.y));
        }

        /// <summary>
        /// Computes the componentwise floor of the base-2 logarithm of x.
        /// </summary>
        /// <remarks>Components of x must be greater than zero, otherwise the result of the component is undefined.</remarks>
        /// <param name="x"><see cref="int3"/> to be used as input.</param>
        /// <returns>Componentwise floor of base-2 logarithm of x.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int3 floorlog2(int3 x)
        {
            return new int3(floorlog2(x.x), floorlog2(x.y), floorlog2(x.z));
        }

        /// <summary>
        /// Computes the componentwise floor of the base-2 logarithm of x.
        /// </summary>
        /// <remarks>Components of x must be greater than zero, otherwise the result of the component is undefined.</remarks>
        /// <param name="x"><see cref="int4"/> to be used as input.</param>
        /// <returns>Componentwise floor of base-2 logarithm of x.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int4 floorlog2(int4 x)
        {
            return new int4(floorlog2(x.x), floorlog2(x.y), floorlog2(x.z), floorlog2(x.w));
        }

        /// <summary>
        /// Computes the floor of the base-2 logarithm of x.
        /// </summary>
        /// <remarks>x must be greater than zero, otherwise the result is undefined.</remarks>
        /// <param name="x">Unsigned integer to be used as input.</param>
        /// <returns>Floor of base-2 logarithm of x.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int floorlog2(uint x)
        {
            return 31 - lzcnt(x);
        }

        /// <summary>
        /// Computes the componentwise floor of the base-2 logarithm of x.
        /// </summary>
        /// <remarks>Components of x must be greater than zero, otherwise the result of the component is undefined.</remarks>
        /// <param name="x"><see cref="uint2"/> to be used as input.</param>
        /// <returns>Componentwise floor of base-2 logarithm of x.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int2 floorlog2(uint2 x)
        {
            return new int2(floorlog2(x.x), floorlog2(x.y));
        }

        /// <summary>
        /// Computes the componentwise floor of the base-2 logarithm of x.
        /// </summary>
        /// <remarks>Components of x must be greater than zero, otherwise the result of the component is undefined.</remarks>
        /// <param name="x"><see cref="uint3"/> to be used as input.</param>
        /// <returns>Componentwise floor of base-2 logarithm of x.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int3 floorlog2(uint3 x)
        {
            return new int3(floorlog2(x.x), floorlog2(x.y), floorlog2(x.z));
        }

        /// <summary>
        /// Computes the componentwise floor of the base-2 logarithm of x.
        /// </summary>
        /// <remarks>Components of x must be greater than zero, otherwise the result of the component is undefined.</remarks>
        /// <param name="x"><see cref="uint4"/> to be used as input.</param>
        /// <returns>Componentwise floor of base-2 logarithm of x.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int4 floorlog2(uint4 x)
        {
            return new int4(floorlog2(x.x), floorlog2(x.y), floorlog2(x.z), floorlog2(x.w));
        }

        private const uint DEG2RAD = 0x3c8efa35;
        private const uint RAD2DEG = 0x42652ee1;

        /// <summary>Returns the result of converting a float value from degrees to radians.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat radians(sfloat x) { return x * sfloat.FromRaw(DEG2RAD); }

        /// <summary>Returns the result of a componentwise conversion of a float2 vector from degrees to radians.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 radians(float2 x) { return x * sfloat.FromRaw(DEG2RAD); }

        /// <summary>Returns the result of a componentwise conversion of a float3 vector from degrees to radians.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 radians(float3 x) { return x * sfloat.FromRaw(DEG2RAD); }

        /// <summary>Returns the result of a componentwise conversion of a float4 vector from degrees to radians.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 radians(float4 x) { return x * sfloat.FromRaw(DEG2RAD); }


        /// <summary>Returns the result of converting a double value from radians to degrees.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat degrees(sfloat x) { return x * sfloat.FromRaw(RAD2DEG); }

        /// <summary>Returns the result of a componentwise conversion of a double2 vector from radians to degrees.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 degrees(float2 x) { return x * sfloat.FromRaw(RAD2DEG); }

        /// <summary>Returns the result of a componentwise conversion of a double3 vector from radians to degrees.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 degrees(float3 x) { return x * sfloat.FromRaw(RAD2DEG); }

        /// <summary>Returns the result of a componentwise conversion of a double4 vector from radians to degrees.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4 degrees(float4 x) { return x * sfloat.FromRaw(RAD2DEG); }


        /// <summary>Returns the minimum component of an int2 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int cmin(int2 x) { return min(x.x, x.y); }

        /// <summary>Returns the minimum component of an int3 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int cmin(int3 x) { return min(min(x.x, x.y), x.z); }

        /// <summary>Returns the minimum component of an int4 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int cmin(int4 x) { return min(min(x.x, x.y), min(x.z, x.w)); }


        /// <summary>Returns the minimum component of a uint2 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint cmin(uint2 x) { return min(x.x, x.y); }

        /// <summary>Returns the minimum component of a uint3 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint cmin(uint3 x) { return min(min(x.x, x.y), x.z); }

        /// <summary>Returns the minimum component of a uint4 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint cmin(uint4 x) { return min(min(x.x, x.y), min(x.z, x.w)); }


        /// <summary>Returns the minimum component of a float2 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat cmin(float2 x) { return min(x.x, x.y); }

        /// <summary>Returns the minimum component of a float3 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat cmin(float3 x) { return min(min(x.x, x.y), x.z); }

        /// <summary>Returns the maximum component of a float3 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat cmin(float4 x) { return min(min(x.x, x.y), min(x.z, x.w)); }


        /// <summary>Returns the maximum component of an int2 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int cmax(int2 x) { return max(x.x, x.y); }

        /// <summary>Returns the maximum component of an int3 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int cmax(int3 x) { return max(max(x.x, x.y), x.z); }

        /// <summary>Returns the maximum component of an int4 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int cmax(int4 x) { return max(max(x.x, x.y), max(x.z, x.w)); }


        /// <summary>Returns the maximum component of a uint2 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint cmax(uint2 x) { return max(x.x, x.y); }

        /// <summary>Returns the maximum component of a uint3 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint cmax(uint3 x) { return max(max(x.x, x.y), x.z); }

        /// <summary>Returns the maximum component of a uint4 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint cmax(uint4 x) { return max(max(x.x, x.y), max(x.z, x.w)); }


        /// <summary>Returns the maximum component of a float2 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat cmax(float2 x) { return max(x.x, x.y); }

        /// <summary>Returns the maximum component of a float3 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat cmax(float3 x) { return max(max(x.x, x.y), x.z); }

        /// <summary>Returns the maximum component of a float4 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat cmax(float4 x) { return max(max(x.x, x.y), max(x.z, x.w)); }


        /// <summary>Returns the horizontal sum of components of an int2 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int csum(int2 x) { return x.x + x.y; }

        /// <summary>Returns the horizontal sum of components of an int3 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int csum(int3 x) { return x.x + x.y + x.z; }

        /// <summary>Returns the horizontal sum of components of an int4 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int csum(int4 x) { return x.x + x.y + x.z + x.w; }


        /// <summary>Returns the horizontal sum of components of a uint2 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint csum(uint2 x) { return x.x + x.y; }

        /// <summary>Returns the horizontal sum of components of a uint3 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint csum(uint3 x) { return x.x + x.y + x.z; }

        /// <summary>Returns the horizontal sum of components of a uint4 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint csum(uint4 x) { return x.x + x.y + x.z + x.w; }


        /// <summary>Returns the horizontal sum of components of a float2 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat csum(float2 x) { return x.x + x.y; }

        /// <summary>Returns the horizontal sum of components of a float3 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat csum(float3 x) { return x.x + x.y + x.z; }

        /// <summary>Returns the horizontal sum of components of a float4 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static sfloat csum(float4 x) { return (x.x + x.y) + (x.z + x.w); }


        /// <summary>
        /// Packs components with an enabled mask to the left.
        /// </summary>
        /// <remarks>
        /// This function is also known as left packing. The effect of this function is to filter out components that
        /// are not enabled and leave an output buffer tightly packed with only the enabled components. A common use
        /// case is if you perform intersection tests on arrays of data in structure of arrays (SoA) form and need to
        /// produce an output array of the things that intersected.
        /// </remarks>
        /// <param name="output">Pointer to packed output array where enabled components should be stored to.</param>
        /// <param name="index">Index into output array where first enabled component should be stored to.</param>
        /// <param name="val">The value to to compress.</param>
        /// <param name="mask">Mask indicating which components are enabled.</param>
        /// <returns>Index to element after the last one stored.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe int compress(int* output, int index, int4 val, bool4 mask)
        {
            if (mask.x)
                output[index++] = val.x;
            if (mask.y)
                output[index++] = val.y;
            if (mask.z)
                output[index++] = val.z;
            if (mask.w)
                output[index++] = val.w;

            return index;
        }

        /// <summary>
        /// Packs components with an enabled mask to the left.
        /// </summary>
        /// <remarks>
        /// This function is also known as left packing. The effect of this function is to filter out components that
        /// are not enabled and leave an output buffer tightly packed with only the enabled components. A common use
        /// case is if you perform intersection tests on arrays of data in structure of arrays (SoA) form and need to
        /// produce an output array of the things that intersected.
        /// </remarks>
        /// <param name="output">Pointer to packed output array where enabled components should be stored to.</param>
        /// <param name="index">Index into output array where first enabled component should be stored to.</param>
        /// <param name="val">The value to to compress.</param>
        /// <param name="mask">Mask indicating which components are enabled.</param>
        /// <returns>Index to element after the last one stored.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe int compress(uint* output, int index, uint4 val, bool4 mask)
        {
            return compress((int*)output, index, *(int4*)&val, mask);
        }

        /// <summary>
        /// Packs components with an enabled mask to the left.
        /// </summary>
        /// <remarks>
        /// This function is also known as left packing. The effect of this function is to filter out components that
        /// are not enabled and leave an output buffer tightly packed with only the enabled components. A common use
        /// case is if you perform intersection tests on arrays of data in structure of arrays (SoA) form and need to
        /// produce an output array of the things that intersected.
        /// </remarks>
        /// <param name="output">Pointer to packed output array where enabled components should be stored to.</param>
        /// <param name="index">Index into output array where first enabled component should be stored to.</param>
        /// <param name="val">The value to to compress.</param>
        /// <param name="mask">Mask indicating which components are enabled.</param>
        /// <returns>Index to element after the last one stored.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe int compress(float* output, int index, float4 val, bool4 mask)
        {
            return compress((int*)output, index, *(int4*)&val, mask);
        }


        /// <summary>Returns a uint hash from a block of memory using the xxhash32 algorithm. Can only be used in an unsafe context.</summary>
        /// <param name="pBuffer">A pointer to the beginning of the data.</param>
        /// <param name="numBytes">Number of bytes to hash.</param>
        /// <param name="seed">Starting seed value.</param>
        public static unsafe uint hash(void* pBuffer, int numBytes, uint seed = 0)
        {
            unchecked
            {
                const uint Prime1 = 2654435761;
                const uint Prime2 = 2246822519;
                const uint Prime3 = 3266489917;
                const uint Prime4 = 668265263;
                const uint Prime5 = 374761393;

                uint4* p = (uint4*)pBuffer;
                uint hash = seed + Prime5;
                if (numBytes >= 16)
                {
                    uint4 state = new uint4(Prime1 + Prime2, Prime2, 0, (uint)-Prime1) + seed;

                    int count = numBytes >> 4;
                    for (int i = 0; i < count; ++i)
                    {
                        state += *p++ * Prime2;
                        state = (state << 13) | (state >> 19);
                        state *= Prime1;
                    }

                    hash = rol(state.x, 1) + rol(state.y, 7) + rol(state.z, 12) + rol(state.w, 18);
                }

                hash += (uint)numBytes;

                uint* puint = (uint*)p;
                for (int i = 0; i < ((numBytes >> 2) & 3); ++i)
                {
                    hash += *puint++ * Prime3;
                    hash = rol(hash, 17) * Prime4;
                }

                byte* pbyte = (byte*)puint;
                for (int i = 0; i < ((numBytes) & 3); ++i)
                {
                    hash += (*pbyte++) * Prime5;
                    hash = rol(hash, 11) * Prime1;
                }

                hash ^= hash >> 15;
                hash *= Prime2;
                hash ^= hash >> 13;
                hash *= Prime3;
                hash ^= hash >> 16;

                return hash;
            }
        }

        /// <summary>
        /// Unity's up axis (0, 1, 0).
        /// </summary>
        /// <remarks>Matches [https://docs.unity3d.com/ScriptReference/Vector3-up.html](https://docs.unity3d.com/ScriptReference/Vector3-up.html)</remarks>
        /// <returns>The up axis.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 up() { return new float3(sfloat.Zero, sfloat.One, sfloat.Zero); }  // for compatibility

        /// <summary>
        /// Unity's down axis (0, -1, 0).
        /// </summary>
        /// <remarks>Matches [https://docs.unity3d.com/ScriptReference/Vector3-down.html](https://docs.unity3d.com/ScriptReference/Vector3-down.html)</remarks>
        /// <returns>The down axis.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 down() { return new float3(sfloat.Zero, -sfloat.One, sfloat.Zero); }

        /// <summary>
        /// Unity's forward axis (0, 0, 1).
        /// </summary>
        /// <remarks>Matches [https://docs.unity3d.com/ScriptReference/Vector3-forward.html](https://docs.unity3d.com/ScriptReference/Vector3-forward.html)</remarks>
        /// <returns>The forward axis.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 forward() { return new float3(sfloat.Zero, sfloat.Zero, sfloat.One); }

        /// <summary>
        /// Unity's back axis (0, 0, -1).
        /// </summary>
        /// <remarks>Matches [https://docs.unity3d.com/ScriptReference/Vector3-back.html](https://docs.unity3d.com/ScriptReference/Vector3-back.html)</remarks>
        /// <returns>The back axis.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 back() { return new float3(sfloat.Zero, sfloat.Zero, -sfloat.One); }

        /// <summary>
        /// Unity's left axis (-1, 0, 0).
        /// </summary>
        /// <remarks>Matches [https://docs.unity3d.com/ScriptReference/Vector3-left.html](https://docs.unity3d.com/ScriptReference/Vector3-left.html)</remarks>
        /// <returns>The left axis.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 left() { return new float3(-sfloat.One, sfloat.Zero, sfloat.Zero); }

        /// <summary>
        /// Unity's right axis (1, 0, 0).
        /// </summary>
        /// <remarks>Matches [https://docs.unity3d.com/ScriptReference/Vector3-right.html](https://docs.unity3d.com/ScriptReference/Vector3-right.html)</remarks>
        /// <returns>The right axis.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 right() { return new float3(sfloat.One, sfloat.Zero, sfloat.Zero); }


        // Internal

        // SSE shuffles
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static float4 unpacklo(float4 a, float4 b)
        {
            return shuffle(a, b, ShuffleComponent.LeftX, ShuffleComponent.RightX, ShuffleComponent.LeftY, ShuffleComponent.RightY);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static float4 unpackhi(float4 a, float4 b)
        {
            return shuffle(a, b, ShuffleComponent.LeftZ, ShuffleComponent.RightZ, ShuffleComponent.LeftW, ShuffleComponent.RightW);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static float4 movelh(float4 a, float4 b)
        {
            return shuffle(a, b, ShuffleComponent.LeftX, ShuffleComponent.LeftY, ShuffleComponent.RightX, ShuffleComponent.RightY);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static float4 movehl(float4 a, float4 b)
        {
            return shuffle(b, a, ShuffleComponent.LeftZ, ShuffleComponent.LeftW, ShuffleComponent.RightZ, ShuffleComponent.RightW);
        }
    }
}
