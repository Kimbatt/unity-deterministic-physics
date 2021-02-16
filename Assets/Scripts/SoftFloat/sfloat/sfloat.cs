
// mostly from https://github.com/CodesInChaos/SoftFloat

// Copyright (c) 2011 CodesInChaos
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software
// and associated documentation files (the "Software"), to deal in the Software without restriction,
// including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
// subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies
// or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
// WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// The MIT License (MIT) - http://www.opensource.org/licenses/mit-license.php
// If you need a different license please contact me

using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

// Internal representation is identical to IEEE binary32 floating point numbers
[DebuggerDisplay("{ToStringInv()}")]
public struct sfloat : IEquatable<sfloat>, IComparable<sfloat>, IComparable, IFormattable
{
    /// <summary>
    /// Raw byte representation of an sfloat number
    /// </summary>
    private readonly uint rawValue;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private sfloat(uint raw)
    {
        rawValue = raw;
    }

    /// <summary>
    /// Creates an sfloat number from its raw byte representation
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static sfloat FromRaw(uint raw)
    {
        return new sfloat(raw);
    }

    public uint RawValue => rawValue;

    private uint RawMantissa { get { return rawValue & 0x7FFFFF; } }
    private int Mantissa
    {
        get
        {
            if (RawExponent != 0)
            {
                uint sign = (uint)((int)rawValue >> 31);
                return (int)(((RawMantissa | 0x800000) ^ sign) - sign);
            }
            else
            {
                uint sign = (uint)((int)rawValue >> 31);
                return (int)(((RawMantissa) ^ sign) - sign);
            }
        }
    }

    private sbyte Exponent => (sbyte)(RawExponent - ExponentBias);
    private byte RawExponent => (byte)(rawValue >> MantissaBits);


    private const uint SignMask = 0x80000000;
    private const int MantissaBits = 23;
    private const int ExponentBits = 8;
    private const int ExponentBias = 127;

    private const uint RawZero = 0;
    private const uint RawNaN = 0xFFC00000; // Same as float.NaN
    private const uint RawPositiveInfinity = 0x7F800000;
    private const uint RawNegativeInfinity = RawPositiveInfinity ^ SignMask;
    private const uint RawOne = 0x3F800000;
    private const uint RawMinusOne = RawOne ^ SignMask;
    private const uint RawMaxValue = 0x7F7FFFFF;
    private const uint RawMinValue = 0x7F7FFFFF ^ SignMask;
    private const uint RawEpsilon = 0x00000001;
    private const uint RawLog2OfE = 0;


    public static sfloat Zero => new sfloat(0);
    public static sfloat PositiveInfinity => new sfloat(RawPositiveInfinity);
    public static sfloat NegativeInfinity => new sfloat(RawNegativeInfinity);
    public static sfloat NaN => new sfloat(RawNaN);
    public static sfloat One => new sfloat(RawOne);
    public static sfloat MinusOne => new sfloat(RawMinusOne);
    public static sfloat MaxValue => new sfloat(RawMaxValue);
    public static sfloat MinValue => new sfloat(RawMinValue);
    public static sfloat Epsilon => new sfloat(RawEpsilon);

    public override string ToString() => ((float)this).ToString();

    /// <summary>
    /// Creates an sfloat number from its parts: sign, exponent, mantissa
    /// </summary>
    /// <param name="sign">Sign of the number: false = the number is positive, true = the number is negative</param>
    /// <param name="exponent">Exponent of the number</param>
    /// <param name="mantissa">Mantissa (significand) of the number</param>
    /// <returns></returns>
    public static sfloat FromParts(bool sign, uint exponent, uint mantissa)
    {
        return FromRaw((sign ? SignMask : 0) | ((exponent & 0xff) << MantissaBits) | (mantissa & ((1 << MantissaBits) - 1)));
    }

    /// <summary>
    /// Creates an sfloat number from a float value
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static explicit operator sfloat(float f)
    {
        uint raw = ReinterpretFloatToInt32(f);
        return new sfloat(raw);
    }

    /// <summary>
    /// Converts an sfloat number to a float value
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static explicit operator float(sfloat f)
    {
        uint raw = f.rawValue;
        return ReinterpretIntToFloat32(raw);
    }

    /// <summary>
    /// Converts an sfloat number to an integer
    /// </summary>
    public static explicit operator int(sfloat f)
    {
        if (f.Exponent < 0)
        {
            return 0;
        }

        int shift = MantissaBits - f.Exponent;
        var mantissa = (int)(f.RawMantissa | (1 << MantissaBits));
        int value = shift < 0 ? mantissa << -shift : mantissa >> shift;
        return f.IsPositive() ? value : -value;
    }

    /// <summary>
    /// Creates an sfloat number from an integer
    /// </summary>
    public static explicit operator sfloat(int value)
    {
        if (value == 0)
        {
            return Zero;
        }

        if (value == int.MinValue)
        {
            // special case
            return FromRaw(0xcf000000);
        }

        bool negative = value < 0;
        uint u = (uint)Math.Abs(value);

        int shifts;

        uint lzcnt = clz(u);
        if (lzcnt < 8)
        {
            int count = 8 - (int)lzcnt;
            u >>= count;
            shifts = -count;
        }
        else
        {
            int count = (int)lzcnt - 8;
            u <<= count;
            shifts = count;
        }

        uint exponent = (uint)(ExponentBias + MantissaBits - shifts);
        return FromParts(negative, exponent, u);
    }

    private static readonly uint[] debruijn32 = new uint[32]
    {
        0, 31, 9, 30, 3, 8, 13, 29, 2, 5, 7, 21, 12, 24, 28, 19,
        1, 10, 4, 14, 6, 22, 25, 20, 11, 15, 23, 26, 16, 27, 17, 18
    };

    /// <summary>
    /// Returns the leading zero count of the given 32-bit unsigned integer
    /// </summary>
    private static uint clz(uint x)
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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static sfloat operator -(sfloat f) => new sfloat(f.rawValue ^ 0x80000000);

    private static sfloat InternalAdd(sfloat f1, sfloat f2)
    {
        byte rawExp1 = f1.RawExponent;
        byte rawExp2 = f2.RawExponent;
        int deltaExp = rawExp1 - rawExp2;

        if (rawExp1 != 255)
        {
            //Finite
            if (deltaExp > 25)
            {
                return f1;
            }

            int man1;
            int man2;
            if (rawExp2 != 0)
            {
                // man1 = f1.Mantissa
                // http://graphics.stanford.edu/~seander/bithacks.html#ConditionalNegate
                uint sign1 = (uint)((int)f1.rawValue >> 31);
                man1 = (int)(((f1.RawMantissa | 0x800000) ^ sign1) - sign1);
                // man2 = f2.Mantissa
                uint sign2 = (uint)((int)f2.rawValue >> 31);
                man2 = (int)(((f2.RawMantissa | 0x800000) ^ sign2) - sign2);
            }
            else
            {
                // Subnorm
                // man2 = f2.Mantissa
                uint sign2 = (uint)((int)f2.rawValue >> 31);
                man2 = (int)((f2.RawMantissa ^ sign2) - sign2);

                man1 = f1.Mantissa;

                rawExp2 = 1;
                if (rawExp1 == 0)
                {
                    rawExp1 = 1;
                }

                deltaExp = rawExp1 - rawExp2;
            }

            int man = (man1 << 6) + ((man2 << 6) >> deltaExp);
            uint absMan = (uint)Math.Abs(man);
            if (absMan == 0)
            {
                return Zero;
            }

            uint msb = absMan >> MantissaBits;
            int rawExp = rawExp1 - 6;
            while (msb == 0)
            {
                rawExp -= 8;
                absMan <<= 8;
                msb = absMan >> MantissaBits;
            }

            int msbIndex = BitScanReverse8(msb);
            rawExp += msbIndex;
            absMan >>= msbIndex;
            if ((uint)(rawExp - 1) < 254)
            {
                uint raw = (uint)man & 0x80000000 | (uint)rawExp << MantissaBits | (absMan & 0x7FFFFF);
                return new sfloat(raw);
            }
            else
            {
                if (rawExp >= 255)
                {
                    //Overflow
                    return man >= 0 ? PositiveInfinity : NegativeInfinity;
                }

                if (rawExp >= -24)
                {
                    uint raw = (uint)man & 0x80000000 | absMan >> (-rawExp + 1);
                    return new sfloat(raw);
                }

                return Zero;
            }
        }
        else
        {
            // Special

            if (rawExp2 != 255)
            {
                // f1 is NaN, +Inf, -Inf and f2 is finite
                return f1;
            }

            // Both not finite
            return f1.rawValue == f2.rawValue ? f1 : NaN;
        }
    }

    public static sfloat operator +(sfloat f1, sfloat f2)
    {
        return f1.RawExponent - f2.RawExponent >= 0 ? InternalAdd(f1, f2) : InternalAdd(f2, f1);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static sfloat operator -(sfloat f1, sfloat f2) => f1 + (-f2);

    public static sfloat operator *(sfloat f1, sfloat f2)
    {
        int man1;
        int rawExp1 = f1.RawExponent;
        uint sign1;
        uint sign2;
        if (rawExp1 == 0)
        {
            // SubNorm
            sign1 = (uint)((int)f1.rawValue >> 31);
            uint rawMan1 = f1.RawMantissa;
            if (rawMan1 == 0)
            {
                if (f2.IsFinite())
                {
                    // 0 * f2
                    return new sfloat((f1.rawValue ^ f2.rawValue) & SignMask);
                }
                else
                {
                    // 0 * Infinity
                    // 0 * NaN
                    return NaN;
                }
            }

            rawExp1 = 1;
            while ((rawMan1 & 0x800000) == 0)
            {
                rawMan1 <<= 1;
                rawExp1--;
            }

            //Debug.Assert(rawMan1 >> MantissaBits == 1);
            man1 = (int)((rawMan1 ^ sign1) - sign1);
        }
        else if (rawExp1 != 255)
        {
            // Norm
            sign1 = (uint)((int)f1.rawValue >> 31);
            man1 = (int)(((f1.RawMantissa | 0x800000) ^ sign1) - sign1);
        }
        else
        {
            // Non finite
            if (f1.rawValue == RawPositiveInfinity)
            {
                if (f2.IsZero())
                {
                    // Infinity * 0
                    return NaN;
                }

                if (f2.IsNaN())
                {
                    // Infinity * NaN
                    return NaN;
                }

                if ((int)f2.rawValue >= 0)
                {
                    // Infinity * f
                    return PositiveInfinity;
                }
                else
                {
                    // Infinity * -f
                    return NegativeInfinity;
                }
            }
            else if (f1.rawValue == RawNegativeInfinity)
            {
                if (f2.IsZero() || f2.IsNaN())
                {
                    // -Infinity * 0
                    // -Infinity * NaN
                    return NaN;
                }

                if ((int)f2.rawValue < 0)
                {
                    // -Infinity * -f
                    return PositiveInfinity;
                }
                else
                {
                    // -Infinity * f
                    return NegativeInfinity;
                }
            }
            else
            {
                return f1;
            }
        }

        int man2;
        int rawExp2 = f2.RawExponent;
        if (rawExp2 == 0)
        {
            // SubNorm
            sign2 = (uint)((int)f2.rawValue >> 31);
            uint rawMan2 = f2.RawMantissa;
            if (rawMan2 == 0)
            {
                if (f1.IsFinite())
                {
                    // f1 * 0
                    return new sfloat((f1.rawValue ^ f2.rawValue) & SignMask);
                }
                else
                {
                    // Infinity * 0
                    // NaN * 0
                    return NaN;
                }
            }

            rawExp2 = 1;
            while ((rawMan2 & 0x800000) == 0)
            {
                rawMan2 <<= 1;
                rawExp2--;
            }
            //Debug.Assert(rawMan2 >> MantissaBits == 1);
            man2 = (int)((rawMan2 ^ sign2) - sign2);
        }
        else if (rawExp2 != 255)
        {
            // Norm
            sign2 = (uint)((int)f2.rawValue >> 31);
            man2 = (int)(((f2.RawMantissa | 0x800000) ^ sign2) - sign2);
        }
        else
        {
            // Non finite
            if (f2.rawValue == RawPositiveInfinity)
            {
                if (f1.IsZero())
                {
                    // 0 * Infinity
                    return NaN;
                }

                if ((int)f1.rawValue >= 0)
                {
                    // f * Infinity
                    return PositiveInfinity;
                }
                else
                {
                    // -f * Infinity
                    return NegativeInfinity;
                }
            }
            else if (f2.rawValue == RawNegativeInfinity)
            {
                if (f1.IsZero())
                {
                    // 0 * -Infinity
                    return NaN;
                }

                if ((int)f1.rawValue < 0)
                {
                    // -f * -Infinity
                    return PositiveInfinity;
                }
                else
                {
                    // f * -Infinity
                    return NegativeInfinity;
                }
            }
            else
            {
                return f2;
            }
        }

        long longMan = (long)man1 * (long)man2;
        int man = (int)(longMan >> MantissaBits);
        //Debug.Assert(man != 0);
        uint absMan = (uint)Math.Abs(man);
        int rawExp = rawExp1 + rawExp2 - ExponentBias;
        uint sign = (uint)man & 0x80000000;
        if ((absMan & 0x1000000) != 0)
        {
            absMan >>= 1;
            rawExp++;
        }

        //Debug.Assert(absMan >> MantissaBits == 1);
        if (rawExp >= 255)
        {
            // Overflow
            return new sfloat(sign ^ RawPositiveInfinity);
        }

        if (rawExp <= 0)
        {
            // Subnorms/Underflow
            if (rawExp <= -24)
            {
                return new sfloat(sign);
            }

            absMan >>= -rawExp + 1;
            rawExp = 0;
        }

        uint raw = sign | (uint)rawExp << MantissaBits | absMan & 0x7FFFFF;
        return new sfloat(raw);
    }

    public static sfloat operator /(sfloat f1, sfloat f2)
    {
        if (f1.IsNaN() || f2.IsNaN())
        {
            return NaN;
        }

        int man1;
        int rawExp1 = f1.RawExponent;
        uint sign1;
        uint sign2;
        if (rawExp1 == 0)
        {
            // SubNorm
            sign1 = (uint)((int)f1.rawValue >> 31);
            uint rawMan1 = f1.RawMantissa;
            if (rawMan1 == 0)
            {
                if (f2.IsZero())
                {
                    // 0 / 0
                    return NaN;
                }
                else
                {
                    // 0 / f
                    return new sfloat((f1.rawValue ^ f2.rawValue) & SignMask);
                }
            }

            rawExp1 = 1;
            while ((rawMan1 & 0x800000) == 0)
            {
                rawMan1 <<= 1;
                rawExp1--;
            }

            //Debug.Assert(rawMan1 >> MantissaBits == 1);
            man1 = (int)((rawMan1 ^ sign1) - sign1);
        }
        else if (rawExp1 != 255)
        {
            // Norm
            sign1 = (uint)((int)f1.rawValue >> 31);
            man1 = (int)(((f1.RawMantissa | 0x800000) ^ sign1) - sign1);
        }
        else
        {
            // Non finite
            if (f1.rawValue == RawPositiveInfinity)
            {
                if (f2.IsZero())
                {
                    // Infinity / 0
                    return PositiveInfinity;
                }

                // +-Infinity / Infinity
                return NaN;
            }
            else if (f1.rawValue == RawNegativeInfinity)
            {
                if (f2.IsZero())
                {
                    // -Infinity / 0
                    return NegativeInfinity;
                }

                // -Infinity / +-Infinity
                return NaN;
            }
            else
            {
                // NaN
                return f1;
            }
        }

        int man2;
        int rawExp2 = f2.RawExponent;
        if (rawExp2 == 0)
        {
            // SubNorm
            sign2 = (uint)((int)f2.rawValue >> 31);
            uint rawMan2 = f2.RawMantissa;
            if (rawMan2 == 0)
            {
                // f / 0
                return new sfloat(((f1.rawValue ^ f2.rawValue) & SignMask) | RawPositiveInfinity);
            }

            rawExp2 = 1;
            while ((rawMan2 & 0x800000) == 0)
            {
                rawMan2 <<= 1;
                rawExp2--;
            }

            //Debug.Assert(rawMan2 >> MantissaBits == 1);
            man2 = (int)((rawMan2 ^ sign2) - sign2);
        }
        else if (rawExp2 != 255)
        {
            // Norm
            sign2 = (uint)((int)f2.rawValue >> 31);
            man2 = (int)(((f2.RawMantissa | 0x800000) ^ sign2) - sign2);
        }
        else
        {
            // Non finite
            if (f2.rawValue == RawPositiveInfinity)
            {
                if (f1.IsZero())
                {
                    // 0 / Infinity
                    return Zero;
                }

                if ((int)f1.rawValue >= 0)
                {
                    // f / Infinity
                    return PositiveInfinity;
                }
                else
                {
                    // -f / Infinity
                    return NegativeInfinity;
                }
            }
            else if (f2.rawValue == RawNegativeInfinity)
            {
                if (f1.IsZero())
                {
                    // 0 / -Infinity
                    return new sfloat(SignMask);
                }

                if ((int)f1.rawValue < 0)
                {
                    // -f / -Infinity
                    return PositiveInfinity;
                }
                else
                {
                    // f / -Infinity
                    return NegativeInfinity;
                }
            }
            else
            {
                // NaN
                return f2;
            }
        }

        long longMan = ((long)man1 << MantissaBits) / (long)man2;
        int man = (int)longMan;
        //Debug.Assert(man != 0);
        uint absMan = (uint)Math.Abs(man);
        int rawExp = rawExp1 - rawExp2 + ExponentBias;
        uint sign = (uint)man & 0x80000000;

        if ((absMan & 0x800000) == 0)
        {
            absMan <<= 1;
            --rawExp;
        }

        //Debug.Assert(absMan >> MantissaBits == 1);
        if (rawExp >= 255)
        {
            // Overflow
            return new sfloat(sign ^ RawPositiveInfinity);
        }

        if (rawExp <= 0)
        {
            // Subnorms/Underflow
            if (rawExp <= -24)
            {
                return new sfloat(sign);
            }

            absMan >>= -rawExp + 1;
            rawExp = 0;
        }

        uint raw = sign | (uint)rawExp << MantissaBits | absMan & 0x7FFFFF;
        return new sfloat(raw);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static sfloat operator %(sfloat f1, sfloat f2) => libm.fmodf(f1, f2);

    private static readonly sbyte[] msb = new sbyte[256]
    {
        -1, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
        5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
        6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
        6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
        7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
        7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
        7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
        7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7
    };

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int BitScanReverse8(uint b) => msb[b];

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static unsafe uint ReinterpretFloatToInt32(float f) => *(uint*)&f;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static unsafe float ReinterpretIntToFloat32(uint i) => *(float*)&i;

    public override bool Equals(object obj) => obj != null && GetType() == obj.GetType() && Equals((sfloat)obj);

    public bool Equals(sfloat other)
    {
        if (RawExponent != 255)
        {
            // 0 == -0
            return (rawValue == other.rawValue) || ((rawValue & 0x7FFFFFFF) == 0) && ((other.rawValue & 0x7FFFFFFF) == 0);
        }
        else
        {
            if (RawMantissa == 0)
            {
                // Infinities
                return rawValue == other.rawValue;
            }
            else
            {
                // NaNs are equal for `Equals` (as opposed to the == operator)
                return other.RawMantissa != 0;
            }
        }
    }

    public override int GetHashCode()
    {
        if (rawValue == SignMask)
        {
            // +0 equals -0
            return 0;
        }

        if (!IsNaN())
        {
            return (int)rawValue;
        }
        else
        {
            // All NaNs are equal
            return unchecked((int)RawNaN);
        }
    }

    public static bool operator ==(sfloat f1, sfloat f2)
    {
        if (f1.RawExponent != 255)
        {
            // 0 == -0
            return (f1.rawValue == f2.rawValue) || ((f1.rawValue & 0x7FFFFFFF) == 0) && ((f2.rawValue & 0x7FFFFFFF) == 0);
        }
        else
        {
            if (f1.RawMantissa == 0)
            {
                // Infinities
                return f1.rawValue == f2.rawValue;
            }
            else
            {
                //NaNs
                return false;
            }
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(sfloat f1, sfloat f2) => !(f1 == f2);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator <(sfloat f1, sfloat f2) => !f1.IsNaN() && !f2.IsNaN() && f1.CompareTo(f2) < 0;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator >(sfloat f1, sfloat f2) => !f1.IsNaN() && !f2.IsNaN() && f1.CompareTo(f2) > 0;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator <=(sfloat f1, sfloat f2) => !f1.IsNaN() && !f2.IsNaN() && f1.CompareTo(f2) <= 0;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator >=(sfloat f1, sfloat f2) => !f1.IsNaN() && !f2.IsNaN() && f1.CompareTo(f2) >= 0;

    public int CompareTo(sfloat other)
    {
        if (IsNaN() && other.IsNaN())
        {
            return 0;
        }

        uint sign1 = (uint)((int)rawValue >> 31);
        int val1 = (int)(((rawValue) ^ (sign1 & 0x7FFFFFFF)) - sign1);

        uint sign2 = (uint)((int)other.rawValue >> 31);
        int val2 = (int)(((other.rawValue) ^ (sign2 & 0x7FFFFFFF)) - sign2);
        return val1.CompareTo(val2);
    }

    public int CompareTo(object obj) => obj is sfloat f ? CompareTo(f) : throw new ArgumentException("obj");

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IsInfinity() => (rawValue & 0x7FFFFFFF) == 0x7F800000;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IsNegativeInfinity() => rawValue == RawNegativeInfinity;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IsPositiveInfinity() => rawValue == RawPositiveInfinity;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IsNaN() => (RawExponent == 255) && !IsInfinity();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IsFinite() => RawExponent != 255;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IsZero() => (rawValue & 0x7FFFFFFF) == 0;

    public string ToString(string format, IFormatProvider formatProvider) => ((float)this).ToString(format, formatProvider);
    public string ToString(string format) => ((float)this).ToString(format);
    public string ToString(IFormatProvider provider) => ((float)this).ToString(provider);
    public string ToStringInv() => ((float)this).ToString(System.Globalization.CultureInfo.InvariantCulture);

    /// <summary>
    /// Returns the absolute value of the given sfloat number
    /// </summary>
    public static sfloat Abs(sfloat f)
    {
        if (f.RawExponent != 255 || f.IsInfinity())
        {
            return new sfloat(f.rawValue & 0x7FFFFFFF);
        }
        else
        {
            // Leave NaN untouched
            return f;
        }
    }

    /// <summary>
    /// Returns the maximum of the two given sfloat values. Returns NaN iff either argument is NaN.
    /// </summary>
    public static sfloat Max(sfloat val1, sfloat val2)
    {
        if (val1 > val2)
        {
            return val1;
        }
        else if (val1.IsNaN())
        {
            return val1;
        }
        else
        {
            return val2;
        }
    }

    /// <summary>
    /// Returns the minimum of the two given sfloat values. Returns NaN iff either argument is NaN.
    /// </summary>
    public static sfloat Min(sfloat val1, sfloat val2)
    {
        if (val1 < val2)
        {
            return val1;
        }
        else if (val1.IsNaN())
        {
            return val1;
        }
        else
        {
            return val2;
        }
    }

    /// <summary>
    /// Returns true if the sfloat number has a positive sign.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IsPositive() => (rawValue & 0x80000000) == 0;

    /// <summary>
    /// Returns true if the sfloat number has a negative sign.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IsNegative() => (rawValue & 0x80000000) != 0;

    public int Sign()
    {
        if (IsNaN())
        {
            return 0;
        }

        if (IsZero())
        {
            return 0;
        }
        else if (IsPositive())
        {
            return 1;
        }
        else
        {
            return -1;
        }
    }
}
