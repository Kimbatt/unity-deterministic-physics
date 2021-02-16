
public static partial class libm
{
    const uint pi = 0x40490fdb; // 3.1415926535897932384626433832795
    const uint half_pi = 0x3fc90fdb; // 1.5707963267948966192313216916398
    const uint two_pi = 0x40c90fdb; // 6.283185307179586476925286766559
    const uint pi_over_4 = 0x3f490fdb; // 0.78539816339744830961566084581988
    const uint pi_times_3_over_4 = 0x4016cbe4; // 2.3561944901923449288469825374596

    /// <summary>
    /// Returns the sine of x
    /// </summary>
    public static sfloat sinf(sfloat x)
    {
        const uint pi_squared_times_five = 0x42456460; // 49.348022005446793094172454999381

        // https://en.wikipedia.org/wiki/Bhaskara_I%27s_sine_approximation_formula
        // sin(x) ~= (16x * (pi - x)) / (5pi^2 - 4x * (pi - x)) if 0 <= x <= pi

        // move x into range
        x %= sfloat.FromRaw(two_pi);
        if (x.IsNegative())
        {
            x += sfloat.FromRaw(two_pi);
        }

        bool negate;
        if (x > sfloat.FromRaw(pi))
        {
            // pi < x <= 2pi, we need to move x to the 0 <= x <= pi range
            // also, we need to negate the result before returning it
            x = sfloat.FromRaw(two_pi) - x;
            negate = true;
        }
        else
        {
            negate = false;
        }

        sfloat piMinusX = sfloat.FromRaw(pi) - x;
        sfloat result = ((sfloat)16.0f * x * piMinusX) / (sfloat.FromRaw(pi_squared_times_five) - (sfloat)4.0f * x * piMinusX);
        return negate ? -result : result;
    }

    /// <summary>
    /// Returns the cosine of x
    /// </summary>
    public static sfloat cosf(sfloat x) => sinf(x + sfloat.FromRaw(half_pi));

    /// <summary>
    /// Returns the tangent of x
    /// </summary>
    public static sfloat tanf(sfloat x) => sinf(x) / cosf(x);

    /// <summary>
    /// Returns the square root of (x*x + y*y)
    /// </summary>
    public static sfloat hypotf(sfloat x, sfloat y)
    {
        sfloat w;

        int ha = (int)x.RawValue;
        ha &= 0x7fffffff;

        int hb = (int)y.RawValue;
        hb &= 0x7fffffff;

        if (hb > ha)
        {
            int temp = ha;
            ha = hb;
            hb = temp;
        }

        sfloat a = sfloat.FromRaw((uint)ha); /* a <- |a| */
        sfloat b = sfloat.FromRaw((uint)hb); /* b <- |b| */

        if (ha - hb > 0xf000000)
        {
            return a + b;
        } /* x/y > 2**30 */

        uint k = 0;
        if (ha > 0x58800000)
        {
            /* a>2**50 */
            if (ha >= 0x7f800000)
            {
                /* Inf or NaN */
                w = a + b; /* for sNaN */
                if (ha == 0x7f800000)
                {
                    w = a;
                }

                if (hb == 0x7f800000)
                {
                    w = b;
                }

                return w;
            }

            /* scale a and b by 2**-60 */
            ha -= 0x5d800000;
            hb -= 0x5d800000;
            k += 60;
            a = sfloat.FromRaw((uint)ha);
            b = sfloat.FromRaw((uint)hb);
        }

        if (hb < 0x26800000)
        {
            /* b < 2**-50 */
            if (hb <= 0x007fffff)
            {
                /* subnormal b or 0 */
                if (hb == 0)
                {
                    return a;
                }

                sfloat t1 = sfloat.FromRaw(0x3f000000); /* t1=2^126 */
                b *= t1;
                a *= t1;
                k -= 126;
            }
            else
            {
                /* scale a and b by 2^60 */
                ha += 0x5d800000; /* a *= 2^60 */
                hb += 0x5d800000; /* b *= 2^60 */
                k -= 60;
                a = sfloat.FromRaw((uint)ha);
                b = sfloat.FromRaw((uint)hb);
            }
        }

        /* medium size a and b */
        w = a - b;
        if (w > b)
        {
            sfloat t1 = sfloat.FromRaw(((uint)ha) & 0xfffff000);
            sfloat t2 = a - t1;
            w = sqrtf(t1 * t1 - (b * (-b) - t2 * (a + t1)));
        }
        else
        {
            a += a;
            sfloat y1 = sfloat.FromRaw(((uint)hb) & 0xfffff000);
            sfloat y2 = b - y1;
            sfloat t1 = sfloat.FromRaw(((uint)ha) + 0x00800000);
            sfloat t2 = a - t1;
            w = sqrtf(t1 * y1 - (w * (-w) - (t1 * y2 + t2 * b)));
        }

        if (k != 0)
        {
            sfloat t1 = sfloat.FromRaw(0x3f800000 + (k << 23));
            return t1 * w;
        }
        else
        {
            return w;
        };
    }


    private static readonly uint[] ATAN_HI = new uint[4]
    {
        0x3eed6338, // 4.6364760399e-01, /* atan(0.5)hi */
        0x3f490fda, // 7.8539812565e-01, /* atan(1.0)hi */
        0x3f7b985e, // 9.8279368877e-01, /* atan(1.5)hi */
        0x3fc90fda, // 1.5707962513e+00, /* atan(inf)hi */
    };

    private static readonly uint[] ATAN_LO = new uint[4]
    {
        0x31ac3769, // 5.0121582440e-09, /* atan(0.5)lo */
        0x33222168, // 3.7748947079e-08, /* atan(1.0)lo */
        0x33140fb4, // 3.4473217170e-08, /* atan(1.5)lo */
        0x33a22168, // 7.5497894159e-08, /* atan(inf)lo */
    };

    private static readonly uint[] A_T = new uint[5]
    {
        0x3eaaaaa9, // 3.3333328366e-01
        0xbe4cca98, // -1.9999158382e-01
        0x3e11f50d, // 1.4253635705e-01
        0xbdda1247, // -1.0648017377e-01
        0x3d7cac25  // 6.1687607318e-02
    };

    /// <summary>
    /// Returns the arctangent of x
    /// </summary>
    public unsafe static sfloat atanf(sfloat x)
    {
        sfloat z;

        uint ix = x.RawValue;
        bool sign = (ix >> 31) != 0;
        ix &= 0x7fffffff;

        if (ix >= 0x4c800000)
        {
            /* if |x| >= 2**26 */
            if (x.IsNaN())
            {
                return x;
            }

            sfloat x1p_120 = sfloat.FromRaw(0x03800000); // 0x1p-120 === 2 ^ (-120)
            z = sfloat.FromRaw(ATAN_HI[3]) + x1p_120;
            return sign ? -z : z;
        }

        int id;
        if (ix < 0x3ee00000)
        {
            /* |x| < 0.4375 */
            if (ix < 0x39800000)
            {
                /* |x| < 2**-12 */
                //if (ix < 0x00800000)
                //{
                //    /* raise underflow for subnormal x */
                //    force_eval!(x * x);
                //}
                return x;
            }
            id = -1;
        }
        else
        {
            x = sfloat.Abs(x);
            if (ix < 0x3f980000)
            {
                /* |x| < 1.1875 */
                if (ix < 0x3f300000)
                {
                    /*  7/16 <= |x| < 11/16 */
                    x = ((sfloat)2.0f * x - (sfloat)1.0f) / ((sfloat)2.0f + x);
                    id = 0;
                }
                else
                {
                    /* 11/16 <= |x| < 19/16 */
                    x = (x - (sfloat)1.0f) / (x + (sfloat)1.0f);
                    id = 1;
                }
            }
            else if (ix < 0x401c0000)
            {
                /* |x| < 2.4375 */
                x = (x - (sfloat)1.5f) / ((sfloat)1.0f + (sfloat)1.5f * x);
                id = 2;
            }
            else
            {
                /* 2.4375 <= |x| < 2**26 */
                x = (sfloat)(-1.0f) / x;
                id = 3;
            }
        };

        /* end of argument reduction */
        z = x * x;
        sfloat w = z * z;

        /* break sum from i=0 to 10 aT[i]z**(i+1) into odd and even poly */
        sfloat s1 = z * (sfloat.FromRaw(A_T[0]) + w * (sfloat.FromRaw(A_T[2]) + w * sfloat.FromRaw(A_T[4])));
        sfloat s2 = w * (sfloat.FromRaw(A_T[1]) + w * sfloat.FromRaw(A_T[3]));
        if (id < 0)
        {
            return x - x * (s1 + s2);
        }

        z = sfloat.FromRaw(ATAN_HI[id]) - ((x * (s1 + s2) - sfloat.FromRaw(ATAN_LO[id])) - x);
        return sign ? -z : z;
    }

    /// <summary>
    /// Returns the signed angle between the positive x axis, and the direction (x, y)
    /// </summary>
    public static sfloat atan2f(sfloat y, sfloat x)
    {
        if (x.IsNaN() || y.IsNaN())
        {
            return x + y;
        }

        uint ix = x.RawValue;
        uint iy = y.RawValue;

        if (ix == 0x3f800000)
        {
            /* x=1.0 */
            return atanf(y);
        }

        uint m = ((iy >> 31) & 1) | ((ix >> 30) & 2); /* 2*sign(x)+sign(y) */
        ix &= 0x7fffffff;
        iy &= 0x7fffffff;

        const uint PI_LO_U32 = 0xb3bbbd2e; // -8.7422776573e-08

        /* when y = 0 */
        if (iy == 0)
        {
            switch (m)
            {
                case 0:
                case 1:
                    return y; /* atan(+-0,+anything)=+-0 */
                case 2:
                    return sfloat.FromRaw(pi); /* atan(+0,-anything) = pi */
                case 3:
                default:
                    return -sfloat.FromRaw(pi); /* atan(-0,-anything) =-pi */
            }
        }

        /* when x = 0 */
        if (ix == 0)
        {
            return (m & 1) != 0 ? -sfloat.FromRaw(half_pi) : sfloat.FromRaw(half_pi);
        }

        /* when x is INF */
        if (ix == 0x7f800000)
        {
            if (iy == 0x7f800000)
            {
                switch (m)
                {
                    case 0:
                        return sfloat.FromRaw(pi_over_4); /* atan(+INF,+INF) */
                    case 1:
                        return -sfloat.FromRaw(pi_over_4); /* atan(-INF,+INF) */
                    case 2:
                        return sfloat.FromRaw(pi_times_3_over_4); /* atan(+INF,-INF)*/
                    case 3:
                    default:
                        return -sfloat.FromRaw(pi_times_3_over_4); /* atan(-INF,-INF)*/
                }
            }
            else
            {
                switch (m)
                {
                    case 0:
                        return sfloat.Zero; /* atan(+...,+INF) */
                    case 1:
                        return -sfloat.Zero; /* atan(-...,+INF) */
                    case 2:
                        return sfloat.FromRaw(pi); /* atan(+...,-INF) */
                    case 3:
                    default:
                        return -sfloat.FromRaw(pi); /* atan(-...,-INF) */
                }
            }
        }

        /* |y/x| > 0x1p26 */
        if (ix + (26 << 23) < iy || iy == 0x7f800000)
        {
            return (m & 1) != 0 ? -sfloat.FromRaw(half_pi) : sfloat.FromRaw(half_pi);
        }

        /* z = atan(|y/x|) with correct underflow */
        sfloat z = (m & 2) != 0 && iy + (26 << 23) < ix
            ? sfloat.Zero /*|y/x| < 0x1p-26, x < 0 */
            : atanf(sfloat.Abs(y / x));

        switch (m)
        {
            case 0:
                return z; /* atan(+,+) */
            case 1:
                return -z; /* atan(-,+) */
            case 2:
                return sfloat.FromRaw(pi) - (z - sfloat.FromRaw(PI_LO_U32)); /* atan(+,-) */
            case 3:
            default:
                return (z - sfloat.FromRaw(PI_LO_U32)) - sfloat.FromRaw(pi); /* atan(-,-) */
        }
    }

    /// <summary>
    /// Returns the arccosine of x
    /// </summary>
    public static sfloat acosf(sfloat x)
    {
        const uint PIO2_HI_U32 = 0x3fc90fda; // 1.5707962513e+00
        const uint PIO2_LO_U32 = 0x33a22168; // 7.5497894159e-08
        const uint P_S0_U32 = 0x3e2aaa75; // 1.6666586697e-01
        const uint P_S1_U32 = 0xbd2f13ba; // -4.2743422091e-02
        const uint P_S2_U32 = 0xbc0dd36b; // -8.6563630030e-03
        const uint Q_S1_U32 = 0xbf34e5ae; // - 7.0662963390e-01

        static sfloat r(sfloat z)
        {
            sfloat p = z * (sfloat.FromRaw(P_S0_U32) + z * (sfloat.FromRaw(P_S1_U32) + z * sfloat.FromRaw(P_S2_U32)));
            sfloat q = (sfloat)1.0f + z * sfloat.FromRaw(Q_S1_U32);
            return p / q;
        }

        sfloat x1p_120 = sfloat.FromRaw(0x03800000); // 0x1p-120 === 2 ^ (-120)

        sfloat z;
        sfloat w;
        sfloat s;

        uint hx = x.RawValue;
        uint ix = hx & 0x7fffffff;

        /* |x| >= 1 or nan */
        if (ix >= 0x3f800000)
        {
            if (ix == 0x3f800000)
            {
                if ((hx >> 31) != 0)
                {
                    return (sfloat)2.0f * sfloat.FromRaw(PIO2_HI_U32) + x1p_120;
                }

                return sfloat.Zero;
            }

            return sfloat.NaN;
        }

        /* |x| < 0.5 */
        if (ix < 0x3f000000)
        {
            if (ix <= 0x32800000)
            {
                /* |x| < 2**-26 */
                return sfloat.FromRaw(PIO2_HI_U32) + x1p_120;
            }

            return sfloat.FromRaw(PIO2_HI_U32) - (x - (sfloat.FromRaw(PIO2_LO_U32) - x * r(x * x)));
        }

        /* x < -0.5 */
        if ((hx >> 31) != 0)
        {
            z = ((sfloat)1.0f + x) * (sfloat)0.5f;
            s = sqrtf(z);
            w = r(z) * s - sfloat.FromRaw(PIO2_LO_U32);
            return (sfloat)2.0 * (sfloat.FromRaw(PIO2_HI_U32) - (s + w));
        }

        /* x > 0.5 */
        z = ((sfloat)1.0f - x) * (sfloat)0.5f;
        s = sqrtf(z);
        hx = s.RawValue;
        sfloat df = sfloat.FromRaw(hx & 0xfffff000);
        sfloat c = (z - df * df) / (s + df);
        w = r(z) * s + c;
        return (sfloat)2.0f * (df + w);
    }

    /// <summary>
    /// Returns the arcsine of x
    /// </summary>
    public static sfloat asinf(sfloat x) => sfloat.FromRaw(half_pi) - acosf(x);
}
