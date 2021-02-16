
public static partial class libm
{
    /// <summary>
    /// Returns the remainder and the quotient when dividing x by y, so that x == y * quotient + remainder
    /// </summary>
    public static void remquof(sfloat x, sfloat y, out sfloat remainder, out int quotient)
    {
        uint ux = x.RawValue;
        uint uy = y.RawValue;
        int ex = (int)((ux >> 23) & 0xff);
        int ey = (int)((uy >> 23) & 0xff);
        bool sx = (ux >> 31) != 0;
        bool sy = (uy >> 31) != 0;
        uint q;
        uint i;
        var uxi = ux;

        if ((uy << 1) == 0 || y.IsNaN() || ex == 0xff)
        {
            sfloat m = (x * y);
            remainder = m / m;
            quotient = 0;
            return;
        }

        if ((ux << 1) == 0)
        {
            remainder = x;
            quotient = 0;
            return;
        }

        /* normalize x and y */
        if (ex == 0)
        {
            i = uxi << 9;
            while ((i >> 31) == 0)
            {
                ex -= 1;
                i <<= 1;
            }

            uxi <<= -ex + 1;
        }
        else
        {
            uxi &= (~0u) >> 9;
            uxi |= 1 << 23;
        }

        if (ey == 0)
        {
            i = uy << 9;
            while ((i >> 31) == 0)
            {
                ey -= 1;
                i <<= 1;
            }

            uy <<= -ey + 1;
        }
        else
        {
            uy &= (~0u) >> 9;
            uy |= 1 << 23;
        }

        q = 0;
        if (ex + 1 != ey)
        {
            if (ex < ey)
            {
                remainder = x;
                quotient = 0;
                return;
            }

            /* x mod y */
            while (ex > ey)
            {
                i = uxi - uy;
                if ((i >> 31) == 0)
                {
                    uxi = i;
                    q += 1;
                }

                uxi <<= 1;
                q <<= 1;
                ex -= 1;
            }

            i = uxi - uy;
            if ((i >> 31) == 0)
            {
                uxi = i;
                q += 1;
            }

            if (uxi == 0)
            {
                ex = -30;
            }
            else
            {
                while ((uxi >> 23) == 0)
                {
                    uxi <<= 1;
                    ex -= 1;
                }
            }
        }

        /* scale result and decide between |x| and |x|-|y| */
        if (ex > 0)
        {
            uxi -= 1 << 23;
            uxi |= ((uint)ex) << 23;
        }
        else
        {
            uxi >>= -ex + 1;
        }

        x = sfloat.FromRaw(uxi);
        if (sy)
        {
            y = -y;
        }

        if ((ex == ey || (ex + 1 == ey && ((sfloat)2.0f * x > y || ((sfloat)2.0f * x == y && (q % 2) != 0)))) && x > y)
        {
            x -= y;
            q += 1;
        }

        q &= 0x7fffffff;
        int quo = sx ^ sy ? -(int)q : (int)q;
        remainder = sx ? -x : x;
        quotient = quo;
    }

    /// <summary>
    /// Returns the remainder when dividing x by y
    /// </summary>
    public static sfloat remainderf(sfloat x, sfloat y)
    {
        remquof(x, y, out sfloat remainder, out _);
        return remainder;
    }

    /// <summary>
    /// Returns x modulo y
    /// </summary>
    public static sfloat fmodf(sfloat x, sfloat y)
    {
        uint uxi = x.RawValue;
        uint uyi = y.RawValue;
        int ex = (int)(uxi >> 23 & 0xff);
        int ey = (int)(uyi >> 23 & 0xff);
        uint sx = uxi & 0x80000000;
        uint i;

        if (uyi << 1 == 0 || y.IsNaN() || ex == 0xff)
        {
            return (x * y) / (x * y);
        }

        if (uxi << 1 <= uyi << 1)
        {
            if (uxi << 1 == uyi << 1)
            {
                //return 0.0 * x;
                return sfloat.Zero;
            }

            return x;
        }

        /* normalize x and y */
        if (ex == 0)
        {
            i = uxi << 9;
            while (i >> 31 == 0)
            {
                ex -= 1;
                i <<= 1;
            }

            uxi <<= -ex + 1;
        }
        else
        {
            uxi &= uint.MaxValue >> 9;
            uxi |= 1 << 23;
        }

        if (ey == 0)
        {
            i = uyi << 9;
            while (i >> 31 == 0)
            {
                ey -= 1;
                i <<= 1;
            }

            uyi <<= -ey + 1;
        }
        else
        {
            uyi &= uint.MaxValue >> 9;
            uyi |= 1 << 23;
        }

        /* x mod y */
        while (ex > ey)
        {
            i = uxi - uyi;
            if (i >> 31 == 0)
            {
                if (i == 0)
                {
                    //return 0.0 * x;
                    return sfloat.Zero;
                }

                uxi = i;
            }

            uxi <<= 1;

            ex -= 1;
        }

        i = uxi - uyi;
        if (i >> 31 == 0)
        {
            if (i == 0)
            {
                //return 0.0 * x;
                return sfloat.Zero;
            }

            uxi = i;
        }

        while (uxi >> 23 == 0)
        {
            uxi <<= 1;
            ex -= 1;
        }

        /* scale result up */
        if (ex > 0)
        {
            uxi -= 1 << 23;
            uxi |= ((uint)ex) << 23;
        }
        else
        {
            uxi >>= -ex + 1;
        }

        uxi |= sx;
        return sfloat.FromRaw(uxi);
    }

    /// <summary>
    /// Rounds x to the nearest integer
    /// </summary>
    public static sfloat roundf(sfloat x)
    {
        sfloat TOINT = (sfloat)8388608.0f;

        uint i = x.RawValue;
        uint e = i >> 23 & 0xff;
        sfloat y;

        if (e >= 0x7f + 23)
        {
            return x;
        }

        if (e < 0x7f - 1)
        {
            //force_eval!(x + TOINT);
            //return 0.0 * x;
            return sfloat.Zero;
        }

        if (i >> 31 != 0)
        {
            x = -x;
        }

        y = x + TOINT - TOINT - x;

        if (y > (sfloat)0.5f)
        {
            y = y + x - sfloat.One;
        }
        else if (y <= (sfloat)(-0.5f))
        {
            y = y + x + sfloat.One;
        }
        else
        {
            y += x;
        }

        return i >> 31 != 0 ? -y : y;
    }

    /// <summary>
    /// Rounds x down to the nearest integer
    /// </summary>
    public static sfloat floorf(sfloat x)
    {
        uint ui = x.RawValue;
        int e = (((int)(ui >> 23)) & 0xff) - 0x7f;

        if (e >= 23)
        {
            return x;
        }

        if (e >= 0)
        {
            uint m = 0x007fffffu >> e;
            if ((ui & m) == 0)
            {
                return x;
            }
            if (ui >> 31 != 0)
            {
                ui += m;
            }
            ui &= ~m;
        }
        else
        {
            if (ui >> 31 == 0)
            {
                ui = 0;
            }
            else if (ui << 1 != 0)
            {
                return (sfloat)(-1.0f);
            }
        }

        return sfloat.FromRaw(ui);
    }

    /// <summary>
    /// Rounds x up to the nearest integer
    /// </summary>
    public static sfloat ceilf(sfloat x)
    {
        uint ui = x.RawValue;
        int e = (int)(((ui >> 23) & 0xff) - (0x7f));

        if (e >= 23)
        {
            return x;
        }

        if (e >= 0)
        {
            uint m = 0x007fffffu >> e;
            if ((ui & m) == 0)
            {
                return x;
            }
            if (ui >> 31 == 0)
            {
                ui += m;
            }
            ui &= ~m;
        }
        else
        {
            if (ui >> 31 != 0)
            {
                return (sfloat)(-0.0f);
            }
            else if (ui << 1 != 0)
            {
                return sfloat.One;
            }
        }

        return sfloat.FromRaw(ui);
    }

    /// <summary>
    /// Truncates x, removing its fractional parts
    /// </summary>
    public static sfloat truncf(sfloat x)
    {
        uint i = x.RawValue;
        int e = (int)(i >> 23 & 0xff) - 0x7f + 9;
        uint m;

        if (e >= 23 + 9)
        {
            return x;
        }

        if (e < 9)
        {
            e = 1;
        }

        m = unchecked((uint)-1) >> e;
        if ((i & m) == 0)
        {
            return x;
        }

        i &= ~m;
        return sfloat.FromRaw(i);
    }

    /// <summary>
    /// Returns the square root of x
    /// </summary>
    public static sfloat sqrtf(sfloat x)
    {
        int sign = unchecked((int)0x80000000);
        int ix;
        int s;
        int q;
        int m;
        int t;
        int i;
        uint r;

        ix = (int)x.RawValue;

        /* take care of Inf and NaN */
        if (((uint)ix & 0x7f800000) == 0x7f800000)
        {
            //return x * x + x; /* sqrt(NaN)=NaN, sqrt(+inf)=+inf, sqrt(-inf)=sNaN */
            if (x.IsNaN() || x.IsNegativeInfinity())
            {
                return sfloat.NaN;
            }
            else // if (x.IsPositiveInfinity())
            {
                return sfloat.PositiveInfinity;
            }
        }

        /* take care of zero */
        if (ix <= 0)
        {
            if ((ix & ~sign) == 0)
            {
                return x; /* sqrt(+-0) = +-0 */
            }

            if (ix < 0)
            {
                //return (x - x) / (x - x); /* sqrt(-ve) = sNaN */
                return sfloat.NaN;
            }
        }

        /* normalize x */
        m = ix >> 23;
        if (m == 0)
        {
            /* subnormal x */
            i = 0;
            while ((ix & 0x00800000) == 0)
            {
                ix <<= 1;
                i += 1;
            }

            m -= i - 1;
        }

        m -= 127; /* unbias exponent */
        ix = (ix & 0x007fffff) | 0x00800000;
        if ((m & 1) == 1)
        {
            /* odd m, double x to make it even */
            ix += ix;
        }

        m >>= 1; /* m = [m/2] */

        /* generate sqrt(x) bit by bit */
        ix += ix;
        q = 0;
        s = 0;
        r = 0x01000000; /* r = moving bit from right to left */

        while (r != 0)
        {
            t = s + (int)r;
            if (t <= ix)
            {
                s = t + (int)r;
                ix -= t;
                q += (int)r;
            }

            ix += ix;
            r >>= 1;
        }

        /* use floating add to find out rounding direction */
        if (ix != 0)
        {
            q += q & 1;
        }

        ix = (q >> 1) + 0x3f000000;
        ix += m << 23;
        return sfloat.FromRaw((uint)ix);
    }
}
