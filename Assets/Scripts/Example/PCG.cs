
// PCG random-number generator
// see https://www.pcg-random.org/ for more info

public struct PCG
{
    private ulong state;
    private readonly ulong seed;

    private void Advance()
    {
        state = state * 6364136223846793005 + seed;
    }

    // [0, 4294967295]
    public uint Uint32()
    {
        ulong oldstate = state;
        Advance();
        // top 5 bits specify the rotation, leading to the following constants:
        // 64 - 5 = 59
        // 32 - 5 = 27
        // (32 + 5) / 2 = 18
        uint xorshifted = (uint)(((oldstate >> 18) ^ oldstate) >> 27);
        int rot = (int)(oldstate >> 59);
        return (xorshifted >> rot) | (xorshifted << (32 - rot));
    }

    // [-2147483648, 2147483647]
    public int Int32()
    {
        return (int)(((long)Uint32()) - int.MaxValue);
    }

    // [0, 2147483647]
    public int Int32Positive()
    {
        return (int)Uint32();
    }

    // [min, max]
    public uint Uint32RangeInclusive(uint minInclusive, uint maxInclusive)
    {
        return (Uint32() % (maxInclusive + 1 - minInclusive)) + minInclusive;
    }

    // [min, max]
    public int Int32RangeInclusive(int minInclusive, int maxInclusive)
    {
        return (Int32Positive() % (maxInclusive + 1 - minInclusive)) + minInclusive;
    }

    // [0, max)
    public uint Uint32(uint maxExclusive)
    {
        return Uint32() % maxExclusive;
    }

    // [min, max)
    public uint Uint32(uint minInclusive, uint maxExclusive)
    {
        return (Uint32() % (maxExclusive - minInclusive)) + minInclusive;
    }

    // [0, 18446744073709551615]
    public ulong Uint64()
    {
        return (((ulong)Uint32()) << 32) | Uint32();
    }

    // [0, max)
    public ulong Uint64(ulong maxExclusive)
    {
        return Uint64() % maxExclusive;
    }

    public PCG(ulong state, ulong seed)
    {
        this.seed = (seed << 1) | 1u;
        this.state = (this.seed + state) * 6364136223846793005 + this.seed;
        Advance();
        Advance();
    }

    public sfloat SFloatInclusive(sfloat minInclusive, sfloat maxInclusive)
    {
        sfloat diff = maxInclusive - minInclusive;
        sfloat rand = (sfloat)Uint32(16777216) / sfloat.FromRaw(0x4b7fffff);
        return diff * rand + minInclusive;
    }

    public sfloat SFloatExclusive(sfloat minInclusive, sfloat maxExclusive)
    {
        sfloat diff = maxExclusive - minInclusive;
        sfloat rand = (sfloat)Uint32(16777216) / sfloat.FromRaw(0x4b800000);
        return diff * rand + minInclusive;
    }
}