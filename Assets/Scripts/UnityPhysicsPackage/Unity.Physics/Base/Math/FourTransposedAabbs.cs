using UnityS.Mathematics;

namespace UnityS.Physics
{
    // 4 transposed AABBs
    public struct FourTransposedAabbs
    {
        public float4 Lx, Hx;    // Lower and upper bounds along the X axis.
        public float4 Ly, Hy;    // Lower and upper bounds along the Y axis.
        public float4 Lz, Hz;    // Lower and upper bounds along the Z axis.

        public static FourTransposedAabbs Empty => new FourTransposedAabbs
        {
            Lx = new float4(sfloat.MaxValue),
            Hx = new float4(sfloat.MinValue),
            Ly = new float4(sfloat.MaxValue),
            Hy = new float4(sfloat.MinValue),
            Lz = new float4(sfloat.MaxValue),
            Hz = new float4(sfloat.MinValue)
        };

        public void SetAllAabbs(Aabb aabb)
        {
            Lx = new float4(aabb.Min.x);
            Ly = new float4(aabb.Min.y);
            Lz = new float4(aabb.Min.z);
            Hx = new float4(aabb.Max.x);
            Hy = new float4(aabb.Max.y);
            Hz = new float4(aabb.Max.z);
        }

        public void SetAabb(int index, Aabb aabb)
        {
            Lx[index] = aabb.Min.x;
            Hx[index] = aabb.Max.x;

            Ly[index] = aabb.Min.y;
            Hy[index] = aabb.Max.y;

            Lz[index] = aabb.Min.z;
            Hz[index] = aabb.Max.z;
        }

        public Aabb GetAabb(int index) => new Aabb
        {
            Min = new float3(Lx[index], Ly[index], Lz[index]),
            Max = new float3(Hx[index], Hy[index], Hz[index])
        };

        public FourTransposedAabbs GetAabbT(int index) => new FourTransposedAabbs
        {
            Lx = new float4(Lx[index]),
            Ly = new float4(Ly[index]),
            Lz = new float4(Lz[index]),
            Hx = new float4(Hx[index]),
            Hy = new float4(Hy[index]),
            Hz = new float4(Hz[index])
        };

        public Aabb GetCompoundAabb() => new Aabb
        {
            Min = new float3(math.cmin(Lx), math.cmin(Ly), math.cmin(Lz)),
            Max = new float3(math.cmax(Hx), math.cmax(Hy), math.cmax(Hz))
        };

        public bool4 Overlap1Vs4(ref FourTransposedAabbs aabbT)
        {
            bool4 lc = (aabbT.Lx <= Hx) & (aabbT.Ly <= Hy) & (aabbT.Lz <= Hz);
            bool4 hc = (aabbT.Hx >= Lx) & (aabbT.Hy >= Ly) & (aabbT.Hz >= Lz);
            bool4 c = lc & hc;
            return c;
        }

        public bool4 Overlap1Vs4(ref FourTransposedAabbs other, int index)
        {
            FourTransposedAabbs aabbT = other.GetAabbT(index);
            return Overlap1Vs4(ref aabbT);
        }

        public float4 DistanceFromPointSquared(ref Math.FourTransposedPoints tranposedPoint)
        {
            float4 px = math.max(tranposedPoint.X, Lx);
            px = math.min(px, Hx) - tranposedPoint.X;

            float4 py = math.max(tranposedPoint.Y, Ly);
            py = math.min(py, Hy) - tranposedPoint.Y;

            float4 pz = math.max(tranposedPoint.Z, Lz);
            pz = math.min(pz, Hz) - tranposedPoint.Z;

            return px * px + py * py + pz * pz;
        }

        public float4 DistanceFromPointSquared(ref Math.FourTransposedPoints tranposedPoint, float3 scale)
        {
            float4 px = math.max(tranposedPoint.X, Lx);
            px = (math.min(px, Hx) - tranposedPoint.X) * scale.x;

            float4 py = math.max(tranposedPoint.Y, Ly);
            py = (math.min(py, Hy) - tranposedPoint.Y) * scale.y;

            float4 pz = math.max(tranposedPoint.Z, Lz);
            pz = (math.min(pz, Hz) - tranposedPoint.Z) * scale.z;

            return px * px + py * py + pz * pz;
        }

        public float4 DistanceFromAabbSquared(ref FourTransposedAabbs tranposedAabb)
        {
            float4 px = math.max(float4.zero, tranposedAabb.Lx - Hx);
            px = math.min(px, tranposedAabb.Hx - Lx);

            float4 py = math.max(float4.zero, tranposedAabb.Ly - Hy);
            py = math.min(py, tranposedAabb.Hy - Ly);

            float4 pz = math.max(float4.zero, tranposedAabb.Lz - Hz);
            pz = math.min(pz, tranposedAabb.Hz - Lz);

            return px * px + py * py + pz * pz;
        }

        public float4 DistanceFromAabbSquared(ref FourTransposedAabbs tranposedAabb, float3 scale)
        {
            float4 px = math.max(float4.zero, tranposedAabb.Lx - Hx);
            px = math.min(px, tranposedAabb.Hx - Lx) * scale.x;

            float4 py = math.max(float4.zero, tranposedAabb.Ly - Hy);
            py = math.min(py, tranposedAabb.Hy - Ly) * scale.y;

            float4 pz = math.max(float4.zero, tranposedAabb.Lz - Hz);
            pz = math.min(pz, tranposedAabb.Hz - Lz) * scale.z;

            return px * px + py * py + pz * pz;
        }

        public bool4 Raycast(Ray ray, sfloat maxFraction, out float4 fractions)
        {
            float4 lx = Lx - new float4(ray.Origin.x);
            float4 hx = Hx - new float4(ray.Origin.x);
            float4 nearXt = lx * new float4(ray.ReciprocalDisplacement.x);
            float4 farXt = hx * new float4(ray.ReciprocalDisplacement.x);

            float4 ly = Ly - new float4(ray.Origin.y);
            float4 hy = Hy - new float4(ray.Origin.y);
            float4 nearYt = ly * new float4(ray.ReciprocalDisplacement.y);
            float4 farYt = hy * new float4(ray.ReciprocalDisplacement.y);

            float4 lz = Lz - new float4(ray.Origin.z);
            float4 hz = Hz - new float4(ray.Origin.z);
            float4 nearZt = lz * new float4(ray.ReciprocalDisplacement.z);
            float4 farZt = hz * new float4(ray.ReciprocalDisplacement.z);

            float4 nearX = math.min(nearXt, farXt);
            float4 farX = math.max(nearXt, farXt);

            float4 nearY = math.min(nearYt, farYt);
            float4 farY = math.max(nearYt, farYt);

            float4 nearZ = math.min(nearZt, farZt);
            float4 farZ = math.max(nearZt, farZt);

            float4 nearMax = math.max(math.max(math.max(nearX, nearY), nearZ), float4.zero);
            float4 farMin = math.min(math.min(math.min(farX, farY), farZ), new float4(maxFraction));

            fractions = nearMax;

            return (nearMax <= farMin) & (lx <= hx);
        }
    }
}
