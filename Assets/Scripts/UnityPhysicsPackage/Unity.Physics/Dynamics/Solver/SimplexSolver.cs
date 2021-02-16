using Unity.Collections;
using UnityS.Mathematics;
using UnityEngine.Assertions;

namespace UnityS.Physics
{
    public struct SurfaceConstraintInfo // TODO: Move into SimplexSolver
    {
        // Info of interest for the character
        public Plane Plane;
        public float3 Velocity;

        // Hit body info
        public int RigidBodyIndex;
        public ColliderKey ColliderKey;
        public float3 HitPosition;

        // Internal state
        public int Priority;
        public bool Touched;
        public bool IsTooSteep;
        public bool IsMaxSlope;
    }

    public static class SimplexSolver
    {
        static sfloat k_Epsilon => sfloat.FromRaw(0x38d1b717);

        public static unsafe void Solve(
            sfloat deltaTime, sfloat minDeltaTime, float3 up, sfloat maxVelocity,
            NativeList<SurfaceConstraintInfo> constraints, ref float3 position, ref float3 velocity, out sfloat integratedTime, bool useConstraintVelocities = true
        )
        {
            // List of planes to solve against (up to 4)
            SurfaceConstraintInfo* supportPlanes = stackalloc SurfaceConstraintInfo[4];
            int numSupportPlanes = 0;

            sfloat remainingTime = deltaTime;
            sfloat currentTime = sfloat.Zero;

            // Clamp the input velocity to max movement speed
            ClampToMaxLength(maxVelocity, ref velocity);

            while (remainingTime > sfloat.Zero)
            {
                int hitIndex = -1;
                sfloat minCollisionTime = remainingTime;

                // Iterate over constraints and solve them
                for (int i = 0; i < constraints.Length; i++)
                {
                    if (constraints[i].Touched) continue;

                    SurfaceConstraintInfo constraint = constraints[i];

                    float3 relVel = velocity - (useConstraintVelocities ? constraint.Velocity : float3.zero);
                    sfloat relProjVel = -math.dot(relVel, constraint.Plane.Normal);
                    if (relProjVel < k_Epsilon)
                    {
                        continue;
                    }

                    // Clamp distance to 0, since penetration is handled by constraint.Velocity already
                    sfloat distance = math.max(constraint.Plane.Distance, sfloat.Zero);
                    if (distance < minCollisionTime * relProjVel)
                    {
                        minCollisionTime = distance / relProjVel;
                        hitIndex = i;
                    }
                }

                // Integrate
                {
                    currentTime += minCollisionTime;
                    remainingTime -= minCollisionTime;
                    position += minCollisionTime * velocity;
                }

                if (hitIndex < 0 || currentTime > minDeltaTime)
                {
                    break;
                }

                // Mark constraint as touched
                {
                    var constraint = constraints[hitIndex];
                    constraint.Touched = true;
                    constraints[hitIndex] = constraint;
                }

                // Add the hit to the current list of active planes
                supportPlanes[numSupportPlanes] = constraints[hitIndex];
                if (!useConstraintVelocities)
                {
                    supportPlanes[numSupportPlanes].Velocity = float3.zero;
                }
                numSupportPlanes++;

                // Solve support planes
                ExamineActivePlanes(up, supportPlanes, ref numSupportPlanes, ref velocity);

                // Clamp the solved velocity to max movement speed
                ClampToMaxLength(maxVelocity, ref velocity);

                // Can't handle more than 4 support planes
                if (numSupportPlanes == 4)
                {
                    break;
                }
            }

            integratedTime = currentTime;
        }

        public static unsafe void ExamineActivePlanes(float3 up, SurfaceConstraintInfo* supportPlanes, ref int numSupportPlanes, ref float3 velocity)
        {
            switch (numSupportPlanes)
            {
                case 1:
                    {
                        Solve1d(supportPlanes[0], ref velocity);
                        return;
                    }
                case 2:
                    {
                        // Test whether we need plane 0 at all
                        float3 tempVelocity = velocity;
                        Solve1d(supportPlanes[1], ref tempVelocity);

                        bool plane0Used = Test1d(supportPlanes[0], tempVelocity);
                        if (!plane0Used)
                        {
                            // Compact the buffer and reduce size
                            supportPlanes[0] = supportPlanes[1];
                            numSupportPlanes = 1;

                            // Write back the result
                            velocity = tempVelocity;
                        }
                        else
                        {
                            Solve2d(up, supportPlanes[0], supportPlanes[1], ref velocity);
                        }

                        return;
                    }
                case 3:
                    {
                        // Try to drop both planes
                        float3 tempVelocity = velocity;
                        Solve1d(supportPlanes[2], ref tempVelocity);

                        bool plane0Used = Test1d(supportPlanes[0], tempVelocity);
                        if (!plane0Used)
                        {
                            bool plane1Used = Test1d(supportPlanes[1], tempVelocity);
                            if (!plane1Used)
                            {
                                // Compact the buffer and reduce size
                                supportPlanes[0] = supportPlanes[2];
                                numSupportPlanes = 1;
                                goto case 1;
                            }
                        }

                        // Try to drop plane 0 or 1
                        for (int testPlane = 0; testPlane < 2; testPlane++)
                        {
                            tempVelocity = velocity;
                            Solve2d(up, supportPlanes[testPlane], supportPlanes[2], ref tempVelocity);

                            bool planeUsed = Test1d(supportPlanes[1 - testPlane], tempVelocity);
                            if (!planeUsed)
                            {
                                supportPlanes[0] = supportPlanes[testPlane];
                                supportPlanes[1] = supportPlanes[2];
                                numSupportPlanes--;
                                goto case 2;
                            }
                        }

                        // Try solve all three
                        Solve3d(up, supportPlanes[0], supportPlanes[1], supportPlanes[2], ref velocity);

                        return;
                    }
                case 4:
                    {
                        for (int i = 0; i < 3; i++)
                        {
                            float3 tempVelocity = velocity;
                            Solve3d(up, supportPlanes[(i + 1) % 3], supportPlanes[(i + 2) % 3], supportPlanes[3], ref tempVelocity);
                            bool planeUsed = Test1d(supportPlanes[i], tempVelocity);
                            if (!planeUsed)
                            {
                                supportPlanes[i] = supportPlanes[2];
                                supportPlanes[2] = supportPlanes[3];
                                numSupportPlanes = 3;
                                goto case 3;
                            }
                        }

                        // Nothing can be dropped so we've failed to solve,
                        // now we do all 3d combinations
                        float3 tempVel = velocity;
                        SurfaceConstraintInfo sp0 = supportPlanes[0];
                        SurfaceConstraintInfo sp1 = supportPlanes[1];
                        SurfaceConstraintInfo sp2 = supportPlanes[2];
                        SurfaceConstraintInfo sp3 = supportPlanes[3];
                        Solve3d(up, sp0, sp1, sp2, ref tempVel);
                        Solve3d(up, sp0, sp1, sp3, ref tempVel);
                        Solve3d(up, sp0, sp2, sp3, ref tempVel);
                        Solve3d(up, sp1, sp2, sp3, ref tempVel);

                        velocity = tempVel;

                        return;
                    }
                default:
                    {
                        // Can't have more than 4 and less than 1 plane
                        Assert.IsTrue(false);
                        break;
                    }
            }
        }

        public static void Solve1d(SurfaceConstraintInfo constraint, ref float3 velocity)
        {
            float3 groundVelocity = constraint.Velocity;
            float3 relVel = velocity - groundVelocity;
            sfloat planeVel = math.dot(relVel, constraint.Plane.Normal);
            relVel -= planeVel * constraint.Plane.Normal;

            velocity = relVel + groundVelocity;
        }

        public static bool Test1d(SurfaceConstraintInfo constraint, float3 velocity)
        {
            float3 relVel = velocity - constraint.Velocity;
            sfloat planeVel = math.dot(relVel, constraint.Plane.Normal);
            return planeVel < -k_Epsilon;
        }

        public static void Solve2d(float3 up, SurfaceConstraintInfo constraint0, SurfaceConstraintInfo constraint1, ref float3 velocity)
        {
            float3 plane0 = constraint0.Plane.Normal;
            float3 plane1 = constraint1.Plane.Normal;

            // Calculate the free axis
            float3 axis = math.cross(plane0, plane1);
            sfloat axisLen2 = math.lengthsq(axis);

            // Check for parallel planes
            if (axisLen2 < k_Epsilon)
            {
                // Do the planes sequentially
                Sort2d(ref constraint0, ref constraint1);
                Solve1d(constraint1, ref velocity);
                Solve1d(constraint0, ref velocity);

                return;
            }

            sfloat invAxisLen = math.rsqrt(axisLen2);
            axis *= invAxisLen;

            // Calculate the velocity of the free axis
            float3 axisVel;
            {
                float4x4 m = new float4x4();
                float3 r0 = math.cross(plane0, plane1);
                float3 r1 = math.cross(plane1, axis);
                float3 r2 = math.cross(axis, plane0);
                m.c0 = new float4(r0, sfloat.Zero);
                m.c1 = new float4(r1, sfloat.Zero);
                m.c2 = new float4(r2, sfloat.Zero);
                m.c3 = new float4(sfloat.Zero, sfloat.Zero, sfloat.Zero, sfloat.One);

                float3 sVel = constraint0.Velocity + constraint1.Velocity;
                float3 t = new float3(
                    math.dot(axis, sVel) * (sfloat)0.5f,
                    math.dot(plane0, constraint0.Velocity),
                    math.dot(plane1, constraint1.Velocity));

                axisVel = math.rotate(m, t);
                axisVel *= invAxisLen;
            }

            float3 groundVelocity = axisVel;
            float3 relVel = velocity - groundVelocity;

            sfloat vel2 = math.lengthsq(relVel);
            sfloat axisVert = math.dot(up, axis);
            sfloat axisProjVel = math.dot(relVel, axis);

            velocity = groundVelocity + axis * axisProjVel;
        }

        public static void Solve3d(float3 up, SurfaceConstraintInfo constraint0, SurfaceConstraintInfo constraint1, SurfaceConstraintInfo constraint2, ref float3 velocity)
        {
            float3 plane0 = constraint0.Plane.Normal;
            float3 plane1 = constraint1.Plane.Normal;
            float3 plane2 = constraint2.Plane.Normal;

            float4x4 m = new float4x4();
            float3 r0 = math.cross(plane1, plane2);
            float3 r1 = math.cross(plane2, plane0);
            float3 r2 = math.cross(plane0, plane1);
            m.c0 = new float4(r0, sfloat.Zero);
            m.c1 = new float4(r1, sfloat.Zero);
            m.c2 = new float4(r2, sfloat.Zero);
            m.c3 = new float4(sfloat.Zero, sfloat.Zero, sfloat.Zero, sfloat.One);

            sfloat det = math.dot(r0, plane0);
            sfloat tst = math.abs(det);
            if (tst < k_Epsilon)
            {
                Sort3d(ref constraint0, ref constraint1, ref constraint2);
                Solve2d(up, constraint1, constraint2, ref velocity);
                Solve2d(up, constraint0, constraint2, ref velocity);
                Solve2d(up, constraint0, constraint1, ref velocity);

                return;
            }

            float3 t = new float3(
                math.dot(plane0, constraint0.Velocity),
                math.dot(plane1, constraint1.Velocity),
                math.dot(plane2, constraint2.Velocity));

            float3 pointVel = math.rotate(m, t);
            pointVel /= det;

            velocity = pointVel;
        }

        public static void Sort2d(ref SurfaceConstraintInfo plane0, ref SurfaceConstraintInfo plane1)
        {
            int priority0 = plane0.Priority;
            int priority1 = plane1.Priority;
            if (priority0 > priority1)
            {
                SwapPlanes(ref plane0, ref plane1);
            }
        }

        public static void Sort3d(ref SurfaceConstraintInfo plane0, ref SurfaceConstraintInfo plane1, ref SurfaceConstraintInfo plane2)
        {
            int priority0 = plane0.Priority;
            int priority1 = plane1.Priority;
            int priority2 = plane2.Priority;
            if (priority0 <= priority1)
            {
                if (priority1 <= priority2)
                {
                    // 0, 1, 2
                }
                else if (priority0 <= priority2)
                {
                    // 0, 2, 1
                    SwapPlanes(ref plane1, ref plane2);
                }
                else
                {
                    // 1, 2, 0
                    SwapPlanes(ref plane0, ref plane1);
                    SwapPlanes(ref plane0, ref plane2);
                }
            }
            else
            {
                if (priority2 < priority1)
                {
                    // 2, 1, 0
                    SwapPlanes(ref plane0, ref plane2);
                }
                else if (priority2 > priority0)
                {
                    // 1, 0, 2
                    SwapPlanes(ref plane0, ref plane1);
                }
                else
                {
                    // 2, 0, 1
                    SwapPlanes(ref plane0, ref plane1);
                    SwapPlanes(ref plane1, ref plane2);
                }
            }
        }

        public static void SwapPlanes(ref SurfaceConstraintInfo plane0, ref SurfaceConstraintInfo plane1)
        {
            var temp = plane0;
            plane0 = plane1;
            plane1 = temp;
        }

        [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
        private static void ClampToMaxLength(sfloat maxLength, ref float3 vector)
        {
            sfloat lengthSq = math.lengthsq(vector);
            bool maxExceeded = lengthSq > maxLength * maxLength;
            if (maxExceeded)
            {
                sfloat invLen = math.rsqrt(lengthSq);
                float3 rescaledVector = maxLength * invLen * vector;
                vector = rescaledVector;
            }
        }
    }
}
