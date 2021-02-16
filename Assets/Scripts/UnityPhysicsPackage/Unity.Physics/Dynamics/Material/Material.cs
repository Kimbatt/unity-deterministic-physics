using System;
using Unity.Assertions;
using UnityS.Mathematics;

namespace UnityS.Physics
{
    /// <summary>
    /// Defines the collision response policy of a collider
    /// </summary>
    public enum CollisionResponsePolicy : byte
    {
        /// <summary>
        /// The collider will collide normally
        /// </summary>
        Collide = 0,
        /// <summary>
        /// The collider will collide normally and raise collision events
        /// </summary>
        CollideRaiseCollisionEvents = 1,
        /// <summary>
        /// The collider will raise trigger events when it overlaps another collider
        /// </summary>
        RaiseTriggerEvents = 3,
        /// <summary>
        /// The collider will skip collision, but can still move and intercept queries
        /// </summary>
        None = byte.MaxValue - 1,
    }

    // Describes how an object should respond to collisions with other objects.
    public struct Material : IEquatable<Material>
    {
        internal MaterialFlags Flags;
        public CombinePolicy FrictionCombinePolicy;
        public CombinePolicy RestitutionCombinePolicy;
        public byte CustomTags;
        public sfloat Friction;
        public sfloat Restitution;

        public CollisionResponsePolicy CollisionResponse
        {
            get => FlagsToCollisionResponse(Flags);
            set
            {
                switch (value)
                {
                    case CollisionResponsePolicy.None:
                        Flags |= MaterialFlags.DisableCollisions;
                        Flags &= ~MaterialFlags.IsTrigger & ~MaterialFlags.EnableCollisionEvents;
                        return;
                    case CollisionResponsePolicy.RaiseTriggerEvents:
                        Flags |= MaterialFlags.IsTrigger;
                        Flags &= ~MaterialFlags.DisableCollisions & ~MaterialFlags.EnableCollisionEvents;
                        return;
                    case CollisionResponsePolicy.CollideRaiseCollisionEvents:
                        Flags |= MaterialFlags.EnableCollisionEvents;
                        Flags &= ~MaterialFlags.IsTrigger & ~MaterialFlags.DisableCollisions;
                        return;
                    case CollisionResponsePolicy.Collide:
                        Flags &= ~MaterialFlags.DisableCollisions & ~MaterialFlags.EnableCollisionEvents & ~MaterialFlags.IsTrigger;
                        return;
                    default:
                        Assert.IsTrue(false, "Invalid collision response provided!");
                        return;
                }
            }
        }

        // If true, the object can have its inertia and mass overridden during solving
        public bool EnableMassFactors
        {
            get { return (Flags & MaterialFlags.EnableMassFactors) != 0; }
            set
            {
                if (value != EnableMassFactors)
                {
                    // Toggle the bit since the value is changing
                    Flags ^= MaterialFlags.EnableMassFactors;
                }
            }
        }

        // If true, the object can apply a surface velocity to its contact points
        public bool EnableSurfaceVelocity
        {
            get { return (Flags & MaterialFlags.EnableSurfaceVelocity) != 0; }
            set
            {
                if (value != EnableSurfaceVelocity)
                {
                    // Toggle the bit since the value is changing
                    Flags ^= MaterialFlags.EnableSurfaceVelocity;
                }
            }
        }

        // Higher priority flags render lower priority ones useless, while same priority flags can co-exist.
        // Priority is as follows:
        // 1. DisableCollisions
        // 2. IsTrigger
        // 3. EnableCollisionEvents
        // 3. EnableMassFactors
        // 3. EnableSurfaceVelocity
        [Flags]
        internal enum MaterialFlags : byte
        {
            None = 0,
            IsTrigger = 1 << 0,
            EnableCollisionEvents = 1 << 1,
            EnableMassFactors = 1 << 2,
            EnableSurfaceVelocity = 1 << 3,
            DisableCollisions = 1 << 4
        }

        // Defines how a value from a pair of materials should be combined.
        public enum CombinePolicy : byte
        {
            GeometricMean,  // sqrt(a * b)
            Minimum,        // min(a, b)
            Maximum,        // max(a, b)
            ArithmeticMean  // (a + b) / 2
        }

        // A default material.
        public static Material Default => new Material
        {
            FrictionCombinePolicy = CombinePolicy.GeometricMean,
            RestitutionCombinePolicy = CombinePolicy.GeometricMean,
            Friction = (sfloat)0.5f,
            Restitution = sfloat.Zero
        };

        private static CollisionResponsePolicy FlagsToCollisionResponse(MaterialFlags flags)
        {
            if ((flags & MaterialFlags.DisableCollisions) != 0)
            {
                return CollisionResponsePolicy.None;
            }
            else if ((flags & MaterialFlags.IsTrigger) != 0)
            {
                return CollisionResponsePolicy.RaiseTriggerEvents;
            }
            else if ((flags & MaterialFlags.EnableCollisionEvents) != 0)
            {
                return CollisionResponsePolicy.CollideRaiseCollisionEvents;
            }
            else
            {
                return CollisionResponsePolicy.Collide;
            }
        }

        // Get combined collision response of the 2 materials.
        // Used only internally by the manifold creation pipeline.
        internal static CollisionResponsePolicy GetCombinedCollisionResponse(Material materialA, Material materialB)
        {
            var flags = materialA.Flags | materialB.Flags;
            return FlagsToCollisionResponse(flags);
        }

        // Get a combined friction value for a pair of materials.
        // The combine policy with the highest value takes priority.
        public static sfloat GetCombinedFriction(Material materialA, Material materialB)
        {
            var policy = (CombinePolicy)math.max((int)materialA.FrictionCombinePolicy, (int)materialB.FrictionCombinePolicy);
            switch (policy)
            {
                case CombinePolicy.GeometricMean:
                    return math.sqrt(materialA.Friction * materialB.Friction);
                case CombinePolicy.Minimum:
                    return math.min(materialA.Friction, materialB.Friction);
                case CombinePolicy.Maximum:
                    return math.max(materialA.Friction, materialB.Friction);
                case CombinePolicy.ArithmeticMean:
                    return (materialA.Friction + materialB.Friction) * (sfloat)0.5f;
                default:
                    return sfloat.Zero;
            }
        }

        // Get a combined restitution value for a pair of materials.
        // The combine policy with the highest value takes priority.
        public static sfloat GetCombinedRestitution(Material materialA, Material materialB)
        {
            var policy = (CombinePolicy)math.max((int)materialA.RestitutionCombinePolicy, (int)materialB.RestitutionCombinePolicy);
            switch (policy)
            {
                case CombinePolicy.GeometricMean:
                    return math.sqrt(materialA.Restitution * materialB.Restitution);
                case CombinePolicy.Minimum:
                    return math.min(materialA.Restitution, materialB.Restitution);
                case CombinePolicy.Maximum:
                    return math.max(materialA.Restitution, materialB.Restitution);
                case CombinePolicy.ArithmeticMean:
                    return (materialA.Restitution + materialB.Restitution) * (sfloat)0.5f;
                default:
                    return sfloat.Zero;
            }
        }

        public bool Equals(Material other)
        {
            return
                Flags == other.Flags &&
                FrictionCombinePolicy == other.FrictionCombinePolicy &&
                RestitutionCombinePolicy == other.RestitutionCombinePolicy &&
                CustomTags == other.CustomTags &&
                Friction == other.Friction &&
                Restitution == other.Restitution;
        }

        public override int GetHashCode()
        {
            return unchecked((int)math.hash(new uint2(
                unchecked((uint)(
                    (byte)Flags
                    | ((byte)FrictionCombinePolicy << 4)
                    | ((byte)RestitutionCombinePolicy << 8)
                    | (CustomTags << 12))
                ),
                math.hash(new float2(Friction, Restitution))
            )));
        }
    }
}
