using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using UnityS.Mathematics;

namespace UnityS.Physics
{
    // Describes which other objects an object can collide with.
    [DebuggerDisplay("Group: {GroupIndex} BelongsTo: {BelongsTo} CollidesWith: {CollidesWith}")]
    public struct CollisionFilter : IEquatable<CollisionFilter>
    {
        // A bit mask describing which layers this object belongs to.
        public uint BelongsTo;

        // A bit mask describing which layers this object can collide with.
        public uint CollidesWith;

        // An optional override for the bit mask checks.
        // If the value in both objects is equal and positive, the objects always collide.
        // If the value in both objects is equal and negative, the objects never collide.
        public int GroupIndex;

        // Returns true if the filter cannot collide with anything,
        // which likely means it was default constructed but not initialized.
        public bool IsEmpty => BelongsTo == 0 || CollidesWith == 0;

        // A collision filter which wants to collide with everything.
        public static readonly CollisionFilter Default = new CollisionFilter
        {
            BelongsTo = 0xffffffff,
            CollidesWith = 0xffffffff,
            GroupIndex = 0
        };

        // A collision filter which never collides with against anything (including Default).
        public static readonly CollisionFilter Zero = new CollisionFilter
        {
            BelongsTo = 0,
            CollidesWith = 0,
            GroupIndex = 0
        };

        // Return true if the given pair of filters want to collide with each other.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsCollisionEnabled(CollisionFilter filterA, CollisionFilter filterB)
        {
            if (filterA.GroupIndex > 0 && filterA.GroupIndex == filterB.GroupIndex)
            {
                return true;
            }
            if (filterA.GroupIndex < 0 && filterA.GroupIndex == filterB.GroupIndex)
            {
                return false;
            }
            return
                (filterA.BelongsTo & filterB.CollidesWith) != 0 &&
                (filterB.BelongsTo & filterA.CollidesWith) != 0;
        }

        // Return a union of two filters.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static CollisionFilter CreateUnion(CollisionFilter filterA, CollisionFilter filterB)
        {
            return new CollisionFilter
            {
                BelongsTo = filterA.BelongsTo | filterB.BelongsTo,
                CollidesWith = filterA.CollidesWith | filterB.CollidesWith,
                GroupIndex = (filterA.GroupIndex == filterB.GroupIndex) ? filterA.GroupIndex : 0
            };
        }

        public override int GetHashCode()
        {
            return unchecked((int)math.hash(new uint3(
                BelongsTo,
                CollidesWith,
                unchecked((uint)GroupIndex)
            )));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(CollisionFilter other)
        {
            return BelongsTo == other.BelongsTo && CollidesWith == other.CollidesWith && GroupIndex == other.GroupIndex;
        }
    }
}
