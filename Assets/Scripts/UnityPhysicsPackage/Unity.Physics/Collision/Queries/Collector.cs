using Unity.Collections;
using Unity.Entities;
using UnityEngine.Assertions;
using static UnityS.Physics.Math;

namespace UnityS.Physics
{
    public interface IQueryResult
    {
        // For casts this is fraction of the query at which the hit occurred.
        // For distance queries, this is a distance from the query object
        sfloat Fraction { get; }

        // Index of the hit body in the CollisionWorld's rigid body array
        int RigidBodyIndex { get; }

        // ColliderKey of the hit leaf collider
        ColliderKey ColliderKey { get; }

        // Material of the hit leaf collider
        Material Material { get; }

        // Entity of the hit body
        Entity Entity { get; }
    }

    struct QueryContext
    {
        public int RigidBodyIndex;
        public ColliderKey ColliderKey;
        public Entity Entity;
        public uint NumColliderKeyBits;
        public MTransform WorldFromLocalTransform;
        public bool IsInitialized;

        public static QueryContext DefaultContext => new QueryContext
        {
            RigidBodyIndex = -1,
            ColliderKey = ColliderKey.Empty,
            Entity = Entity.Null,
            NumColliderKeyBits = 0,
            WorldFromLocalTransform = MTransform.Identity,
            IsInitialized = true
        };

        public ColliderKey SetSubKey(uint childSubKeyNumOfBits, uint childSubKey)
        {
            var parentColliderKey = ColliderKey;
            parentColliderKey.PopSubKey(NumColliderKeyBits, out uint parentKey);

            var colliderKey = new ColliderKey(childSubKeyNumOfBits, childSubKey);
            colliderKey.PushSubKey(NumColliderKeyBits, parentKey);
            return colliderKey;
        }

        public ColliderKey PushSubKey(uint childSubKeyNumOfBits, uint childSubKey)
        {
            var colliderKey = SetSubKey(childSubKeyNumOfBits, childSubKey);
            NumColliderKeyBits += childSubKeyNumOfBits;
            return colliderKey;
        }
    }

    // Interface for collecting hits during a collision query
    public interface ICollector<T> where T : struct, IQueryResult
    {
        // Whether to exit the query as soon as any hit has been accepted
        bool EarlyOutOnFirstHit { get; }

        // The maximum fraction of the query within which to check for hits
        // For casts, this is a fraction along the ray
        // For distance queries, this is a distance from the query object
        sfloat MaxFraction { get; }

        // The number of hits that have been collected
        int NumHits { get; }

        // Called when the query hits something
        // Return true to accept the hit, or false to ignore it
        bool AddHit(T hit);
    }

    // A collector which exits the query as soon as any hit is detected.
    public struct AnyHitCollector<T> : ICollector<T> where T : struct, IQueryResult
    {
        public bool EarlyOutOnFirstHit => true;
        public sfloat MaxFraction { get; }
        public int NumHits => 0;

        public AnyHitCollector(sfloat maxFraction)
        {
            MaxFraction = maxFraction;
        }

        #region ICollector

        public bool AddHit(T hit)
        {
            Assert.IsTrue(hit.Fraction < MaxFraction);
            return true;
        }

        #endregion
    }

    // A collector which stores only the closest hit.
    public struct ClosestHitCollector<T> : ICollector<T> where T : struct, IQueryResult
    {
        public bool EarlyOutOnFirstHit => false;
        public sfloat MaxFraction { get; private set; }
        public int NumHits { get; private set; }

        private T m_ClosestHit;
        public T ClosestHit => m_ClosestHit;

        public ClosestHitCollector(sfloat maxFraction)
        {
            MaxFraction = maxFraction;
            m_ClosestHit = default(T);
            NumHits = 0;
        }

        #region ICollector

        public bool AddHit(T hit)
        {
            Assert.IsTrue(hit.Fraction <= MaxFraction);
            MaxFraction = hit.Fraction;
            m_ClosestHit = hit;
            NumHits = 1;
            return true;
        }

        #endregion
    }

    // A collector which stores every hit.
    public struct AllHitsCollector<T> : ICollector<T> where T : struct, IQueryResult
    {
        public bool EarlyOutOnFirstHit => false;
        public sfloat MaxFraction { get; }
        public int NumHits => AllHits.Length;

        public NativeList<T> AllHits;

        public AllHitsCollector(sfloat maxFraction, ref NativeList<T> allHits)
        {
            MaxFraction = maxFraction;
            AllHits = allHits;
        }

        #region ICollector

        public bool AddHit(T hit)
        {
            Assert.IsTrue(hit.Fraction < MaxFraction);

            AllHits.Add(hit);
            return true;
        }

        #endregion
    }

    // A collector used to provide filtering for QueryInteraction enum
    // This is a wrapper of the user provided collector, which serves to enable
    // filtering based on the QueryInteraction parameter.
    internal unsafe struct QueryInteractionCollector<T, C> : ICollector<T>
        where T : struct, IQueryResult
        where C : struct, ICollector<T>
    {
        public bool EarlyOutOnFirstHit => Collector.EarlyOutOnFirstHit;

        public sfloat MaxFraction => Collector.MaxFraction;

        public int NumHits => Collector.NumHits;

        // Todo: have a QueryInteraction field here, and filter differently based on it in AddHit()
        // at the moment, this collector will only get constructed if IgnoreTriggers interaction is selected
        public C Collector;

        public bool AddHit(T hit)
        {
            if (hit.Material.CollisionResponse == CollisionResponsePolicy.RaiseTriggerEvents)
            {
                return false;
            }

            return Collector.AddHit(hit);
        }
    }
}
