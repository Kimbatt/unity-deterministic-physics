using System;
using System.Runtime.CompilerServices;
using Unity.Assertions;
using Unity.Collections;
using Unity.Entities;
using UnityS.Mathematics;
using static UnityS.Physics.Math;

namespace UnityS.Physics
{
    // An instance of a collider in a physics world.
    public struct RigidBody : ICollidable
    {
        // The rigid body's collider (allowed to be null)
        public BlobAssetReference<Collider> Collider;

        // The rigid body's transform in world space
        public RigidTransform WorldFromBody;

        // The entity that rigid body represents
        public Entity Entity;

        // Arbitrary custom tags.
        // These get copied into contact manifolds and can be used to inform contact modifiers.
        public byte CustomTags;

        public static readonly RigidBody Zero = new RigidBody
        {
            WorldFromBody = RigidTransform.identity,
            Collider = default,
            Entity = Entity.Null,
            CustomTags = 0
        };

        #region ICollidable implementation

        public Aabb CalculateAabb()
        {
            if (Collider.IsCreated)
            {
                return Collider.Value.CalculateAabb(WorldFromBody);
            }
            return new Aabb { Min = WorldFromBody.pos, Max = WorldFromBody.pos };
        }

        public Aabb CalculateAabb(RigidTransform transform)
        {
            if (Collider.IsCreated)
            {
                return Collider.Value.CalculateAabb(math.mul(transform, WorldFromBody));
            }
            return new Aabb { Min = WorldFromBody.pos, Max = WorldFromBody.pos };
        }

        public bool CastRay(RaycastInput input) => QueryWrappers.RayCast(ref this, input);
        public bool CastRay(RaycastInput input, out RaycastHit closestHit) => QueryWrappers.RayCast(ref this, input, out closestHit);
        public bool CastRay(RaycastInput input, ref NativeList<RaycastHit> allHits) => QueryWrappers.RayCast(ref this, input, ref allHits);
        public bool CastRay<T>(RaycastInput input, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            // Transform the ray into body space
            var worldFromBody = new MTransform(WorldFromBody);
            MTransform bodyFromWorld = Inverse(worldFromBody);

            input.Ray.Origin = Mul(bodyFromWorld, input.Ray.Origin);
            input.Ray.Displacement = math.mul(bodyFromWorld.Rotation, input.Ray.Displacement);

            SetQueryContextParameters(ref input.QueryContext, ref worldFromBody);

            return Collider.IsCreated && Collider.Value.CastRay(input, ref collector);
        }

        public bool CastCollider(ColliderCastInput input) => QueryWrappers.ColliderCast(ref this, input);
        public bool CastCollider(ColliderCastInput input, out ColliderCastHit closestHit) => QueryWrappers.ColliderCast(ref this, input, out closestHit);
        public bool CastCollider(ColliderCastInput input, ref NativeList<ColliderCastHit> allHits) => QueryWrappers.ColliderCast(ref this, input, ref allHits);
        public bool CastCollider<T>(ColliderCastInput input, ref T collector) where T : struct, ICollector<ColliderCastHit>
        {
            // Transform the input into body space
            MTransform worldFromBody = new MTransform(WorldFromBody);
            MTransform bodyFromWorld = Inverse(worldFromBody);

            input.Orientation = math.mul(math.inverse(WorldFromBody.rot), input.Orientation);
            input.Ray.Origin = Mul(bodyFromWorld, input.Ray.Origin);
            input.Ray.Displacement = math.mul(bodyFromWorld.Rotation, input.Ray.Displacement);

            SetQueryContextParameters(ref input.QueryContext, ref worldFromBody);

            return Collider.IsCreated && Collider.Value.CastCollider(input, ref collector);
        }

        public bool CalculateDistance(PointDistanceInput input) => QueryWrappers.CalculateDistance(ref this, input);
        public bool CalculateDistance(PointDistanceInput input, out DistanceHit closestHit) => QueryWrappers.CalculateDistance(ref this, input, out closestHit);
        public bool CalculateDistance(PointDistanceInput input, ref NativeList<DistanceHit> allHits) => QueryWrappers.CalculateDistance(ref this, input, ref allHits);
        public bool CalculateDistance<T>(PointDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            // Transform the input into body space
            MTransform worldFromBody = new MTransform(WorldFromBody);
            MTransform bodyFromWorld = Inverse(worldFromBody);

            input.Position = Mul(bodyFromWorld, input.Position);

            SetQueryContextParameters(ref input.QueryContext, ref worldFromBody);

            return Collider.IsCreated && Collider.Value.CalculateDistance(input, ref collector);
        }

        public bool CalculateDistance(ColliderDistanceInput input) => QueryWrappers.CalculateDistance(ref this, input);
        public bool CalculateDistance(ColliderDistanceInput input, out DistanceHit closestHit) => QueryWrappers.CalculateDistance(ref this, input, out closestHit);
        public bool CalculateDistance(ColliderDistanceInput input, ref NativeList<DistanceHit> allHits) => QueryWrappers.CalculateDistance(ref this, input, ref allHits);
        public bool CalculateDistance<T>(ColliderDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            // Transform the input into body space
            MTransform worldFromBody = new MTransform(WorldFromBody);
            MTransform bodyFromWorld = Inverse(worldFromBody);

            input.Transform = new RigidTransform(
                math.mul(math.inverse(WorldFromBody.rot), input.Transform.rot),
                Mul(bodyFromWorld, input.Transform.pos));

            SetQueryContextParameters(ref input.QueryContext, ref worldFromBody);

            return Collider.IsCreated && Collider.Value.CalculateDistance(input, ref collector);
        }

        #endregion

        #region private

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void SetQueryContextParameters (ref QueryContext context, ref MTransform worldFromBody)
        {
            // QueryContext.WorldFromLocalTransform is not expected to be initialized at this point
            // and should have default value (zeros in all fields)
            Assert.IsTrue(context.WorldFromLocalTransform.Translation.Equals(float3.zero));
            Assert.IsTrue(context.WorldFromLocalTransform.Rotation.Equals(float3x3.zero));

            context.ColliderKey = ColliderKey.Empty;
            context.Entity = Entity;
            context.WorldFromLocalTransform = worldFromBody;
            if (!context.IsInitialized)
            {
                context.RigidBodyIndex = -1;
                context.IsInitialized = true;
            }
        }

        #endregion
    }

    // A pair of rigid body indices
    public struct BodyIndexPair
    {
        // B before A to match Havok
        public int BodyIndexB;
        public int BodyIndexA;

        public bool IsValid => BodyIndexB != -1 && BodyIndexA != -1;

        public static BodyIndexPair Invalid => new BodyIndexPair { BodyIndexB = -1, BodyIndexA = -1 };
    }

    // A pair of entities
    public struct EntityPair
    {
        // B before A for consistency with other pairs
        public Entity EntityB;
        public Entity EntityA;
    }

    // A pair of custom rigid body tags
    public struct CustomTagsPair
    {
        // B before A for consistency with other pairs
        public byte CustomTagsB;
        public byte CustomTagsA;
    }
}
