using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityS.Mathematics;
using static UnityS.Physics.Math;

namespace UnityS.Physics
{
    // A header preceding a number of contact points in a stream.
    struct ContactHeader
    {
        public BodyIndexPair BodyPair;
        public CustomTagsPair BodyCustomTags;
        public JacobianFlags JacobianFlags;
        public int NumContacts;
        public float3 Normal;
        public sfloat CoefficientOfFriction;
        public sfloat CoefficientOfRestitution;
        public ColliderKeyPair ColliderKeys;

        // followed by NumContacts * ContactPoint
    }

    // A contact point in a manifold. All contacts share the same normal.
    public struct ContactPoint
    {
        public float3 Position; // world space position on object B
        public sfloat Distance;  // separating distance along the manifold normal
    }

    // Contact manifold stream generation functions
    static class ManifoldQueries
    {
        // A context passed through the manifold generation functions
        internal unsafe struct Context
        {
            public BodyIndexPair BodyIndices;
            public CustomTagsPair BodyCustomTags;
            public bool BodiesHaveInfiniteMass;
            public NativeStream.Writer* ContactWriter;  // cannot be passed by value
        }

        // Write a set of contact manifolds for a pair of bodies to the given stream.
        public static unsafe void BodyBody(in RigidBody rigidBodyA, in RigidBody rigidBodyB, in MotionVelocity motionVelocityA, in MotionVelocity motionVelocityB,
            sfloat collisionTolerance, sfloat timeStep, BodyIndexPair pair, ref NativeStream.Writer contactWriter)
        {
            var colliderA = (Collider*)rigidBodyA.Collider.GetUnsafePtr();
            var colliderB = (Collider*)rigidBodyB.Collider.GetUnsafePtr();

            if (colliderA == null || colliderB == null || !CollisionFilter.IsCollisionEnabled(colliderA->Filter, colliderB->Filter))
            {
                return;
            }

            // Build combined motion expansion
            MotionExpansion expansion;
            {
                MotionExpansion expansionA = motionVelocityA.CalculateExpansion(timeStep);
                MotionExpansion expansionB = motionVelocityB.CalculateExpansion(timeStep);
                expansion = new MotionExpansion
                {
                    Linear = expansionA.Linear - expansionB.Linear,
                    Uniform = expansionA.Uniform + expansionB.Uniform + collisionTolerance
                };
            }

            var context = new Context
            {
                BodyIndices = pair,
                BodyCustomTags = new CustomTagsPair { CustomTagsA = rigidBodyA.CustomTags, CustomTagsB = rigidBodyB.CustomTags },
                BodiesHaveInfiniteMass = motionVelocityA.HasInfiniteInertiaAndMass && motionVelocityB.HasInfiniteInertiaAndMass,
                ContactWriter = (NativeStream.Writer*)UnsafeUtility.AddressOf(ref contactWriter)
            };

            var worldFromA = new MTransform(rigidBodyA.WorldFromBody);
            var worldFromB = new MTransform(rigidBodyB.WorldFromBody);

            // Dispatch to appropriate manifold generator
            switch (colliderA->CollisionType)
            {
                case CollisionType.Convex:
                    switch (colliderB->CollisionType)
                    {
                        case CollisionType.Convex:
                            ConvexConvex(context, ColliderKeyPair.Empty, colliderA, colliderB, worldFromA, worldFromB, expansion.MaxDistance, false);
                            break;
                        case CollisionType.Composite:
                            ConvexComposite(context, ColliderKey.Empty, colliderA, colliderB, worldFromA, worldFromB, expansion, false);
                            break;
                        case CollisionType.Terrain:
                            ConvexTerrain(context, ColliderKeyPair.Empty, colliderA, colliderB, worldFromA, worldFromB, expansion.MaxDistance, false);
                            break;
                    }
                    break;
                case CollisionType.Composite:
                    switch (colliderB->CollisionType)
                    {
                        case CollisionType.Convex:
                            CompositeConvex(context, colliderA, colliderB, worldFromA, worldFromB, expansion, false);
                            break;
                        case CollisionType.Composite:
                            CompositeComposite(context, colliderA, colliderB, worldFromA, worldFromB, expansion, false);
                            break;
                        case CollisionType.Terrain:
                            CompositeTerrain(context, colliderA, colliderB, worldFromA, worldFromB, expansion.MaxDistance, false);
                            break;
                    }
                    break;
                case CollisionType.Terrain:
                    switch (colliderB->CollisionType)
                    {
                        case CollisionType.Convex:
                            TerrainConvex(context, ColliderKeyPair.Empty, colliderA, colliderB, worldFromA, worldFromB, expansion.MaxDistance, false);
                            break;
                        case CollisionType.Composite:
                            TerrainComposite(context, colliderA, colliderB, worldFromA, worldFromB, expansion.MaxDistance, false);
                            break;
                        case CollisionType.Terrain:
                            UnityEngine.Assertions.Assert.IsTrue(false);
                            break;
                    }
                    break;
            }
        }

        private static unsafe void WriteManifold(ConvexConvexManifoldQueries.Manifold manifold, Context context, ColliderKeyPair colliderKeys,
            Material materialA, Material materialB, bool flipped)
        {
            // Write results to stream
            if (manifold.NumContacts > 0)
            {
                if (flipped)
                {
                    manifold.Flip();
                }

                var header = new ContactHeader
                {
                    BodyPair = context.BodyIndices,
                    BodyCustomTags = context.BodyCustomTags,
                    NumContacts = manifold.NumContacts,
                    Normal = manifold.Normal,
                    ColliderKeys = colliderKeys
                };

                // Apply materials
                {
                    // Combined collision response of the two
                    CollisionResponsePolicy combinedCollisionResponse = Material.GetCombinedCollisionResponse(materialA, materialB);
                    Unity.Assertions.Assert.IsFalse(combinedCollisionResponse == CollisionResponsePolicy.None,
                        "DisableCollisions pairs should have been filtered out earlier!");

                    if (combinedCollisionResponse == CollisionResponsePolicy.RaiseTriggerEvents)
                    {
                        header.JacobianFlags |= JacobianFlags.IsTrigger;
                    }
                    else
                    {
                        if (combinedCollisionResponse == CollisionResponsePolicy.CollideRaiseCollisionEvents)
                        {
                            header.JacobianFlags |= JacobianFlags.EnableCollisionEvents;
                        }

                        Material.MaterialFlags combinedFlags = materialA.Flags | materialB.Flags;
                        if ((combinedFlags & Material.MaterialFlags.EnableMassFactors) != 0)
                        {
                            header.JacobianFlags |= JacobianFlags.EnableMassFactors;
                        }
                        if ((combinedFlags & Material.MaterialFlags.EnableSurfaceVelocity) != 0)
                        {
                            header.JacobianFlags |= JacobianFlags.EnableSurfaceVelocity;
                        }

                        header.CoefficientOfFriction = Material.GetCombinedFriction(materialA, materialB);
                        header.CoefficientOfRestitution = Material.GetCombinedRestitution(materialA, materialB);
                    }
                }

                context.ContactWriter->Write(header);

                // Group the contact points in 2s (when 4-6 contact points) and 3s (6 or more contact points)
                // to avoid the order forcing the magnitude of the impulse on one side of the face.
                // When less than 4 contact points access them in order.
                int startIndex = 0;
                int increment = header.NumContacts < 6 ?
                    math.max(header.NumContacts / 2, 1) : (header.NumContacts / 3 + ((header.NumContacts % 3 > 0) ? 1 : 0));

                int contactIndex = 0;
                while (true)
                {
                    if (contactIndex >= header.NumContacts)
                    {
                        startIndex++;
                        if (startIndex == increment)
                        {
                            break;
                        }
                        contactIndex = startIndex;
                    }
                    context.ContactWriter->Write(manifold[contactIndex]);
                    contactIndex += increment;
                }
            }
        }

        private static unsafe void ConvexConvex(
            Context context, ColliderKeyPair colliderKeys,
            Collider* convexColliderA, Collider* convexColliderB, [NoAlias] in MTransform worldFromA, [NoAlias] in MTransform worldFromB,
            sfloat maxDistance, bool flipped)
        {
            Material materialA = ((ConvexColliderHeader*)convexColliderA)->Material;
            Material materialB = ((ConvexColliderHeader*)convexColliderB)->Material;

            CollisionResponsePolicy combinedCollisionResponse = Material.GetCombinedCollisionResponse(materialA, materialB);

            // Skip the shapes if any of them is marked with a "None" collision response
            if (combinedCollisionResponse == CollisionResponsePolicy.None)
            {
                return;
            }

            // Skip if the bodies have infinite mass and the materials don't want to raise any solver events,
            // since the resulting contacts can't have any effect during solving
            if (context.BodiesHaveInfiniteMass)
            {
                if (combinedCollisionResponse != CollisionResponsePolicy.RaiseTriggerEvents &&
                    combinedCollisionResponse != CollisionResponsePolicy.CollideRaiseCollisionEvents)
                {
                    return;
                }
            }

            MTransform aFromB = Mul(Inverse(worldFromA), worldFromB);

            ConvexConvexManifoldQueries.Manifold contactManifold;
            switch (convexColliderA->Type)
            {
                case ColliderType.Sphere:
                    switch (convexColliderB->Type)
                    {
                        case ColliderType.Sphere:
                            ConvexConvexManifoldQueries.SphereSphere(
                                (SphereCollider*)convexColliderA, (SphereCollider*)convexColliderB,
                                worldFromA, aFromB, maxDistance, out contactManifold);
                            break;
                        case ColliderType.Capsule:
                            ConvexConvexManifoldQueries.CapsuleSphere(
                                (CapsuleCollider*)convexColliderB, (SphereCollider*)convexColliderA,
                                worldFromB, Inverse(aFromB), maxDistance, out contactManifold);
                            flipped = !flipped;
                            break;
                        case ColliderType.Triangle:
                            ConvexConvexManifoldQueries.TriangleSphere(
                                (PolygonCollider*)convexColliderB, (SphereCollider*)convexColliderA,
                                worldFromB, Inverse(aFromB), maxDistance, out contactManifold);
                            flipped = !flipped;
                            break;
                        case ColliderType.Box:
                            ConvexConvexManifoldQueries.BoxSphere(
                                (BoxCollider*)convexColliderB, (SphereCollider*)convexColliderA,
                                worldFromB, Inverse(aFromB), maxDistance, out contactManifold);
                            flipped = !flipped;
                            break;
                        case ColliderType.Quad:
                        case ColliderType.Cylinder:
                        case ColliderType.Convex:
                            ConvexConvexManifoldQueries.ConvexConvex(
                                ref ((SphereCollider*)convexColliderA)->ConvexHull, ref ((ConvexCollider*)convexColliderB)->ConvexHull,
                                worldFromA, aFromB, maxDistance, out contactManifold);
                            break;
                        default:
                            SafetyChecks.ThrowNotImplementedException();
                            return;
                    }
                    break;
                case ColliderType.Box:
                    switch (convexColliderB->Type)
                    {
                        case ColliderType.Sphere:
                            ConvexConvexManifoldQueries.BoxSphere(
                                (BoxCollider*)convexColliderA, (SphereCollider*)convexColliderB,
                                worldFromA, aFromB, maxDistance, out contactManifold);
                            break;
                        case ColliderType.Triangle:
                            ConvexConvexManifoldQueries.BoxTriangle(
                                (BoxCollider*)convexColliderA, (PolygonCollider*)convexColliderB,
                                worldFromA, aFromB, maxDistance, out contactManifold);
                            break;
                        case ColliderType.Box:
                            ConvexConvexManifoldQueries.BoxBox(
                                (BoxCollider*)convexColliderA, (BoxCollider*)convexColliderB,
                                worldFromA, aFromB, maxDistance, out contactManifold);
                            break;
                        case ColliderType.Capsule:
                        case ColliderType.Quad:
                        case ColliderType.Cylinder:
                        case ColliderType.Convex:
                            ConvexConvexManifoldQueries.ConvexConvex(
                                ref ((BoxCollider*)convexColliderA)->ConvexHull, ref ((ConvexCollider*)convexColliderB)->ConvexHull,
                                worldFromA, aFromB, maxDistance, out contactManifold);
                            break;
                        default:
                            SafetyChecks.ThrowNotImplementedException();
                            return;
                    }
                    break;
                case ColliderType.Capsule:
                    switch (convexColliderB->Type)
                    {
                        case ColliderType.Sphere:
                            ConvexConvexManifoldQueries.CapsuleSphere(
                                (CapsuleCollider*)convexColliderA, (SphereCollider*)convexColliderB,
                                worldFromA, aFromB, maxDistance, out contactManifold);
                            break;
                        case ColliderType.Capsule:
                            ConvexConvexManifoldQueries.CapsuleCapsule(
                                (CapsuleCollider*)convexColliderA, (CapsuleCollider*)convexColliderB,
                                worldFromA, aFromB, maxDistance, out contactManifold);
                            break;
                        case ColliderType.Triangle:
                            ConvexConvexManifoldQueries.CapsuleTriangle(
                                (CapsuleCollider*)convexColliderA, (PolygonCollider*)convexColliderB,
                                worldFromA, aFromB, maxDistance, out contactManifold);
                            break;
                        case ColliderType.Quad:
                        case ColliderType.Box:
                        case ColliderType.Cylinder:
                        case ColliderType.Convex:
                            ConvexConvexManifoldQueries.ConvexConvex(
                                ref ((CapsuleCollider*)convexColliderA)->ConvexHull, ref ((ConvexCollider*)convexColliderB)->ConvexHull,
                                worldFromA, aFromB, maxDistance, out contactManifold);
                            break;
                        default:
                            SafetyChecks.ThrowNotImplementedException();
                            return;
                    }
                    break;
                case ColliderType.Triangle:
                    switch (convexColliderB->Type)
                    {
                        case ColliderType.Sphere:
                            ConvexConvexManifoldQueries.TriangleSphere(
                                (PolygonCollider*)convexColliderA, (SphereCollider*)convexColliderB,
                                worldFromA, aFromB, maxDistance, out contactManifold);
                            break;
                        case ColliderType.Capsule:
                            ConvexConvexManifoldQueries.CapsuleTriangle(
                                (CapsuleCollider*)convexColliderB, (PolygonCollider*)convexColliderA,
                                worldFromB, Inverse(aFromB), maxDistance, out contactManifold);
                            flipped = !flipped;
                            break;
                        case ColliderType.Box:
                            ConvexConvexManifoldQueries.BoxTriangle(
                                (BoxCollider*)convexColliderB, (PolygonCollider*)convexColliderA,
                                worldFromB, Inverse(aFromB), maxDistance, out contactManifold);
                            flipped = !flipped;
                            break;
                        case ColliderType.Triangle:
                        case ColliderType.Quad:
                        case ColliderType.Cylinder:
                        case ColliderType.Convex:
                            ConvexConvexManifoldQueries.ConvexConvex(
                                ref ((PolygonCollider*)convexColliderA)->ConvexHull, ref ((ConvexCollider*)convexColliderB)->ConvexHull,
                                worldFromA, aFromB, maxDistance, out contactManifold);
                            break;
                        default:
                            SafetyChecks.ThrowNotImplementedException();
                            return;
                    }
                    break;
                case ColliderType.Quad:
                case ColliderType.Cylinder:
                case ColliderType.Convex:
                    ConvexConvexManifoldQueries.ConvexConvex(
                        ref ((ConvexCollider*)convexColliderA)->ConvexHull, ref ((ConvexCollider*)convexColliderB)->ConvexHull,
                        worldFromA, aFromB, maxDistance, out contactManifold);
                    break;
                default:
                    SafetyChecks.ThrowNotImplementedException();
                    return;
            }

            WriteManifold(contactManifold, context, colliderKeys, materialA, materialB, flipped);
        }

        private static unsafe void ConvexComposite(
            Context context, ColliderKey convexKeyA,
            [NoAlias] Collider* convexColliderA, [NoAlias] Collider* compositeColliderB, [NoAlias] in MTransform worldFromA, [NoAlias] in MTransform worldFromB,
            MotionExpansion expansion, bool flipped)
        {
            Material materialA = ((ConvexColliderHeader*)convexColliderA)->Material;

            // Skip the collision if convexColliderA is marked with a "None" collision response
            if (materialA.CollisionResponse == CollisionResponsePolicy.None)
            {
                return;
            }

            // Calculate swept AABB of A in B
            MTransform bFromWorld = Inverse(worldFromB);
            MTransform bFromA = Mul(bFromWorld, worldFromA);
            expansion.Linear = math.mul(bFromWorld.Rotation, expansion.Linear);
            var transform = new RigidTransform(new quaternion(bFromA.Rotation), bFromA.Translation); // TODO: avoid this conversion to and back from float3x3
            Aabb aabbAinB = expansion.ExpandAabb(convexColliderA->CalculateAabb(transform));

            // Do the midphase query and build manifolds for any overlapping leaf colliders
            var input = new OverlapAabbInput { Aabb = aabbAinB, Filter = convexColliderA->Filter };
            var collector = new ConvexCompositeOverlapCollector(
                context,
                convexColliderA, convexKeyA, compositeColliderB,
                worldFromA, worldFromB, expansion.MaxDistance, flipped);
            OverlapQueries.AabbCollider(input, compositeColliderB, ref collector);
        }

        private static unsafe void CompositeConvex(
            Context context,
            [NoAlias] Collider* compositeColliderA, [NoAlias] Collider* convexColliderB, [NoAlias] in MTransform worldFromA, [NoAlias] in MTransform worldFromB,
            MotionExpansion expansion, bool flipped)
        {
            // Flip the relevant inputs and call convex-vs-composite
            expansion.Linear = -expansion.Linear;
            ConvexComposite(context, ColliderKey.Empty,
                convexColliderB, compositeColliderA, worldFromB, worldFromA, expansion, !flipped);
        }

        internal unsafe struct ConvexCompositeOverlapCollector : IOverlapCollector
        {
            readonly Context m_Context;
            readonly Collider* m_ConvexColliderA;
            readonly ColliderKey m_ConvexColliderKey;
            readonly Collider* m_CompositeColliderB;
            readonly MTransform m_WorldFromA;
            readonly MTransform m_WorldFromB;
            readonly sfloat m_CollisionTolerance;
            readonly bool m_Flipped;

            ColliderKeyPath m_CompositeColliderKeyPath;

            public ConvexCompositeOverlapCollector(
                Context context,
                Collider* convexCollider, ColliderKey convexColliderKey, Collider* compositeCollider,
                MTransform worldFromA, MTransform worldFromB, sfloat collisionTolerance, bool flipped)
            {
                m_Context = context;
                m_ConvexColliderA = convexCollider;
                m_ConvexColliderKey = convexColliderKey;
                m_CompositeColliderB = compositeCollider;
                m_CompositeColliderKeyPath = ColliderKeyPath.Empty;
                m_WorldFromA = worldFromA;
                m_WorldFromB = worldFromB;
                m_CollisionTolerance = collisionTolerance;
                m_Flipped = flipped;
            }

            public void AddRigidBodyIndices(int* indices, int count) => SafetyChecks.ThrowNotSupportedException();

            public void AddColliderKeys([NoAlias] ColliderKey* keys, int count)
            {
                var colliderKeys = new ColliderKeyPair { ColliderKeyA = m_ConvexColliderKey, ColliderKeyB = m_ConvexColliderKey };
                CollisionFilter filter = m_ConvexColliderA->Filter;

                // Collide the convex A with all overlapping leaves of B
                switch (m_CompositeColliderB->Type)
                {
                    // Special case meshes (since we know all polygons will be built on the fly)
                    case ColliderType.Mesh:
                    {
                        Mesh* mesh = &((MeshCollider*)m_CompositeColliderB)->Mesh;
                        uint numMeshKeyBits = mesh->NumColliderKeyBits;
                        var polygon = new PolygonCollider();
                        polygon.InitNoVertices(CollisionFilter.Default, Material.Default);
                        for (int i = 0; i < count; i++)
                        {
                            ColliderKey compositeKey = m_CompositeColliderKeyPath.GetLeafKey(keys[i]);
                            uint meshKey = compositeKey.Value >> (32 - (int)numMeshKeyBits);
                            if (mesh->GetPolygon(meshKey, filter, ref polygon))
                            {
                                if (m_Flipped)
                                {
                                    colliderKeys.ColliderKeyA = compositeKey;
                                }
                                else
                                {
                                    colliderKeys.ColliderKeyB = compositeKey;
                                }

                                switch (m_ConvexColliderA->CollisionType)
                                {
                                    case CollisionType.Convex:
                                        ConvexConvex(
                                            m_Context, colliderKeys, m_ConvexColliderA, (Collider*)&polygon,
                                            m_WorldFromA, m_WorldFromB, m_CollisionTolerance, m_Flipped);
                                        break;

                                    case CollisionType.Terrain:
                                        TerrainConvex(
                                            m_Context, colliderKeys, m_ConvexColliderA, (Collider*)&polygon,
                                            m_WorldFromA, m_WorldFromB, m_CollisionTolerance, m_Flipped);
                                        break;
                                    default: // GetLeaf() may not return a composite collider
                                        SafetyChecks.ThrowNotImplementedException();
                                        return;
                                }
                            }
                        }
                    }
                    break;

                    // General case for all other composites (compounds, compounds of meshes, etc)
                    default:
                    {
                        for (int i = 0; i < count; i++)
                        {
                            ColliderKey compositeKey = m_CompositeColliderKeyPath.GetLeafKey(keys[i]);
                            m_CompositeColliderB->GetLeaf(compositeKey, out ChildCollider leaf);
                            if (CollisionFilter.IsCollisionEnabled(filter, leaf.Collider->Filter))  // TODO: shouldn't be needed if/when filtering is done fully by the BVH query
                            {
                                if (m_Flipped)
                                {
                                    colliderKeys.ColliderKeyA = compositeKey;
                                }
                                else
                                {
                                    colliderKeys.ColliderKeyB = compositeKey;
                                }

                                MTransform worldFromLeafB = Mul(m_WorldFromB, new MTransform(leaf.TransformFromChild));
                                switch (leaf.Collider->CollisionType)
                                {
                                    case CollisionType.Convex:
                                        ConvexConvex(
                                            m_Context, colliderKeys, m_ConvexColliderA, leaf.Collider,
                                            m_WorldFromA, worldFromLeafB, m_CollisionTolerance, m_Flipped);
                                        break;
                                    case CollisionType.Terrain:
                                        ConvexTerrain(
                                            m_Context, colliderKeys, m_ConvexColliderA, leaf.Collider,
                                            m_WorldFromA, worldFromLeafB, m_CollisionTolerance, m_Flipped);
                                        break;
                                    default: // GetLeaf() may not return a composite collider
                                        SafetyChecks.ThrowNotImplementedException();
                                        return;
                                }
                            }
                        }
                    }
                    break;
                }
            }

            public void PushCompositeCollider(ColliderKeyPath compositeKey)
            {
                m_CompositeColliderKeyPath.PushChildKey(compositeKey);
            }

            public void PopCompositeCollider(uint numCompositeKeyBits)
            {
                m_CompositeColliderKeyPath.PopChildKey(numCompositeKeyBits);
            }
        }

        private static unsafe void CompositeComposite(
            Context context,
            Collider* compositeColliderA, Collider* compositeColliderB, MTransform worldFromA, MTransform worldFromB,
            MotionExpansion expansion, bool flipped)
        {
            // Flip the order if necessary, so that A has fewer leaves than B
            if (compositeColliderA->NumColliderKeyBits > compositeColliderB->NumColliderKeyBits)
            {
                Collider* c = compositeColliderA;
                compositeColliderA = compositeColliderB;
                compositeColliderB = c;

                MTransform t = worldFromA;
                worldFromA = worldFromB;
                worldFromB = t;

                expansion.Linear = -expansion.Linear;
                flipped = !flipped;
            }

            var collector = new CompositeCompositeLeafCollector(
                context,
                compositeColliderA, compositeColliderB,
                worldFromA, worldFromB, expansion, flipped);
            compositeColliderA->GetLeaves(ref collector);
        }

        private unsafe struct CompositeCompositeLeafCollector : ILeafColliderCollector
        {
            // Inputs
            readonly Context m_Context;
            readonly Collider* m_CompositeColliderA;
            readonly Collider* m_CompositeColliderB;
            MTransform m_WorldFromA;
            readonly MTransform m_WorldFromB;
            readonly MotionExpansion m_Expansion;
            readonly bool m_Flipped;

            ColliderKeyPath m_KeyPath;

            public CompositeCompositeLeafCollector(
                Context context,
                Collider* compositeColliderA, Collider* compositeColliderB,
                MTransform worldFromA, MTransform worldFromB, MotionExpansion expansion, bool flipped)
            {
                m_Context = context;
                m_CompositeColliderA = compositeColliderA;
                m_CompositeColliderB = compositeColliderB;
                m_WorldFromA = worldFromA;
                m_WorldFromB = worldFromB;
                m_Expansion = expansion;
                m_Flipped = flipped;
                m_KeyPath = ColliderKeyPath.Empty;
            }

            public void AddLeaf(ColliderKey key, ref ChildCollider leaf)
            {
                MTransform worldFromLeafA = Mul(m_WorldFromA, new MTransform(leaf.TransformFromChild));
                ConvexComposite(
                    m_Context, m_KeyPath.GetLeafKey(key), leaf.Collider, m_CompositeColliderB,
                    worldFromLeafA, m_WorldFromB, m_Expansion, m_Flipped);
            }

            public void PushCompositeCollider(ColliderKeyPath compositeKey, MTransform parentFromComposite, out MTransform worldFromParent)
            {
                m_KeyPath.PushChildKey(compositeKey);
                worldFromParent = m_WorldFromA;
                m_WorldFromA = Math.Mul(worldFromParent, parentFromComposite);
            }

            public void PopCompositeCollider(uint numCompositeKeyBits, MTransform worldFromParent)
            {
                m_WorldFromA = worldFromParent;
                m_KeyPath.PopChildKey(numCompositeKeyBits);
            }
        }

        private static unsafe void ConvexTerrain(
            Context context, ColliderKeyPair colliderKeys, [NoAlias] Collider* convexColliderA, [NoAlias] Collider* terrainColliderB, [NoAlias] in MTransform worldFromA, [NoAlias] in MTransform worldFromB,
            sfloat maxDistance, bool flipped)
        {
            ref var terrain = ref ((TerrainCollider*)terrainColliderB)->Terrain;

            Material materialA = ((ConvexColliderHeader*)convexColliderA)->Material;
            Material materialB = ((TerrainCollider*)terrainColliderB)->Material;

            CollisionResponsePolicy combinedCollisionResponse = Material.GetCombinedCollisionResponse(materialA, materialB);

            // Skip the shapes if any of them is marked with a "None" collision response
            if (combinedCollisionResponse == CollisionResponsePolicy.None)
            {
                return;
            }

            // Skip if the bodies have infinite mass and the materials don't want to raise any solver events,
            // since the resulting contacts can't have any effect during solving
            if (context.BodiesHaveInfiniteMass)
            {
                if (combinedCollisionResponse != CollisionResponsePolicy.RaiseTriggerEvents &&
                    combinedCollisionResponse != CollisionResponsePolicy.CollideRaiseCollisionEvents)
                {
                    return;
                }
            }

            // Get vertices from hull
            ref ConvexHull hull = ref ((ConvexCollider*)convexColliderA)->ConvexHull;
            float3* capsuleVertices = stackalloc float3[8];
            float3* vertices;
            int numVertices = hull.NumVertices;
            if (numVertices == 2)
            {
                // Add extra sample points along the capsule
                float3 v0 = hull.Vertices[0];
                float3 v1 = hull.Vertices[1];
                float3 t0 = sfloat.Zero;
                float3 t1 = sfloat.One;
                float3 d = sfloat.FromRaw(0x3e124925);
                for (int i = 0; i < 8; i++)
                {
                    capsuleVertices[i] = v0 * t0 + v1 * t1;
                    t0 += d;
                    t1 -= d;
                }
                vertices = capsuleVertices;
                numVertices = 8;
            }
            else
            {
                vertices = hull.VerticesPtr;
            }

            // Create manifold(s)
            var manifold = new ConvexConvexManifoldQueries.Manifold();
            MTransform bFromA = Math.Mul(Math.Inverse(worldFromB), worldFromA);
            for (int iVertex = 0; iVertex < numVertices; iVertex++)
            {
                float3 pointAInB = Math.Mul(bFromA, vertices[iVertex]);
                float3 normalInB = float3.zero;
                if (terrain.GetHeightAndGradient(pointAInB.xz, out sfloat height, out float2 gradient))
                {
                    float3 normal = math.normalize(new float3(gradient.x, sfloat.One, gradient.y));
                    sfloat distance = (pointAInB.y - height) * normal.y;
                    if (distance < maxDistance + hull.ConvexRadius)
                    {
                        // The current manifold must be flushed if it's full or the normals don't match
                        if (math.dot(normalInB, normal) < sfloat.FromRaw(0x3f7fff58) || manifold.NumContacts == ConvexConvexManifoldQueries.Manifold.k_MaxNumContacts)
                        {
                            WriteManifold(manifold, context, colliderKeys, materialA, materialB, flipped);
                            manifold = new ConvexConvexManifoldQueries.Manifold
                            {
                                Normal = math.mul(worldFromB.Rotation, normal)
                            };
                            normalInB = normal;
                        }

                        manifold[manifold.NumContacts++] = new ContactPoint
                        {
                            Position = Math.Mul(worldFromB, pointAInB - normal * distance),
                            Distance = distance - hull.ConvexRadius
                        };
                    }
                }
            }

            // Flush the last manifold
            WriteManifold(manifold, context, colliderKeys, materialA, materialB, flipped);
        }

        private static unsafe void TerrainConvex(
            Context context, ColliderKeyPair colliderKeys,
            Collider* terrainColliderA, Collider* convexColliderB, MTransform worldFromA, MTransform worldFromB,
            sfloat maxDistance, bool flipped)
        {
            ConvexTerrain(context, colliderKeys, convexColliderB, terrainColliderA, worldFromB, worldFromA, maxDistance, !flipped);
        }

        private static unsafe void CompositeTerrain(
            Context context, Collider* compositeColliderA, Collider* terrainColliderB, MTransform worldFromA, MTransform worldFromB,
            sfloat maxDistance, bool flipped)
        {
            var collector = new CompositeTerrainLeafCollector(
                context, compositeColliderA, terrainColliderB, worldFromA, worldFromB, maxDistance, flipped);
            compositeColliderA->GetLeaves(ref collector);
        }

        private static unsafe void TerrainComposite(
            Context context, Collider* terrainColliderA, Collider* compositeColliderB, MTransform worldFromA, MTransform worldFromB,
            sfloat maxDistance, bool flipped)
        {
            CompositeTerrain(context, compositeColliderB, terrainColliderA, worldFromB, worldFromA, maxDistance, !flipped);
        }

        private unsafe struct CompositeTerrainLeafCollector : ILeafColliderCollector
        {
            readonly Context m_Context;
            readonly Collider* m_CompositeColliderA;
            readonly Collider* m_TerrainColliderB;
            MTransform m_WorldFromA;
            readonly MTransform m_WorldFromB;
            readonly sfloat m_MaxDistance;
            readonly bool m_Flipped;

            ColliderKeyPath m_KeyPath;

            public CompositeTerrainLeafCollector(
                Context context,
                Collider* compositeColliderA, Collider* terrainColliderB,
                MTransform worldFromA, MTransform worldFromB, sfloat maxDistance, bool flipped)
            {
                m_Context = context;
                m_CompositeColliderA = compositeColliderA;
                m_TerrainColliderB = terrainColliderB;
                m_WorldFromA = worldFromA;
                m_WorldFromB = worldFromB;
                m_MaxDistance = maxDistance;
                m_Flipped = flipped;
                m_KeyPath = ColliderKeyPath.Empty;
            }

            public void AddLeaf(ColliderKey key, ref ChildCollider leaf)
            {
                MTransform worldFromLeafA = Mul(m_WorldFromA, new MTransform(leaf.TransformFromChild));
                var colliderKeys = new ColliderKeyPair
                {
                    ColliderKeyA = m_KeyPath.GetLeafKey(key),
                    ColliderKeyB = ColliderKey.Empty
                };

                ConvexTerrain(
                    m_Context, colliderKeys, leaf.Collider, m_TerrainColliderB,
                    worldFromLeafA, m_WorldFromB, m_MaxDistance, m_Flipped);
            }

            public void PushCompositeCollider(ColliderKeyPath compositeKey, MTransform parentFromComposite, out MTransform worldFromParent)
            {
                m_KeyPath.PushChildKey(compositeKey);
                worldFromParent = m_WorldFromA;
                m_WorldFromA = Math.Mul(worldFromParent, parentFromComposite);
            }

            public void PopCompositeCollider(uint numCompositeKeyBits, MTransform worldFromParent)
            {
                m_WorldFromA = worldFromParent;
                m_KeyPath.PopChildKey(numCompositeKeyBits);
            }
        }
    }
}
