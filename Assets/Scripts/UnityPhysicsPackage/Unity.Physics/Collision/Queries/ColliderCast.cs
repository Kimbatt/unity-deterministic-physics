using System;
using Unity.Entities;
using UnityS.Mathematics;
using Unity.Collections.LowLevel.Unsafe;
using static UnityS.Physics.BoundingVolumeHierarchy;
using static UnityS.Physics.Math;

namespace UnityS.Physics
{
    // The input to collider cast queries consists of a Collider and its initial orientation,
    // and the Start & End positions of a line segment the Collider is to be swept along.
    public unsafe struct ColliderCastInput
    {
        [NativeDisableUnsafePtrRestriction] public Collider* Collider;
        public quaternion Orientation { get; set; }

        public float3 Start
        {
            get => Ray.Origin;
            set
            {
                float3 end = Ray.Origin + Ray.Displacement;
                Ray.Origin = value;
                Ray.Displacement = end - value;
            }
        }
        public float3 End
        {
            get => Ray.Origin + Ray.Displacement;
            set => Ray.Displacement = value - Ray.Origin;
        }

        internal Ray Ray;
        internal QueryContext QueryContext;

        public override string ToString() =>
            $"RaycastInput {{ Start = {Start}, End = {End}, Collider = {Collider->Type}, Orientation = {Orientation} }}";
    }

    // A hit from a collider cast query
    public struct ColliderCastHit : IQueryResult
    {
        /// <summary>
        /// Fraction of the distance along the Ray where the hit occurred.
        /// </summary>
        /// <value> Returns a value between 0 and 1. </value>
        public sfloat Fraction { get; set; }

        /// <summary>
        ///
        /// </summary>
        /// <value> Returns RigidBodyIndex of queried body.</value>
        public int RigidBodyIndex { get; set; }

        /// <summary>
        ///
        /// </summary>
        /// <value> Returns ColliderKey of queried leaf collider</value>
        public ColliderKey ColliderKey { get; set; }

        /// <summary>
        ///
        /// </summary>
        /// <value> Returns Material of queried leaf collider</value>
        public Material Material { get; set; }

        /// <summary>
        ///
        /// </summary>
        /// <value> Returns Entity of queried body</value>
        public Entity Entity { get; set; }

        /// <summary>
        /// The point in query space where the hit occurred.
        /// </summary>
        /// <value> Returns the position of the point where the hit occurred. </value>
        public float3 Position { get; set; }

        /// <summary>
        ///
        /// </summary>
        /// <value> Returns the normal of the point where the hit occurred. </value>
        public float3 SurfaceNormal { get; set; }

        public override string ToString() =>
            $"ColliderCastHit {{ Fraction = {Fraction}, RigidBodyIndex = {RigidBodyIndex}, ColliderKey = {ColliderKey}, Entity = {Entity}, Position = {Position}, SurfaceNormal = {SurfaceNormal} }}";
    }

    // Collider cast query implementations
    static class ColliderCastQueries
    {
        public static unsafe bool ColliderCollider<T>(ColliderCastInput input, Collider* target, ref T collector) where T : struct, ICollector<ColliderCastHit>
        {
            if (!CollisionFilter.IsCollisionEnabled(input.Collider->Filter, target->Filter))
            {
                return false;
            }

            if (!input.QueryContext.IsInitialized)
            {
                input.QueryContext = QueryContext.DefaultContext;
            }

            switch (input.Collider->CollisionType)
            {
                case CollisionType.Convex:
                    switch (target->Type)
                    {
                        case ColliderType.Sphere:
                        case ColliderType.Capsule:
                        case ColliderType.Triangle:
                        case ColliderType.Quad:
                        case ColliderType.Box:
                        case ColliderType.Cylinder:
                        case ColliderType.Convex:
                            if (ConvexConvex(input, target, collector.MaxFraction, out ColliderCastHit hit))
                            {
                                return collector.AddHit(hit);
                            }
                            return false;
                        case ColliderType.Mesh:
                            return ConvexMesh(input, (MeshCollider*)target, ref collector);
                        case ColliderType.Compound:
                            return ConvexCompound(input, (CompoundCollider*)target, ref collector);
                        case ColliderType.Terrain:
                            return ConvexTerrain(input, (TerrainCollider*)target, ref collector);
                        default:
                            SafetyChecks.ThrowNotImplementedException();
                            return default;
                    }
                case CollisionType.Composite:
                case CollisionType.Terrain:
                    // no support for casting composite shapes
                    SafetyChecks.ThrowNotImplementedException();
                    return default;
                default:
                    SafetyChecks.ThrowNotImplementedException();
                    return default;
            }
        }

        private static unsafe bool ConvexConvex(ColliderCastInput input, Collider* target, sfloat maxFraction, out ColliderCastHit hit)
        {
            hit = default;

            // Get the current transform
            MTransform targetFromQuery = new MTransform(input.Orientation, input.Start);

            // Conservative advancement
            sfloat tolerance = sfloat.FromRaw(0x3a83126f);      // return if this close to a hit
            sfloat keepDistance = sfloat.FromRaw(0x38d1b717);   // avoid bad cases for GJK (penetration / exact hit)
            int iterations = 10;                // return after this many advances, regardless of accuracy
            sfloat fraction = sfloat.Zero;

            while (true)
            {
                if (fraction >= maxFraction)
                {
                    // Exceeded the maximum fraction without a hit
                    return false;
                }

                // Find the current distance
                DistanceQueries.Result distanceResult = DistanceQueries.ConvexConvex(target, input.Collider, targetFromQuery);

                // Check for a hit
                if (distanceResult.Distance < tolerance || --iterations == 0)
                {
                    targetFromQuery.Translation = input.Start;
                    hit.Position = Mul(input.QueryContext.WorldFromLocalTransform, distanceResult.PositionOnBinA);
                    hit.SurfaceNormal = math.mul(input.QueryContext.WorldFromLocalTransform.Rotation, -distanceResult.NormalInA);
                    hit.Fraction = fraction;
                    hit.RigidBodyIndex = input.QueryContext.RigidBodyIndex;
                    hit.ColliderKey = input.QueryContext.ColliderKey;
                    hit.Material = ((ConvexColliderHeader*)target)->Material;
                    hit.Entity = input.QueryContext.Entity;

                    return true;
                }

                // Check for a miss
                sfloat dot = math.dot(distanceResult.NormalInA, input.Ray.Displacement);
                if (dot <= sfloat.Zero)
                {
                    // Collider is moving away from the target, it will never hit
                    return false;
                }

                // Advance
                fraction += (distanceResult.Distance - keepDistance) / dot;
                if (fraction >= maxFraction)
                {
                    // Exceeded the maximum fraction without a hit
                    return false;
                }

                targetFromQuery.Translation = math.lerp(input.Start, input.End, fraction);
            }
        }

        internal unsafe struct ConvexMeshLeafProcessor : IColliderCastLeafProcessor
        {
            private readonly Mesh* m_Mesh;
            private readonly uint m_NumColliderKeyBits;

            public ConvexMeshLeafProcessor(MeshCollider* meshCollider)
            {
                m_Mesh = &meshCollider->Mesh;
                m_NumColliderKeyBits = meshCollider->NumColliderKeyBits;
            }

            public bool ColliderCastLeaf<T>(ColliderCastInput input, int primitiveKey, ref T collector)
                where T : struct, ICollector<ColliderCastHit>
            {
                m_Mesh->GetPrimitive(primitiveKey, out float3x4 vertices, out Mesh.PrimitiveFlags flags, out CollisionFilter filter, out Material material);

                if (!CollisionFilter.IsCollisionEnabled(input.Collider->Filter, filter)) // TODO: could do this check within GetPrimitive()
                {
                    return false;
                }

                int numPolygons = Mesh.GetNumPolygonsInPrimitive(flags);
                bool isQuad = Mesh.IsPrimitiveFlagSet(flags, Mesh.PrimitiveFlags.IsQuad);

                bool acceptHit = false;

                var polygon = new PolygonCollider();
                polygon.InitNoVertices(CollisionFilter.Default, material);
                for (int polygonIndex = 0; polygonIndex < numPolygons; polygonIndex++)
                {
                    sfloat fraction = collector.MaxFraction;

                    if (isQuad)
                    {
                        polygon.SetAsQuad(vertices[0], vertices[1], vertices[2], vertices[3]);
                    }
                    else
                    {
                        polygon.SetAsTriangle(vertices[0], vertices[1 + polygonIndex], vertices[2 + polygonIndex]);
                    }

                    if (ConvexConvex(input, (Collider*)&polygon, collector.MaxFraction, out ColliderCastHit hit))
                    {
                        hit.ColliderKey = input.QueryContext.SetSubKey(m_NumColliderKeyBits, (uint)(primitiveKey << 1 | polygonIndex));
                        acceptHit |= collector.AddHit(hit);
                    }
                }

                return acceptHit;
            }
        }

        private static unsafe bool ConvexMesh<T>(ColliderCastInput input, MeshCollider* meshCollider, ref T collector)
            where T : struct, ICollector<ColliderCastHit>
        {
            var leafProcessor = new ConvexMeshLeafProcessor(meshCollider);
            return meshCollider->Mesh.BoundingVolumeHierarchy.ColliderCast(input, ref leafProcessor, ref collector);
        }

        internal unsafe struct ConvexCompoundLeafProcessor : IColliderCastLeafProcessor
        {
            private readonly CompoundCollider* m_CompoundCollider;

            public ConvexCompoundLeafProcessor(CompoundCollider* compoundCollider)
            {
                m_CompoundCollider = compoundCollider;
            }

            public bool ColliderCastLeaf<T>(ColliderCastInput input, int leafData, ref T collector)
                where T : struct, ICollector<ColliderCastHit>
            {
                ref CompoundCollider.Child child = ref m_CompoundCollider->Children[leafData];

                if (!CollisionFilter.IsCollisionEnabled(input.Collider->Filter, child.Collider->Filter))
                {
                    return false;
                }

                // Transform the cast into child space
                ColliderCastInput inputLs = input;
                RigidTransform childFromCompound = math.inverse(child.CompoundFromChild);
                inputLs.Ray.Origin = math.transform(childFromCompound, input.Ray.Origin);
                inputLs.Ray.Displacement = math.mul(childFromCompound.rot, input.Ray.Displacement);
                inputLs.Orientation = math.mul(childFromCompound.rot, input.Orientation);
                inputLs.QueryContext.ColliderKey = input.QueryContext.PushSubKey(m_CompoundCollider->NumColliderKeyBits, (uint)leafData);
                inputLs.QueryContext.NumColliderKeyBits = input.QueryContext.NumColliderKeyBits;
                inputLs.QueryContext.WorldFromLocalTransform = Mul(input.QueryContext.WorldFromLocalTransform, new MTransform(child.CompoundFromChild));

                return child.Collider->CastCollider(inputLs, ref collector);
            }
        }

        private static unsafe bool ConvexCompound<T>(ColliderCastInput input, CompoundCollider* compoundCollider, ref T collector)
            where T : struct, ICollector<ColliderCastHit>
        {
            var leafProcessor = new ConvexCompoundLeafProcessor(compoundCollider);
            return compoundCollider->BoundingVolumeHierarchy.ColliderCast(input, ref leafProcessor, ref collector);
        }

        private static unsafe bool ConvexTerrain<T>(ColliderCastInput input, TerrainCollider* terrainCollider, ref T collector)
            where T : struct, ICollector<ColliderCastHit>
        {
            ref Terrain terrain = ref terrainCollider->Terrain;
            Material material = terrainCollider->Material;

            bool hadHit = false;

            // Get a ray for the min corner of the AABB in tree-space and the extents of the AABB in tree-space
            float3 aabbExtents;
            Ray aabbRay;
            Terrain.QuadTreeWalker walker;
            {
                Aabb aabb = input.Collider->CalculateAabb(new RigidTransform(input.Orientation, input.Start));
                Aabb aabbInTree = new Aabb
                {
                    Min = aabb.Min * terrain.InverseScale,
                    Max = aabb.Max * terrain.InverseScale
                };
                aabbExtents = aabbInTree.Extents;
                aabbRay = new Ray
                {
                    Origin = aabbInTree.Min,
                    Displacement = input.Ray.Displacement * terrain.InverseScale
                };

                float3 maxDisplacement = aabbRay.Displacement * collector.MaxFraction;
                Aabb queryAabb = new Aabb
                {
                    Min = aabbInTree.Min + math.min(maxDisplacement, float3.zero),
                    Max = aabbInTree.Max + math.max(maxDisplacement, float3.zero)
                };
                walker = new Terrain.QuadTreeWalker(&terrainCollider->Terrain, queryAabb);
            }

            // Traverse the tree
            int numHits = collector.NumHits;
            while (walker.Pop())
            {
                FourTransposedAabbs bounds = walker.Bounds;
                bounds.Lx -= aabbExtents.x;
                bounds.Ly -= aabbExtents.y;
                bounds.Lz -= aabbExtents.z;
                bool4 hitMask = bounds.Raycast(aabbRay, collector.MaxFraction, out float4 hitFractions);
                hitMask &= (walker.Bounds.Ly <= walker.Bounds.Hy); // Mask off empty children
                if (walker.IsLeaf)
                {
                    // Leaf node, collidercast against hit child quads
                    int4 hitIndex;
                    int hitCount = math.compress((int*)(&hitIndex), 0, new int4(0, 1, 2, 3), hitMask);
                    for (int iHit = 0; iHit < hitCount; iHit++)
                    {
                        // Get the quad vertices
                        walker.GetQuad(hitIndex[iHit], out int2 quadIndex, out float3 a, out float3 b, out float3 c, out float3 d);

                        // Test each triangle in the quad
                        var polygon = new PolygonCollider();
                        polygon.InitNoVertices(CollisionFilter.Default, material);
                        for (int iTriangle = 0; iTriangle < 2; iTriangle++)
                        {
                            // Cast
                            sfloat fraction = collector.MaxFraction;
                            polygon.SetAsTriangle(a, b, c);

                            if (ConvexConvex(input, (Collider*)&polygon, collector.MaxFraction, out ColliderCastHit hit))
                            {
                                hit.ColliderKey = input.QueryContext.SetSubKey(terrain.NumColliderKeyBits, terrain.GetSubKey(quadIndex, iTriangle));
                                hadHit |= collector.AddHit(hit);
                            }

                            // Next triangle
                            a = c;
                            c = d;
                        }
                    }
                }
                else
                {
                    // Interior node, add hit child nodes to the stack
                    walker.Push(hitMask);
                }
            }

            return hadHit;
        }
    }
}
