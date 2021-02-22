using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using UnityS.Mathematics;
using UnityS.Physics.Extensions;
using UnityS.Transforms;

namespace UnityS.Physics
{
    /// <summary>
    /// Add this tag to a body or joint to exclude it from the physics world.
    /// This allows you to retain all of its other data, but temporarily ignore it for purposes of physics
    /// </summary>
    public struct PhysicsExclude : IComponentData { }

    // The collision geometry of a rigid body.
    // If not present, the rigid body cannot collide with anything.
    public struct PhysicsCollider : IComponentData
    {
        public BlobAssetReference<Collider> Value;  // null is allowed

        public bool IsValid => Value.IsCreated;
        public unsafe Collider* ColliderPtr => (Collider*)Value.GetUnsafePtr();
        public MassProperties MassProperties => Value.IsCreated ? Value.Value.MassProperties : MassProperties.UnitSphere;
    }

    // The mass properties of a rigid body.
    // If not present, the rigid body has infinite mass and inertia.
    public struct PhysicsMass : IComponentData
    {
        public RigidTransform Transform;        // center of mass and orientation of principal axes
        public sfloat InverseMass;               // zero is allowed, for infinite mass
        public float3 InverseInertia;           // zero is allowed, for infinite inertia
        public sfloat AngularExpansionFactor;    // see MassProperties.AngularExpansionFactor

        public float3 CenterOfMass { get => Transform.pos; set => Transform.pos = value; }
        public quaternion InertiaOrientation { get => Transform.rot; set => Transform.rot = value; }

        public static PhysicsMass CreateDynamic(MassProperties massProperties, sfloat mass)
        {
            SafetyChecks.CheckFiniteAndPositiveAndThrow(mass, nameof(mass));

            return new PhysicsMass
            {
                Transform = massProperties.MassDistribution.Transform,
                InverseMass = math.rcp(mass),
                InverseInertia = math.rcp(massProperties.MassDistribution.InertiaTensor * mass),
                AngularExpansionFactor = massProperties.AngularExpansionFactor
            };
        }

        public static PhysicsMass CreateKinematic(MassProperties massProperties)
        {
            return new PhysicsMass
            {
                Transform = massProperties.MassDistribution.Transform,
                InverseMass = sfloat.Zero,
                InverseInertia = float3.zero,
                AngularExpansionFactor = massProperties.AngularExpansionFactor
            };
        }
    }

    /// <summary>
    /// Add this component to a dynamic body if it needs to sometimes switch to being kinematic.
    /// This allows you to retain its dynamic mass properties on its <see cref="PhysicsMass"/> component, but have the physics solver temporarily treat it as if it were kinematic.
    /// Kinematic bodies will have infinite mass and inertia. They should also not be affected by gravity.
    /// Hence, if IsKinematic is non-zero the value in an associated <see cref="PhysicsGravityFactor"/> component is also ignored.
    /// </summary>
    public struct PhysicsMassOverride : IComponentData
    {
        public byte IsKinematic;
    }

    /// <summary>
    /// The velocity of a rigid body.
    /// If absent, the rigid body is static.
    /// </summary>
    public struct PhysicsVelocity : IComponentData
    {
        /// <summary>
        /// The body's world-space linear velocity in units per second.
        /// </summary>
        public float3 Linear;
        /// <summary>
        /// The body's angular velocity in radians per second about each principal axis specified by <see cref="PhysicsMass.Transform"/>.
        /// In order to get or set world-space values, use <see cref="ComponentExtensions.GetAngularVelocityWorldSpace"/> and <see cref="ComponentExtensions.SetAngularVelocityWorldSpace"/>, respectively.
        /// </summary>
        public float3 Angular;

        /// <summary>
        /// Create a <see cref="PhysicsVelocity"/> required to move a body to a target position and orientation.
        /// Use this method to control kinematic bodies directly if they need to generate contact events when moving to their new positions.
        /// If you need to teleport kinematic bodies you can simply set their Translation and Rotation values directly.
        /// </summary>
        /// <param name="bodyMass">The body's <see cref="PhysicsMass"/> component.</param>
        /// <param name="bodyPosition">The body's Translation component.</param>
        /// <param name="bodyOrientation">The body's Rotation component.</param>
        /// <param name="targetTransform">The desired Translation and Rotation values the body should move to in world space.</param>
        /// <param name="stepFrequency">The step frequency in the simulation where the body's motion is solved (i.e., 1 / FixedDeltaTime).</param>
        public static PhysicsVelocity CalculateVelocityToTarget(
            in PhysicsMass bodyMass, in Translation bodyPosition, in Rotation bodyOrientation,
            in RigidTransform targetTransform, in sfloat stepFrequency
        )
        {
            var velocity = new PhysicsVelocity();
            var worldFromBody = new RigidTransform(bodyOrientation.Value, bodyPosition.Value);
            var worldFromMotion = math.mul(worldFromBody, bodyMass.Transform);
            PhysicsWorldExtensions.CalculateVelocityToTargetImpl(
                worldFromBody, math.inverse(worldFromMotion.rot), bodyMass.Transform.pos, targetTransform, stepFrequency,
                out velocity.Linear, out velocity.Angular
            );
            return velocity;
        }
    }

    // Optional damping applied to the rigid body velocities during each simulation step.
    // This scales the velocities using: math.clamp(1 - damping * Timestep, 0, 1)
    public struct PhysicsDamping : IComponentData
    {
        public sfloat Linear;     // damping applied to the linear velocity
        public sfloat Angular;    // damping applied to the angular velocity
    }

    // Optional gravity factor applied to a rigid body during each simulation step.
    // This scales the gravity vector supplied to the simulation step.
    public struct PhysicsGravityFactor : IComponentData
    {
        public sfloat Value;
    }

    // Optional custom tags attached to a rigid body.
    // This will be copied to any contacts and Jacobians involving this rigid body,
    // providing additional context to any user logic operating on those structures.
    public struct PhysicsCustomTags : IComponentData
    {
        public byte Value;
    }

    // Parameters describing how to step the physics world.
    // If none is present in the scene, default values will be used.
    public struct PhysicsStep : IComponentData
    {
        public SimulationType SimulationType;
        public float3 Gravity;
        public int SolverIterationCount;
        public Solver.StabilizationHeuristicSettings SolverStabilizationHeuristicSettings;

        public byte MultiThreaded;

        // Whether to synchronize collision world after physics step to enable precise query results.
        // Note that `BuildPhysicsWorld` will do this work on the following frame anyway, so only use this option when
        // another system must know about the results of the simulation before the end of the frame
        // (e.g., to destroy or create some other body that must be present in the following frame).
        // In most cases, tolerating a frame of latency is easier to work with and is better for performance.
        public byte SynchronizeCollisionWorld;

        public static PhysicsStep Default => new PhysicsStep
        {
            SimulationType = SimulationType.UnityPhysics,
            Gravity = new float3(sfloat.Zero, sfloat.FromRaw(0xc11cf5c3), sfloat.Zero),
            SolverIterationCount = 4,
            SolverStabilizationHeuristicSettings = Solver.StabilizationHeuristicSettings.Default,
            MultiThreaded = 1,
            SynchronizeCollisionWorld = 0
        };
    }

    /// <summary>
    /// Proxy component data structure for CollisionWorld to use in DataFlowGraph nodes. 
    /// </summary>
    /// <seealso cref="ColliderCastNode"/>
    /// <seealso cref="RaycastNode"/>
    public unsafe struct CollisionWorldProxy : IComponentData
    {
        struct NativeArrayProxy<T> where T : struct
        {
            void* Ptr;
            int Size;

            public bool IsCreated => Ptr != null;

            public NativeArrayProxy(NativeArray<T> nativeArray)
            {
                Ptr = nativeArray.GetUnsafeReadOnlyPtr();
                Size = nativeArray.Length;
            }

            public NativeArrayProxy(NativeSlice<T> nativeSlice)
            {
                Ptr = nativeSlice.GetUnsafeReadOnlyPtr();
                Size = nativeSlice.Length;
            }

            public NativeArray<T> ToArray(AtomicSafetyManager* safetyManager)
            {
                var nativeArray = NativeArrayUnsafeUtility.ConvertExistingDataToNativeArray<T>((void*)Ptr, Size, Allocator.Invalid);
                safetyManager->MarkNativeArrayAsReadOnly(ref nativeArray);
                return nativeArray;
            }
        }

        struct BroadPhaseTreeProxy
        {
            NativeArrayProxy<BoundingVolumeHierarchy.Node> Nodes;
            NativeArrayProxy<CollisionFilter> NodeFilters;
            NativeArrayProxy<CollisionFilter> BodyFilters;
            NativeArrayProxy<BoundingVolumeHierarchy.Builder.Range> Ranges;
            NativeArrayProxy<int> BranchCount;

            public bool IsCreated => Nodes.IsCreated && NodeFilters.IsCreated && BodyFilters.IsCreated && Ranges.IsCreated && BranchCount.IsCreated;

            public BroadPhaseTreeProxy(Broadphase.Tree tree)
            {
                Nodes = new NativeArrayProxy<BoundingVolumeHierarchy.Node>(tree.Nodes);
                NodeFilters = new NativeArrayProxy<CollisionFilter>(tree.NodeFilters);
                BodyFilters = new NativeArrayProxy<CollisionFilter>(tree.BodyFilters);
                Ranges = new NativeArrayProxy<BoundingVolumeHierarchy.Builder.Range>(tree.Ranges);
                BranchCount = new NativeArrayProxy<int>(tree.BranchCount);
            }

            public Broadphase.Tree ToTree(AtomicSafetyManager* safetyManager)
            {
                return new Broadphase.Tree
                {
                    BranchCount = BranchCount.ToArray(safetyManager),
                    Nodes = Nodes.ToArray(safetyManager),
                    NodeFilters = NodeFilters.ToArray(safetyManager),
                    BodyFilters = BodyFilters.ToArray(safetyManager),
                    Ranges = Ranges.ToArray(safetyManager)
                };
            }
        }

        NativeArrayProxy<RigidBody> m_Bodies;
        BroadPhaseTreeProxy m_StaticTree;
        BroadPhaseTreeProxy m_DynamicTree;

        [NativeDisableUnsafePtrRestriction]
        readonly AtomicSafetyManager* m_SafetyManager;

        public bool IsCreated => m_Bodies.IsCreated && m_StaticTree.IsCreated && m_DynamicTree.IsCreated;

        internal CollisionWorldProxy(CollisionWorld collisionWorld, AtomicSafetyManager* safetyManager)
        {
            m_Bodies = new NativeArrayProxy<RigidBody>(collisionWorld.Bodies);
            m_StaticTree = new BroadPhaseTreeProxy(collisionWorld.Broadphase.StaticTree);
            m_DynamicTree = new BroadPhaseTreeProxy(collisionWorld.Broadphase.DynamicTree);
            m_SafetyManager = safetyManager;
        }

        public CollisionWorld ToCollisionWorld() =>
            new CollisionWorld(m_Bodies.ToArray(m_SafetyManager), new Broadphase(m_StaticTree.ToTree(m_SafetyManager), m_DynamicTree.ToTree(m_SafetyManager)));
    }
}
