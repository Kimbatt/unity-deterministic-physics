using System.Runtime.CompilerServices;
using UnityS.Mathematics;

namespace UnityS.Physics
{
    // Describes how mass is distributed within an object
    // Represented by a transformed box inertia of unit mass.
    public struct MassDistribution
    {
        // The center of mass and the orientation to principal axis space
        public RigidTransform Transform;

        // Diagonalized inertia tensor for a unit mass
        public float3 InertiaTensor;

        // Get the inertia as a 3x3 matrix
        public float3x3 InertiaMatrix
        {
            get
            {
                var r = new float3x3(Transform.rot);
                var r2 = new float3x3(InertiaTensor.x * r.c0, InertiaTensor.y * r.c1, InertiaTensor.z * r.c2);
                return math.mul(r2, math.inverse(r));
            }
        }
    }

    // The mass properties of an object.
    public struct MassProperties
    {
        // The distribution of a unit mass throughout the object.
        public MassDistribution MassDistribution;

        // The volume of the object.
        public sfloat Volume;

        // Upper bound on the rate of change of the object's extent in any direction,
        // with respect to angular speed around its center of mass.
        // Used to determine how much to expand a rigid body's AABB to enclose its swept volume.
        public sfloat AngularExpansionFactor;

        // The mass properties of a unit sphere
        public static MassProperties UnitSphere => new MassProperties
        {
            MassDistribution = new MassDistribution
            {
                Transform = RigidTransform.identity,
                InertiaTensor = new float3(sfloat.FromRaw(0x3ecccccd))
            },
            Volume = sfloat.FromRaw(0x40860a92),
            AngularExpansionFactor = sfloat.Zero
        };
    }

    // A dynamic rigid body's "cold" motion data, used during Jacobian building and integration.
    public struct MotionData
    {
        // Center of mass and inertia orientation in world space
        public RigidTransform WorldFromMotion;

        // Center of mass and inertia orientation in rigid body space
        public RigidTransform BodyFromMotion;

        // Damping applied to the motion during each simulation step
        public sfloat LinearDamping;
        public sfloat AngularDamping;

        public static MotionData Zero => new MotionData
        {
            WorldFromMotion = RigidTransform.identity,
            BodyFromMotion = RigidTransform.identity,
            LinearDamping = sfloat.Zero,
            AngularDamping = sfloat.Zero
        };
    }

    // A dynamic rigid body's "hot" motion data, used during solving.
    public struct MotionVelocity
    {
        public float3 LinearVelocity;   // world space
        public float3 AngularVelocity;  // motion space
        public float3 PreviousLinearVelocity;
        public float3 PreviousAngularVelocity;
        public float3 InverseInertia;
        public sfloat InverseMass;
        public sfloat AngularExpansionFactor;

        // A multiplier applied to the simulation step's gravity vector
        public sfloat GravityFactor;

        internal bool HasInfiniteInertiaAndMass => !math.any(InverseInertia) && InverseMass.IsZero();

        public static MotionVelocity Zero => new MotionVelocity
        {
            LinearVelocity = float3.zero,
            AngularVelocity = float3.zero,
            PreviousLinearVelocity = float3.zero,
            PreviousAngularVelocity = float3.zero,
            InverseInertia = float3.zero,
            InverseMass = sfloat.Zero,
            AngularExpansionFactor = sfloat.Zero,
            GravityFactor = sfloat.Zero
        };

        // Apply a linear impulse (in world space)
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ApplyLinearImpulse(float3 impulse)
        {
            LinearVelocity += impulse * InverseMass;
        }

        // Apply an angular impulse (in motion space)
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ApplyAngularImpulse(float3 impulse)
        {
            AngularVelocity += impulse * InverseInertia;
        }

        // Calculate the distances by which to expand collision tolerances based on the speed of the object.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal MotionExpansion CalculateExpansion(sfloat timeStep) => new MotionExpansion
        {
            Linear = LinearVelocity * timeStep,
            // math.length(AngularVelocity) * timeStep is conservative approximation of sin((math.length(AngularVelocity) * timeStep)
            Uniform = math.min(math.length(AngularVelocity) * timeStep * AngularExpansionFactor, AngularExpansionFactor)
        };
    }

    // Provides an upper bound on change in a body's extents in any direction during a step.
    // Used to determine how far away from the body to look for collisions.
    struct MotionExpansion
    {
        public float3 Linear;   // how far to look ahead of the object
        public sfloat Uniform;   // how far to look around the object

        public sfloat MaxDistance => math.length(Linear) + Uniform;

        public static readonly MotionExpansion Zero = new MotionExpansion
        {
            Linear = new float3(sfloat.Zero),
            Uniform = sfloat.Zero
        };

        // Expand an AABB
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Aabb ExpandAabb(Aabb aabb) => new Aabb
        {
            Max = math.max(aabb.Max, aabb.Max + Linear) + Uniform,
            Min = math.min(aabb.Min, aabb.Min + Linear) - Uniform
        };
    }

    // A linear and angular velocity
    public struct Velocity
    {
        public float3 Linear;   // world space
        public float3 Angular;  // motion space

        public static readonly Velocity Zero = new Velocity
        {
            Linear = new float3(sfloat.Zero),
            Angular = new float3(sfloat.Zero)
        };
    }
}
