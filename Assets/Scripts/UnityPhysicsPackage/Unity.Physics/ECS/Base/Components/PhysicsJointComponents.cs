using System;
using System.Collections.Generic;
using System.ComponentModel;
using Unity.Collections;
using Unity.Entities;
using UnityS.Mathematics;
using UnityS.Physics.Extensions;
using Unity.Properties;
using FloatRange = UnityS.Physics.Math.FloatRange;

namespace UnityS.Physics
{
    /// <summary>
    /// A pair of bodies with some constraint between them (either a joint or a motor)
    /// </summary>
    public struct PhysicsConstrainedBodyPair : IComponentData
    {
        internal EntityPair Entities;

        /// <summary>
        /// Specifies whether the two bodies in the pair should generate contact events.
        /// </summary>
        public int EnableCollision;

        /// <summary>
        /// The first entity in the pair.
        /// </summary>
        [CreateProperty]
        public Entity EntityA => Entities.EntityA;
        /// <summary>
        /// The second entity in the pair.
        /// </summary>
        [CreateProperty]
        public Entity EntityB => Entities.EntityB;

        public PhysicsConstrainedBodyPair(Entity entityA, Entity entityB, bool enableCollision)
        {
            Entities = new EntityPair { EntityA = entityA, EntityB = entityB };
            EnableCollision = enableCollision ? 1 : 0;
        }
    }

    /// <summary>
    /// A buffer element to indicate additional entities in a complex joint configuration.
    /// Most joint types can be described with a single <see cref="PhysicsJoint"/>, but complex setups like ragdoll joints require more than one joint to stabilize.
    /// This component allows you to locate all of the other joints in such setups to facilitate group destruction, deactivation, and so on.
    /// Each joint entity in the group should have this component, with one element for each other joint in the group.
    /// </summary>
    [InternalBufferCapacity(4)]
    public struct PhysicsJointCompanion : IBufferElementData
    {
        /// <summary>
        /// Another entity presumed to have <see cref="PhysicsConstrainedBodyPair"/> and <see cref="PhysicsJoint"/>, as well as <see cref="PhysicsJointCompanion"/> with at least one element whose value matches the entity to which this buffer is added.
        /// The <see cref="PhysicsConstrainedBodyPair"/> of this other entity is further presumed to have the same value as that for the entity to which this buffer is added.
        /// </summary>
        public Entity JointEntity;
    }

    /// <summary>
    /// A designation of one of several types of canonical joints.
    /// </summary>
    public enum JointType : byte
    {
        /// <summary>
        /// The default type.
        /// Use this value to designate a <see cref="Constraint"/> configuration that does not map to a common type of joint, or to designate unknown configurations.
        /// </summary>
        Custom,
        /// <summary>
        /// A joint constraining the anchor point on body A to the target point on body B, about which the bodies can freely rotate.
        /// </summary>
        BallAndSocket,
        /// <summary>
        /// A joint constraining the position and orientation on body A to the target position and orientation on body B.
        /// </summary>
        Fixed,
        /// <summary>
        /// A joint constraining the anchor point on body A to the target point on body B, where each body can rotate freely about the vector where their axes align.
        /// The perpendicular axis of each body frame has no effect.
        /// </summary>
        Hinge,
        /// <summary>
        /// A joint constraining the anchor point on body A to the target point on body B within a specified linear distance range.
        /// </summary>
        LimitedDistance,
        /// <summary>
        /// A joint constraining the anchor point on body A to the target point on body B, where each body can rotate within a limited range about the vector where their axes align.
        /// The perpendicular axis of each body frame is used as a reference point for the range of motion.
        /// </summary>
        LimitedHinge,
        /// <summary>
        /// A joint constraining the anchor point on body A to the target point on body B, where the bodies' orientations are locked to one another and they can translate along the vector where the bodies' axes align.
        /// </summary>
        Prismatic,
        /// <summary>
        /// A joint constraining the orientation of body A to the orientation of body B, where the relative orientation of the bodies' axes may fall within a conical range of motion.
        /// The bodies' perpendicular axes may further rotate about their axes within a limited range of motion.
        /// This type is used in conjunction with <see cref="RagdollPerpendicularCone"/>.
        /// </summary>
        RagdollPrimaryCone,
        /// <summary>
        /// A joint constraining the anchor point on body A to the target point on body B, where the relative orientation of the bodies' perpendicular axes may fall within a range of motion between two cones aligned with the primary axis.
        /// This type is used in conjunction with <see cref="RagdollPrimaryCone"/>, where body B's axis for this joint corresponds with body A's primary axis in the primary cone.
        /// </summary>
        RagdollPerpendicularCone,
        /// <summary>
        /// A joint constraining the degrees of freedom of body A to a reference frame in the space of body B.
        /// </summary>
        LimitedDegreeOfFreedom,
    }

    /// <summary>
    /// A set of constraints on the relative motion of a <see cref="PhysicsConstrainedBodyPair"/>.
    /// Most joint types can be described with a single instance, but complex setups like ragdoll joints require more than one instance to stabilize.
    /// In these cases, you should associate multiple joints using <see cref="PhysicsJointCompanion"/>.
    /// </summary>
    public struct PhysicsJoint : IComponentData
    {
        BodyFrame m_BodyAFromJoint;
        BodyFrame m_BodyBFromJoint;
        byte m_Version;
        JointType m_JointType;
        FixedList128<Constraint> m_Constraints;

        /// <summary>
        /// The anchor point and orientation in the space of the first body.
        /// </summary>
        [CreateProperty]
        public BodyFrame BodyAFromJoint
        {
            get => m_BodyAFromJoint;
            set
            {
                if (m_BodyAFromJoint.Equals(value))
                    return;
                m_BodyAFromJoint = value;
                ++m_Version;
            }
        }

        /// <summary>
        /// The target point and orientation in the space of the second body.
        /// </summary>
        [CreateProperty]
        public BodyFrame BodyBFromJoint
        {
            get => m_BodyBFromJoint;
            set
            {
                if (m_BodyBFromJoint.Equals(value))
                    return;
                m_BodyBFromJoint = value;
                ++m_Version;
            }
        }

        /// <summary>
        /// A counter to keep track of changes to the joint's definition.
        /// Use it with back-ends like Havok that cache information about the joint if its properties have not changed.
        /// </summary>
        [CreateProperty]
        public byte Version => m_Version;

        /// <summary>
        /// An optional property to provide a hint about what behavior the underlying constraints represent.
        /// Use it in conjunction with convenience setters in <see cref="JointComponentExtensions"/> to modify constraints.
        /// Its value is set when you use a factory method to construct a new <see cref="PhysicsJoint"/> instance, but it has no effect on joint behavior.
        /// See also
        /// <seealso cref="CreateBallAndSocket"/>,
        /// <seealso cref="CreateFixed"/>,
        /// <seealso cref="CreateHinge"/>,
        /// <seealso cref="CreateLimitedDistance"/>,
        /// <seealso cref="CreateLimitedHinge"/>,
        /// <seealso cref="CreatePrismatic"/>,
        /// <seealso cref="CreateRagdoll"/>.
        /// <seealso cref="CreateLimitedDOF"/>.
        /// </summary>
        [CreateProperty]
        public JointType JointType
        {
            get => m_JointType;
            set
            {
                if (m_JointType == value)
                    return;
                m_JointType = value;
                ++m_Version;
            }
        }

        [CreateProperty]
        [Obsolete("This property is only included to display data in the Entity Inspector. It should not be used.", true)]
        IEnumerable<Constraint> Constraints => m_Constraints.ToArray();

        internal Constraint this[int constraintIndex] => m_Constraints[constraintIndex];

        /// <summary>
        /// Get the sequence of <see cref="Constraint"/> atoms to apply between the two bodies.
        /// </summary>
        public FixedList128<Constraint> GetConstraints() => m_Constraints;

        /// <summary>
        /// Set the sequence of <see cref="Constraint"/> atoms to apply between the two bodies.
        /// </summary>
        /// <param name="constraints">A sequence of <see cref="Constraint"/> atoms to apply in order.</param>
        public void SetConstraints(FixedList128<Constraint> constraints)
        {
            for (var i = 0; i < constraints.Length; ++i)
            {
                var limits = new FloatRange(constraints[i].Min, constraints[i].Max).Sorted();
                constraints.ElementAt(i).Min = limits.Min;
                constraints.ElementAt(i).Max = limits.Max;
            }
            if (m_Constraints.Equals(constraints))
                return;
            m_Constraints = constraints;
            ++m_Version;
        }

        #region Constructors

        /// <summary>
        /// Create a <see cref="JointType.BallAndSocket"/> joint.
        /// </summary>
        /// <param name="anchorA">Specifies the anchor point in the space of body A.</param>
        /// <param name="anchorB">Specifies the target point in the space of body B.</param>
        public static PhysicsJoint CreateBallAndSocket(float3 anchorA, float3 anchorB)
        {
            var bodyAFromJoint = BodyFrame.Identity;
            bodyAFromJoint.Position = anchorA;
            var bodyBFromJoint = BodyFrame.Identity;
            bodyBFromJoint.Position = anchorB;
            return new PhysicsJoint
            {
                BodyAFromJoint = bodyAFromJoint,
                BodyBFromJoint = bodyBFromJoint,
                m_JointType = JointType.BallAndSocket,
                m_Constraints = new FixedList128<Constraint>
                {
                    Length = 1,
                    [0] = Constraint.BallAndSocket(Constraint.DefaultSpringFrequency, Constraint.DefaultSpringDamping)
                }
            };
        }

        /// <summary>
        /// Create a <see cref="JointType.Fixed"/> joint.
        /// </summary>
        /// <param name="bodyAFromJoint">Specifies a reference point and orientation in the space of body A.</param>
        /// <param name="bodyBFromJoint">Specifies a target point and orientation in the space of body B.</param>
        public static PhysicsJoint CreateFixed(BodyFrame bodyAFromJoint, BodyFrame bodyBFromJoint) =>
            new PhysicsJoint
            {
                BodyAFromJoint = bodyAFromJoint,
                BodyBFromJoint = bodyBFromJoint,
                m_JointType = JointType.Fixed,
                m_Constraints = new FixedList128<Constraint>
                {
                    Length = 2,
                    [0] = Constraint.BallAndSocket(Constraint.DefaultSpringFrequency, Constraint.DefaultSpringDamping),
                    [1] = Constraint.FixedAngle(Constraint.DefaultSpringFrequency, Constraint.DefaultSpringDamping)
                }
            };

        /// <summary>
        /// Create a <see cref="JointType.Hinge"/> joint.
        /// </summary>
        /// <param name="bodyAFromJoint">Specifies the anchor point and axis of rotation in the space of body A.</param>
        /// <param name="bodyBFromJoint">Specifies the target point and axis of alignment in the space of body B.</param>
        public static PhysicsJoint CreateHinge(BodyFrame bodyAFromJoint, BodyFrame bodyBFromJoint) =>
            new PhysicsJoint
            {
                BodyAFromJoint = bodyAFromJoint,
                BodyBFromJoint = bodyBFromJoint,
                m_JointType = JointType.Hinge,
                m_Constraints = new FixedList128<Constraint>
                {
                    Length = 2,
                    [0] = Constraint.Hinge(0, Constraint.DefaultSpringFrequency, Constraint.DefaultSpringDamping),
                    [1] = Constraint.BallAndSocket(Constraint.DefaultSpringFrequency, Constraint.DefaultSpringDamping)
                }
            };

        internal const int k_LimitedDistanceRangeIndex = 0;

        /// <summary>
        /// Create a <see cref="JointType.LimitedDistance"/> joint.
        /// </summary>
        /// <param name="anchorA">Specifies the anchor point in the space of body A.</param>
        /// <param name="anchorB">Specifies the target point in the space of body B.</param>
        /// <param name="distanceRange">The minimum required distance and maximum possible distance between the two anchor points.</param>
        public static PhysicsJoint CreateLimitedDistance(float3 anchorA, float3 anchorB, FloatRange distanceRange)
        {
            var bodyAFromJoint = BodyFrame.Identity;
            bodyAFromJoint.Position = anchorA;
            var bodyBFromJoint = BodyFrame.Identity;
            bodyBFromJoint.Position = anchorB;
            return new PhysicsJoint
            {
                BodyAFromJoint = bodyAFromJoint,
                BodyBFromJoint = bodyBFromJoint,
                m_JointType = JointType.LimitedDistance,
                m_Constraints = new FixedList128<Constraint>
                {
                    Length = 1,
                    [k_LimitedDistanceRangeIndex] = Constraint.LimitedDistance(distanceRange.Sorted(), Constraint.DefaultSpringFrequency, Constraint.DefaultSpringDamping)
                }
            };
        }

        internal const int k_LimitedHingeRangeIndex = 0;

        /// <summary>
        /// Create a <see cref="JointType.LimitedHinge"/> joint.
        /// </summary>
        /// <param name="bodyAFromJoint">Specifies the anchor point, axis of rotation, and rest orientation in the space of body A.</param>
        /// <param name="bodyBFromJoint">Specifies the target point, axis of alignment, and reference orientation in the space of body B.</param>
        /// <param name="angularRange">The minimum required and maximum possible angle of rotation about the aligned axes.</param>
        public static PhysicsJoint CreateLimitedHinge(
            BodyFrame bodyAFromJoint, BodyFrame bodyBFromJoint, FloatRange angularRange
        ) =>
            new PhysicsJoint
            {
                BodyAFromJoint = bodyAFromJoint,
                BodyBFromJoint = bodyBFromJoint,
                m_JointType = JointType.LimitedHinge,
                m_Constraints = new FixedList128<Constraint>
                {
                    Length = 3,
                    [k_LimitedHingeRangeIndex]     = Constraint.Twist(0, angularRange.Sorted(), Constraint.DefaultSpringFrequency, Constraint.DefaultSpringDamping),
                    [k_LimitedHingeRangeIndex + 1] = Constraint.Hinge(0, Constraint.DefaultSpringFrequency, Constraint.DefaultSpringDamping),
                    [k_LimitedHingeRangeIndex + 2] = Constraint.BallAndSocket(Constraint.DefaultSpringFrequency, Constraint.DefaultSpringDamping)
                }
            };

        internal const int k_PrismaticDistanceOnAxisIndex = 1;

        /// <summary>
        /// Create a <see cref="JointType.Prismatic"/> joint.
        /// </summary>
        /// <param name="bodyAFromJoint">Specifies the anchor point and axis of rotation in the space of body A.</param>
        /// <param name="bodyBFromJoint">Specifies the target point and axis of alignment in the space of body B.</param>
        /// <param name="distanceOnAxis">The minimum required and maximum possible distance between the two anchor points along their aligned axes.</param>
        public static PhysicsJoint CreatePrismatic(
            BodyFrame bodyAFromJoint, BodyFrame bodyBFromJoint, FloatRange distanceOnAxis
        ) =>
            new PhysicsJoint
            {
                BodyAFromJoint = bodyAFromJoint,
                BodyBFromJoint = bodyBFromJoint,
                m_JointType = JointType.Prismatic,
                m_Constraints = new FixedList128<Constraint>
                {
                    Length = 3,
                    [0]                                  = Constraint.FixedAngle(Constraint.DefaultSpringFrequency, Constraint.DefaultSpringDamping),
                    [k_PrismaticDistanceOnAxisIndex]     = Constraint.Planar(0, distanceOnAxis.Sorted(), Constraint.DefaultSpringFrequency, Constraint.DefaultSpringDamping),
                    [k_PrismaticDistanceOnAxisIndex + 1] = Constraint.Cylindrical(0, float2.zero, Constraint.DefaultSpringFrequency, Constraint.DefaultSpringDamping)
                }
            };

        internal const int k_RagdollPrimaryMaxConeIndex = 1;
        internal const int k_RagdollPrimaryTwistRangeIndex = 0;
        internal const int k_RagdollPerpendicularRangeIndex = 0;

        /// <summary>
        /// Create a <see cref="JointType.RagdollPrimaryCone"/> and <see cref="JointType.RagdollPerpendicularCone"/> joint suitable for multi-axial joints on characters.
        /// The primary axis of the joined bodies can rotate within a range of motion defined by the maximum cone angle, minus the intersection with a pair of cones along the perpendicular axis.
        /// The bodies may also twist about the primary axis within this range.
        /// </summary>
        /// <param name="bodyAFromJoint">Specifies the anchor point, axis of rotation, and rest orientation in the space of body A.</param>
        /// <param name="bodyBFromJoint">Specifies the target point, axis of alignment, and reference orientation in the space of body B.</param>
        /// <param name="maxConeAngle">Half angle of the primary cone, which defines the maximum possible range of motion in which the primary axis is restricted. This value is clamped to the range (-pi, pi).</param>
        /// <param name="angularPlaneRange">The range of angular motion defining the cones perpendicular to the primary cone, between which the primary axis may swing. This range may be asymmetrical, and is clamped to the range (-pi/2, pi/2)./param>
        /// <param name="angularTwistRange">The range of angular motion for twisting around the primary axis within the region defined by the primary and perpendicular cones. This range is usually symmetrical, and is clamped to the range (-pi, pi).</param>
        /// <param name="primaryConeAndTwist">The joint defining the maximum conical range of the primary axis and the angular range of motion about it.</param>
        /// <param name="perpendicularCone">The joint defining the conic sections about the perpendicular axis to subtract from the primary cone.</param>
        public static void CreateRagdoll(
            BodyFrame bodyAFromJoint, BodyFrame bodyBFromJoint,
            sfloat maxConeAngle, FloatRange angularPlaneRange, FloatRange angularTwistRange,
            out PhysicsJoint primaryConeAndTwist, out PhysicsJoint perpendicularCone
        )
        {
            primaryConeAndTwist = new PhysicsJoint
            {
                BodyAFromJoint = bodyAFromJoint,
                BodyBFromJoint = bodyBFromJoint,
                m_JointType = JointType.RagdollPrimaryCone,
                m_Constraints = new FixedList128<Constraint>
                {
                    Length = 2,
                    [k_RagdollPrimaryTwistRangeIndex] = Constraint.Twist(0, math.clamp(angularTwistRange.Sorted(), new float2(-math.PI), new float2(math.PI)), Constraint.DefaultSpringFrequency, Constraint.DefaultSpringDamping),
                    [k_RagdollPrimaryMaxConeIndex]    = Constraint.Cone(0, new FloatRange(sfloat.Zero, math.min(math.abs(maxConeAngle), math.PI)), Constraint.DefaultSpringFrequency, Constraint.DefaultSpringDamping)
                }
            };

            perpendicularCone = new PhysicsJoint
            {
                BodyAFromJoint = bodyAFromJoint,
                BodyBFromJoint = new BodyFrame
                {
                    Axis = bodyBFromJoint.PerpendicularAxis,
                    PerpendicularAxis = bodyBFromJoint.Axis,
                    Position = bodyBFromJoint.Position
                },
                m_JointType = JointType.RagdollPerpendicularCone,
                m_Constraints = new FixedList128<Constraint>
                {
                    Length = 2,
                    [k_RagdollPerpendicularRangeIndex]     = Constraint.Cone(0, math.clamp(angularPlaneRange.Sorted() + new float2(sfloat.FromRaw(0x3fc90fdb)), new float2(sfloat.Zero), new float2(math.PI)), Constraint.DefaultSpringFrequency, Constraint.DefaultSpringDamping),
                    [k_RagdollPerpendicularRangeIndex + 1] = Constraint.BallAndSocket(Constraint.DefaultSpringFrequency, Constraint.DefaultSpringDamping)
                }
            };
        }

        internal const int k_LimitedDOFLinearIndex = 0;
        internal const int k_LimitedDOFAngularIndex = 1;

        /// <summary>
        /// Create a <see cref="JointType.LimitedDegreeOfFreedom"/> joint.
        /// </summary>
        /// <param name="offset">Specifies a target point and orientation in the space of body B.</param>
        /// <param name="linearLocks">Specifies which linear axes are constrained.</param>
        /// <param name="angularLocks">Specifies which angular axes are constrained.</param>
        public static PhysicsJoint CreateLimitedDOF(RigidTransform offset, bool3 linearLocks, bool3 angularLocks)
        {
            var joint = new PhysicsJoint
            {
                BodyAFromJoint = BodyFrame.Identity,
                BodyBFromJoint = offset,
                m_JointType = JointType.LimitedDegreeOfFreedom,
                m_Constraints = new FixedList128<Constraint>
                {
                    Length = 2,
                    [k_LimitedDOFLinearIndex] = new Constraint
                    {
                        ConstrainedAxes = linearLocks,
                        Type = ConstraintType.Linear,
                        Min = sfloat.Zero,
                        Max = sfloat.Zero,
                        SpringFrequency = Constraint.DefaultSpringFrequency,
                        SpringDamping = Constraint.DefaultSpringDamping
                    },
                    [k_LimitedDOFAngularIndex] = new Constraint
                    {
                        ConstrainedAxes = angularLocks,
                        Type = ConstraintType.Angular,
                        Min = sfloat.Zero,
                        Max = sfloat.Zero,
                        SpringFrequency = Constraint.DefaultSpringFrequency,
                        SpringDamping = Constraint.DefaultSpringDamping
                    },
                }
            };
            return joint;
        }

        #endregion
    }
}
