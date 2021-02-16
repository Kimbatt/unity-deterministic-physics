using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using UnityS.Mathematics;
using UnityEngine.Assertions;
using static UnityS.Physics.Math;

namespace UnityS.Physics
{
    public enum ConstraintType : byte
    {
        Linear,
        Angular
    }

    // A linear or angular constraint in 1, 2, or 3 dimensions.
    public struct Constraint : IEquatable<Constraint>
    {
        // TODO think more about these
        // Current values give tau = 0.6 damping = 0.99 at 50hz
        // The values are huge and we can't get damping = 1 -- a stiff constraint is the limit of a damped spring as spring params go to infinity.
        public static sfloat DefaultSpringFrequency => sfloat.FromRaw(0x4771fefa); // 61950.977267809007887192914302327f
        public static sfloat DefaultSpringDamping => sfloat.FromRaw(0x451e21f2); // 2530.12155587434178122630287018f

        public bool3 ConstrainedAxes;
        public ConstraintType Type;

        public sfloat Min;
        public sfloat Max;
        public sfloat SpringFrequency;
        public sfloat SpringDamping;

        // Number of affected degrees of freedom.  1, 2, or 3.
        internal int Dimension
        {
            get
            {
                return math.select(math.select(math.select(2, 0, !math.any(ConstrainedAxes)), 1, ConstrainedAxes.x ^ ConstrainedAxes.y ^ ConstrainedAxes.z), 3, math.all(ConstrainedAxes));
            }
        }

        // Selects the free axis from a constraint with Dimension == 1
        internal int FreeAxis2D
        {
            get
            {
                Assert.IsTrue(Dimension == 2);
                return math.select(2, math.select(0, 1, ConstrainedAxes[0]), ConstrainedAxes[2]);
            }
        }

        // Selects the constrained axis from a constraint with Dimension == 2
        internal int ConstrainedAxis1D
        {
            get
            {
                Assert.IsTrue(Dimension == 1);
                return math.select(math.select(1, 0, ConstrainedAxes[0]), 2, ConstrainedAxes[2]);
            }
        }

        #region Common linear constraints

        /// <summary>
        /// Constrains linear motion about all three axes to zero.
        /// </summary>
        /// <param name="springFrequency"></param>
        /// <param name="springDamping"></param>
        public static Constraint BallAndSocket(sfloat springFrequency /* = DefaultSpringFrequency*/, sfloat springDamping /* = DefaultSpringDamping*/ )
        {
            return new Constraint
            {
                ConstrainedAxes = new bool3(true),
                Type = ConstraintType.Linear,
                Min = sfloat.Zero,
                Max = sfloat.Zero,
                SpringFrequency = springFrequency,
                SpringDamping = springDamping
            };
        }

        /// <summary>
        /// Constrains linear motion about all three axes within the specified range.
        /// </summary>
        /// <param name="distanceRange">The minimum required distance and maximum possible distance between the constrained bodies.</param>
        /// <param name="springFrequency"></param>
        /// <param name="springDamping"></param>
        public static Constraint LimitedDistance(FloatRange distanceRange, sfloat springFrequency /* = DefaultSpringFrequency */, sfloat springDamping /* = DefaultSpringDamping */)
        {
            distanceRange = distanceRange.Sorted();
            return new Constraint
            {
                ConstrainedAxes = new bool3(true),
                Type = ConstraintType.Linear,
                Min = distanceRange.Min,
                Max = distanceRange.Max,
                SpringFrequency = springFrequency,
                SpringDamping = springDamping
            };
        }

        /// <summary>
        /// Constrains linear motion about two axes within the specified range. Movement about the third is unrestricted.
        /// </summary>
        /// <param name="freeAxis">The axis along which the bodies may freely translate.</param>
        /// <param name="distanceRange">
        /// The minimum required distance and maximum possible distance between the constrained bodies about the two constrained axes.
        /// A minimum value of zero produces a cylindrical range of motion, while a minimum value greater than zero results in a tube-shaped range of motion.
        /// </param>
        /// <param name="springFrequency"></param>
        /// <param name="springDamping"></param>
        public static Constraint Cylindrical(int freeAxis, FloatRange distanceRange, sfloat springFrequency /* = DefaultSpringFrequency */, sfloat springDamping /* = DefaultSpringDamping */)
        {
            Assert.IsTrue(freeAxis >= 0 && freeAxis <= 2);
            distanceRange = distanceRange.Sorted();
            return new Constraint
            {
                ConstrainedAxes = new bool3(freeAxis != 0, freeAxis != 1, freeAxis != 2),
                Type = ConstraintType.Linear,
                Min = distanceRange.Min,
                Max = distanceRange.Max,
                SpringFrequency = springFrequency,
                SpringDamping = springDamping
            };
        }

        /// <summary>
        /// Constrains linear motion about one axis within the specified range. Movement about the other two is unrestricted.
        /// </summary>
        /// <param name="limitedAxis">The axis along which the bodies' translation is restricted.</param>
        /// <param name="distanceRange">
        /// The minimum required distance and maximum possible distance between the constrained bodies about the constrained axis.
        /// Identical minimum and maximum values result in a plane, while different values constrain the bodies between two parallel planes.
        /// </param>
        /// <param name="springFrequency"></param>
        /// <param name="springDamping"></param>
        public static Constraint Planar(int limitedAxis, FloatRange distanceRange, sfloat springFrequency /* = DefaultSpringFrequency */, sfloat springDamping /* = DefaultSpringDamping */)
        {
            Assert.IsTrue(limitedAxis >= 0 && limitedAxis <= 2);
            distanceRange = distanceRange.Sorted();
            return new Constraint
            {
                ConstrainedAxes = new bool3(limitedAxis == 0, limitedAxis == 1, limitedAxis == 2),
                Type = ConstraintType.Linear,
                Min = distanceRange.Min,
                Max = distanceRange.Max,
                SpringFrequency = springFrequency,
                SpringDamping = springDamping
            };
        }

        #endregion

        #region Common angular constraints

        /// <summary>
        /// Constrains angular motion about all three axes to zero.
        /// </summary>
        /// <param name="springFrequency"></param>
        /// <param name="springDamping"></param>
        public static Constraint FixedAngle(sfloat springFrequency /* = DefaultSpringFrequency */, sfloat springDamping /* = DefaultSpringDamping */)
        {
            return new Constraint
            {
                ConstrainedAxes = new bool3(true),
                Type = ConstraintType.Angular,
                Min = sfloat.Zero,
                Max = sfloat.Zero,
                SpringFrequency = springFrequency,
                SpringDamping = springDamping
            };
        }

        /// <summary>
        /// Constrains angular motion about two axes to zero. Rotation around the third is unrestricted.
        /// </summary>
        /// <param name="freeAxis">The axis around which the bodies may freely rotate.</param>
        /// <param name="springFrequency"></param>
        /// <param name="springDamping"></param>
        public static Constraint Hinge(int freeAxis, sfloat springFrequency /* = DefaultSpringFrequency */, sfloat springDamping /* = DefaultSpringDamping */)
        {
            Assert.IsTrue(freeAxis >= 0 && freeAxis <= 2);
            return new Constraint
            {
                ConstrainedAxes = new bool3(freeAxis != 0, freeAxis != 1, freeAxis != 2),
                Type = ConstraintType.Angular,
                Min = sfloat.Zero,
                Max = sfloat.Zero,
                SpringFrequency = springFrequency,
                SpringDamping = springDamping
            };
        }

        /// <summary>
        /// Constrains angular motion about two axes within the specified range. Rotation around the third is unrestricted.
        /// </summary>
        /// <param name="freeAxis">The axis specifying the height of the cone within which the bodies may rotate.</param>
        /// <param name="angularRange">
        /// The minimum required angle and maximum possible angle between the free axis and its bind pose orientation.
        /// A minimum value of zero produces a conical range of motion, while a minimum value greater than zero results in motion restricted to the intersection of the inner and outer cones.
        /// </param>
        /// <param name="springFrequency"></param>
        /// <param name="springDamping"></param>
        public static Constraint Cone(int freeAxis, FloatRange angularRange, sfloat springFrequency /* = DefaultSpringFrequency */, sfloat springDamping /* = DefaultSpringDamping */)
        {
            Assert.IsTrue(freeAxis >= 0 && freeAxis <= 2);
            angularRange = angularRange.Sorted();
            return new Constraint
            {
                ConstrainedAxes = new bool3(freeAxis != 0, freeAxis != 1, freeAxis != 2),
                Type = ConstraintType.Angular,
                Min = angularRange.Min,
                Max = angularRange.Max,
                SpringFrequency = springFrequency,
                SpringDamping = springDamping
            };
        }

        /// <summary>
        /// Constrains angular motion about about one axis within the specified range.
        /// </summary>
        /// <param name="limitedAxis">The axis around which the bodies' rotation is restricted.</param>
        /// <param name="angularRange">The minimum required angle and maximum possible angle of rotation between the constrained bodies around the constrained axis.</param>
        /// <param name="springFrequency"></param>
        /// <param name="springDamping"></param>
        public static Constraint Twist(int limitedAxis, FloatRange angularRange, sfloat springFrequency /* = DefaultSpringFrequency */, sfloat springDamping /* = DefaultSpringDamping */)
        {
            Assert.IsTrue(limitedAxis >= 0 && limitedAxis <= 2);
            angularRange = angularRange.Sorted();
            return new Constraint
            {
                ConstrainedAxes = new bool3(limitedAxis == 0, limitedAxis == 1, limitedAxis == 2),
                Type = ConstraintType.Angular,
                Min = angularRange.Min,
                Max = angularRange.Max,
                SpringFrequency = springFrequency,
                SpringDamping = springDamping
            };
        }

        #endregion

        public bool Equals(Constraint other)
        {
            return ConstrainedAxes.Equals(other.ConstrainedAxes)
                && Type == other.Type
                && Min.Equals(other.Min)
                && Max.Equals(other.Max)
                && SpringFrequency.Equals(other.SpringFrequency)
                && SpringDamping.Equals(other.SpringDamping);
        }

        public override bool Equals(object obj) => obj is Constraint other && Equals(other);

        public override int GetHashCode()
        {
            unchecked
            {
                return (int)math.hash(new uint3x2(
                    new uint3((uint)Type, (uint)Min.GetHashCode(), (uint)Max.GetHashCode()),
                    new uint3(math.hash(ConstrainedAxes), (uint)SpringFrequency.GetHashCode(), (uint)SpringDamping.GetHashCode())
                ));
            }
        }
    }

    // A runtime joint instance, attached to specific rigid bodies
    public struct Joint
    {
        public BodyIndexPair BodyPair;
        public MTransform AFromJoint;
        public MTransform BFromJoint;
        // Note that Constraints needs to be 4-byte aligned for Android 32.
        public FixedList128<Constraint> Constraints;
        public byte EnableCollision; // If non-zero, allows these bodies to collide
        public byte Version;

        // The entity that contained the component which created this joint
        // Note, this isn't necessarily an entity with a rigid body, as a pair
        // of bodies can have an arbitrary number of constraints, but only one
        // instance of a particular component type per entity
        public Entity Entity;
    }
}
