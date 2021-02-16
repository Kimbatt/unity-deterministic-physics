using Unity.Entities;
using UnityS.Mathematics;
using UnityS.Transforms;

namespace UnityS.Physics.GraphicsIntegration
{
    /// <summary>
    /// A component to indicate that the graphical representation of a dynamic rigid body's motion should be smoothed when the rendering framerate is greater than the fixed step rate used by physics.
    /// When used on its own, it indicates that smoothing should extrapolate into the future based on the body's current velocity.
    /// The result is thus up-to-date, but can mis-predict the body's transformations since any future collision response has not yet been resolved.
    /// Note that when used, the values of the body's LocalToWorld matrix are modified, and may differ from those of its Translation and Rotation components.
    /// See also <seealso cref="PhysicsGraphicalInterpolationBuffer"/>.
    /// </summary>
    [WriteGroup(typeof(LocalToWorld))]
    public struct PhysicsGraphicalSmoothing : IComponentData
    {
        /// <summary>
        /// The body's linear and angular velocity from the most recent physics tick.
        /// </summary>
        public PhysicsVelocity CurrentVelocity;
        /// <summary>
        /// If non-zero, apply smoothing.
        /// Set this value to 0 when teleporting a body to prevent smoothing the motion of its graphics representation.
        /// <see cref="SmoothRigidBodiesGraphicalMotion"/> will reset this value to 1 each frame.
        /// </summary>
        public byte ApplySmoothing;
    }

    /// <summary>
    /// Stores the state of a rigid body from the previous physics tick in order to interpolate the motion of the body's graphical representation.
    /// When used in conjunction with <see cref="PhysicsGraphicalSmoothing"/>, it indicates that smoothing should interpolate between the two most recent physics simulation ticks.
    /// The result is thus a more accurate representation of the physics simulation, but is one tick behind.
    /// </summary>
    public struct PhysicsGraphicalInterpolationBuffer : IComponentData
    {
        /// <summary>
        /// The body's position and orientation from the previous physics tick.
        /// </summary>
        public RigidTransform PreviousTransform;
        /// <summary>
        /// The body's linear and angular velocity from the previous physics tick.
        /// </summary>
        public PhysicsVelocity PreviousVelocity;
    }
}
