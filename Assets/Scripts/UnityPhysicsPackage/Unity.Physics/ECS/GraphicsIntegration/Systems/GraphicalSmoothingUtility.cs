using System.Runtime.CompilerServices;
using Unity.Collections;
using UnityS.Mathematics;
using UnityS.Physics.Extensions;
using UnityS.Transforms;

namespace UnityS.Physics.GraphicsIntegration
{
    /// <summary>
    /// Utility functions for smoothing the motion of rigid bodies' graphical representations when physics steps at a lower frequency than rendering.
    /// </summary>
    public static class GraphicalSmoothingUtility
    {
        /// <summary>
        /// Compute a simple extrapolated transform for the graphical representation of a rigid body using its <c>currentTransform</c> and <c>currentVelocity</c>.
        /// Because bodies' motion may change the next time physics steps (e.g., as the result of a collision), using this method can mis-predict their future locations, causing them to appear to snap into place the next time physics steps.
        /// It generally results in a solid contact and kick from collisions, albeit with some interpenetration, as well as slight jitter on small, smooth velocity changes (e.g., the top of a parabolic arc).
        /// <code>
        /// Simulation:                Graphical Extrapolation:
        ///
        ///               O (t=2)                     O (t=2)
        /// (t=0) O      /               (t=0) O     o
        ///        \    /                       o   o
        ///         \  O (t=1)                   o O (t=1)
        /// _________\/_________       ___________o________
        ///                                        o
        /// </code>
        /// </summary>
        /// <param name="currentTransform">The transform of the rigid body after physics has stepped (i.e., the value of its <c>Translation</c> and <c>Rotation</c> components).</param>
        /// <param name="currentVelocity">The velocity of the rigid body after physics has stepped (i.e., the value of its <see cref="PhysicsVelocity"/> component).</param>
        /// <param name="mass">The body's <see cref="PhysicsMass"/> component.</param>
        /// <param name="timeAhead">A value indicating how many seconds the current elapsed time for graphics is ahead of the elapsed time when physics last stepped.</param>
        /// <returns>
        /// An extrapolated transform for a rigid body's graphical representation, suitable for constructing its <c>LocalToWorld</c> matrix before rendering.
        /// See also <seealso cref="BuildLocalToWorld"/>.
        /// </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static RigidTransform Extrapolate(
            in RigidTransform currentTransform, in PhysicsVelocity currentVelocity, in PhysicsMass mass, sfloat timeAhead
        )
        {
            var newTransform = currentTransform;
            currentVelocity.Integrate(mass, timeAhead, ref newTransform.pos, ref newTransform.rot);
            return newTransform;
        }

        /// <summary>
        /// Compute a simple interpolated transform for the graphical representation of a rigid body between <c>previousTransform</c> and <c>currentTransform</c>.
        /// Because bodies' motion is often deflected during a physics step when there is a contact event, using this method can make bodies appear to change direction before a collision is actually visible.
        /// <code>
        /// Simulation:                Graphical Interpolation:
        ///
        ///               O (t=2)                     O (t=2)
        /// (t=0) O      /               (t=0) O     o
        ///        \    /                        o  o
        ///         \  O (t=1)                     O (t=1)
        /// _________\/_________       ____________________
        ///
        /// </code>
        /// (Note that for cartoons, an animator would use squash and stretch to force a body to make contact even if it is technically not hitting on a specific frame.)
        /// See <see cref="InterpolateUsingVelocity"/> for an alternative approach.
        /// </summary>
        /// <param name="previousTransform">The transform of the rigid body before physics stepped.</param>
        /// <param name="currentTransform">The transform of the rigid body after physics has stepped (i.e., the value of its <c>Translation</c> and <c>Rotation</c> components).</param>
        /// <param name="normalizedTimeAhead">A value in the range [0, 1] indicating how many seconds the current elapsed time for graphics is ahead of the elapsed time when physics last stepped, as a proportion of the fixed timestep used by physics.</param>
        /// <returns>
        /// An interpolated transform for a rigid body's graphical representation, suitable for constructing its <c>LocalToWorld</c> matrix before rendering.
        /// See also <seealso cref="BuildLocalToWorld"/>.
        /// </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static RigidTransform Interpolate(
            in RigidTransform previousTransform,
            in RigidTransform currentTransform,
            sfloat normalizedTimeAhead
        )
        {
            return new RigidTransform(
                math.nlerp(previousTransform.rot, currentTransform.rot, normalizedTimeAhead),
                math.lerp(previousTransform.pos, currentTransform.pos, normalizedTimeAhead)
            );
        }

        /// <summary>
        /// Compute an interpolated transform for the graphical representation of a rigid body between its previous transform and current transform, integrating forward using an interpolated velocity value.
        /// This method tries to achieve a compromise between the visual artifacts of <see cref="Interpolate"/> and <see cref="Extrapolate"/>.
        /// While integrating forward using <c>previousVelocity</c> alone would exhibit behavior similar to <see cref="Extrapolate"/>, doing so in conjunction with <c>previousTransform</c> results in smoother motion for small velocity changes.
        /// Collisions can still appear premature when physics has a low tick rate, however, as when using <see cref="Interpolate"/>.
        /// This method is the default approach used by the <see cref="SmoothRigidBodiesGraphicalMotion"/> system for interpolated bodies.
        /// </summary>
        /// <param name="previousTransform">The transform of the rigid body before physics stepped.</param>
        /// <param name="previousVelocity">The velocity of the rigid body before physics stepped.</param>
        /// <param name="currentVelocity">The velocity of the rigid body after physics has stepped (i.e., the value of its <see cref="PhysicsVelocity"/> component).</param>
        /// <param name="mass">The body's <see cref="PhysicsMass"/> component.</param>
        /// <param name="timeAhead">A value indicating how many seconds the current elapsed time for graphics is ahead of the elapsed time when physics last stepped.</param>
        /// <param name="normalizedTimeAhead">A value in the range [0, 1] indicating how many seconds the current elapsed time for graphics is ahead of the elapsed time when physics last stepped, as a proportion of the fixed timestep used by physics.</param>
        /// <returns>
        /// An interpolated transform for a rigid body's graphical representation, suitable for constructing its <c>LocalToWorld</c> matrix before rendering.
        /// See also <seealso cref="BuildLocalToWorld"/>.
        /// </returns>
        public static RigidTransform InterpolateUsingVelocity(
            in RigidTransform previousTransform,
            in PhysicsVelocity previousVelocity,
            in PhysicsVelocity currentVelocity,
            in PhysicsMass mass,
            sfloat timeAhead, sfloat normalizedTimeAhead
        )
        {
            var newTransform = previousTransform;

            // Partially integrate with old velocities
            previousVelocity.Integrate(mass, timeAhead * (sfloat.One - normalizedTimeAhead), ref newTransform.pos, ref newTransform.rot);
            // Blend the previous and current velocities
            var interpolatedVelocity = new PhysicsVelocity
            {
                Linear = math.lerp(previousVelocity.Linear, currentVelocity.Linear, normalizedTimeAhead),
                Angular = math.lerp(previousVelocity.Angular, currentVelocity.Angular, normalizedTimeAhead)
            };
            // Then finish integration with blended velocities
            interpolatedVelocity.Integrate(mass, timeAhead * normalizedTimeAhead, ref newTransform.pos, ref newTransform.rot);

            return newTransform;
        }

        /// <summary>
        /// Construct a <c>LocalToWorld</c> matrix for a rigid body's graphical representation.
        /// </summary>
        /// <param name="i">The index of the rigid body in the chunk.</param>
        /// <param name="transform">The body's world space transform.</param>
        /// <param name="hasAnyScale"><c>true</c> if the rigid body has any scale component; otherwise, <c>false</c>.</param>
        /// <param name="hasNonUniformScale"><c>true</c> if the rigid body has a NonUniformScale component; otherwise, <c>false</c>.</param>
        /// <param name="nonUniformScales">The NonUniformScale values in the chunk, if any.</param>
        /// <param name="hasScale"><c>true</c> if the rigid body has a Scale component; otherwise, <c>false</c>.</param>
        /// <param name="scales">The Scale values in the chunk, if any.</param>
        /// <param name="compositeScales">The CompositeScale values in the chunk, if any.</param>
        /// <returns>A LocalToWorld matrix to use in place of those produced by default.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static LocalToWorld BuildLocalToWorld(
            int i, RigidTransform transform,
            bool hasAnyScale,
            bool hasNonUniformScale, NativeArray<NonUniformScale> nonUniformScales,
            bool hasScale, NativeArray<Scale> scales,
            NativeArray<CompositeScale> compositeScales
        )
        {
            var tr = new float4x4(transform);

            if (!hasAnyScale)
                return new LocalToWorld { Value = tr };

            var scale = hasNonUniformScale
                ? float4x4.Scale(nonUniformScales[i].Value)
                : hasScale
                    ? float4x4.Scale(new float3(scales[i].Value))
                    : compositeScales[i].Value;

            return new LocalToWorld { Value = math.mul(tr, scale) };
        }
    }
}
