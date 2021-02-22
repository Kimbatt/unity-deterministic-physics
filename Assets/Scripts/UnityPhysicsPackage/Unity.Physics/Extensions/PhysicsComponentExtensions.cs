using System;
using System.ComponentModel;
using UnityS.Mathematics;
using UnityS.Transforms;

namespace UnityS.Physics.Extensions
{
    /// <summary>
    /// Different ways to apply changes to a rigid body's velocity.
    /// </summary>
    public enum ForceMode
    {
        /// <summary>Apply a continuous force to the rigid body, using its mass.</summary>
        Force = 0,
        /// <summary>Apply a continuous acceleration to the rigid body, ignoring its mass.</summary>
        Impulse = 1,
        /// <summary>Apply an instant force impulse to the rigid body, using its mass.</summary>
        VelocityChange = 2,
        /// <summary>Apply an instant velocity change to the rigid body, ignoring its mass.</summary>
        Acceleration = 5
    }

    /// <summary>
    /// Utility functions acting on physics components.
    /// </summary>
    public static class PhysicsComponentExtensions
    {
        /// <summary>
        /// Get a body's effective mass in a given direction and from a particular point in world space.
        /// </summary>
        /// <param name="bodyMass">The body's <see cref="PhysicsMass"/> component.</param>
        /// <param name="bodyPosition">The body's <see cref="Translation"/> component.</param>
        /// <param name="bodyOrientation">The body's <see cref="Rotation"/> component.</param>
        /// <param name="impulse">An impulse in world space.</param>
        /// <param name="point">A point in world space.</param>
        /// <returns>A body's effective mass with respect to the specified point and impulse.</returns>
        public static sfloat GetEffectiveMass(in this PhysicsMass bodyMass, in Translation bodyPosition, in Rotation bodyOrientation, float3 impulse, float3 point) =>
            PhysicsWorldExtensions.GetEffectiveMassImpl(GetCenterOfMassWorldSpace(bodyMass, bodyPosition, bodyOrientation), bodyMass.InverseInertia, impulse, point);

        /// <summary>
        /// Get the center of mass in world space
        /// </summary>
        /// <param name="bodyMass">The body's <see cref="PhysicsMass"/> component</param>
        /// <param name="bodyPosition">The body's <see cref="Translation"/> component</param>
        /// <param name="bodyOrientation">The body's <see cref="Rotation"/> component</param>
        /// <returns>The center of mass in world space</returns>
        public static float3 GetCenterOfMassWorldSpace(in this PhysicsMass bodyMass, in Translation bodyPosition, in Rotation bodyOrientation) =>
            math.rotate(bodyOrientation.Value, bodyMass.CenterOfMass) + bodyPosition.Value;

        /// <summary>
        /// Set the center of mass in world space
        /// </summary>
        /// <param name="bodyMass">The body's <see cref="PhysicsMass"/> component</param>
        /// <param name="bodyPosition">The body's <see cref="Translation"/> component</param>
        /// <param name="bodyOrientation">The body's <see cref="Rotation"/> component</param>
        /// <param name="com">A position in world space for the new Center Of Mass</param>
        public static void SetCenterOfMassWorldSpace(ref this PhysicsMass bodyMass, in Translation bodyPosition, in Rotation bodyOrientation, float3 com)
        {
            com -= bodyPosition.Value;
            math.rotate(math.inverse(bodyOrientation.Value), com);
            bodyMass.CenterOfMass = com;
        }

        /// <summary>
        /// Get the linear velocity of a rigid body at a given point (in world space)
        /// </summary>
        /// <param name="bodyVelocity">The body's <see cref="PhysicsVelocity"/> component.</param>
        /// <param name="bodyMass">The body's <see cref="PhysicsMass"/> component</param>
        /// <param name="bodyPosition">The body's <see cref="Translation"/> component</param>
        /// <param name="bodyOrientation">The body's <see cref="Rotation"/> component</param>
        /// <param name="point">A reference position in world space</param>
        /// <returns>The linear velocity of a rigid body at a given point (in world space)</returns>
        public static float3 GetLinearVelocity(in this PhysicsVelocity bodyVelocity, PhysicsMass bodyMass, Translation bodyPosition, Rotation bodyOrientation, float3 point)
        {
            var worldFromEntity = new RigidTransform(bodyOrientation.Value, bodyPosition.Value);
            var worldFromMotion = math.mul(worldFromEntity, bodyMass.Transform);

            return PhysicsWorldExtensions.GetLinearVelocityImpl(worldFromMotion, bodyVelocity.Angular, bodyVelocity.Linear, point);
        }

        /// <summary>
        /// Get the world-space angular velocity of a rigid body.
        /// </summary>
        /// <param name="bodyVelocity">The body's <see cref="PhysicsVelocity"/> component.</param>
        /// <param name="bodyMass">The body's <see cref="PhysicsMass"/> component</param>
        /// <param name="bodyOrientation">The body's <see cref="Rotation"/> component</param>
        /// <returns>The angular velocity of a rigid body in world space</returns>
        public static float3 GetAngularVelocityWorldSpace(in this PhysicsVelocity bodyVelocity, in PhysicsMass bodyMass, in Rotation bodyOrientation)
        {
            quaternion worldFromMotion = math.mul(bodyOrientation.Value, bodyMass.InertiaOrientation);
            return math.rotate(worldFromMotion, bodyVelocity.Angular);
        }

        /// <summary>
        /// Set the world-space angular velocity of a rigid body.
        /// </summary>
        /// <param name="bodyVelocity">The body's <see cref="PhysicsVelocity"/> component.</param>
        /// <param name="bodyMass">The body's <see cref="PhysicsMass"/> component</param>
        /// <param name="bodyOrientation">The body's <see cref="Rotation"/> component</param>
        /// <param name="angularVelocity">An angular velocity in world space specifying radians per second about each axis.</param>
        public static void SetAngularVelocityWorldSpace(ref this PhysicsVelocity bodyVelocity, in PhysicsMass bodyMass, in Rotation bodyOrientation, in float3 angularVelocity)
        {
            quaternion inertiaOrientationInWorldSpace = math.mul(bodyOrientation.Value, bodyMass.InertiaOrientation);
            float3 angularVelocityInertiaSpace = math.rotate(math.inverse(inertiaOrientationInWorldSpace), angularVelocity);
            bodyVelocity.Angular = angularVelocityInertiaSpace;
        }

        /// <summary>
        /// Converts a force into an impulse based on the force mode and the bodies mass and inertia properties.
        /// </summary>
        /// <param name="bodyMass">The body's <see cref="PhysicsMass"/> component.</param>
        /// <param name="force">The force to be applied to a body.</param>
        /// <param name="mode">The method used to apply the force to its targets.</param>
        /// <param name="timestep">The change in time from the current to the next frame.</param>
        /// <param name="impulse">A returned impulse proportional to the provided 'force' and based on the supplied 'mode'.</param>
        /// <param name="impulseMass">A returned PhysicsMass component to be passed to an Apply function.</param>
        public static void GetImpulseFromForce(in this PhysicsMass bodyMass, in float3 force, in ForceMode mode, in sfloat timestep, out float3 impulse, out PhysicsMass impulseMass)
        {
            var unitMass = new PhysicsMass
            {
                InverseInertia = new float3(sfloat.One),
                InverseMass = sfloat.One,
                Transform = bodyMass.Transform
            };

            switch (mode)
            {
                case ForceMode.Force:
                    // Add a continuous force to the rigidbody, using its mass.
                    impulseMass = bodyMass;
                    impulse = force * timestep;
                    break;
                case ForceMode.Acceleration:
                    // Add a continuous acceleration to the rigidbody, ignoring its mass.
                    impulseMass = unitMass;
                    impulse = force * timestep;
                    break;
                case ForceMode.Impulse:
                    // Add an instant force impulse to the rigidbody, using its mass.
                    impulseMass = bodyMass;
                    impulse = force;
                    break;
                case ForceMode.VelocityChange:
                    // Add an instant velocity change to the rigidbody, ignoring its mass.
                    impulseMass = unitMass;
                    impulse = force;
                    break;
                default:
                    impulseMass = bodyMass;
                    impulse = float3.zero;
                    break;
            }
        }

        /// <summary>
        /// Converts a force into an impulse based on the force mode and the bodies mass and inertia properties.
        /// Equivalent to UnityEngine.Rigidbody.AddExplosionForce
        /// </summary>
        /// <param name="bodyVelocity">The body's <see cref="PhysicsVelocity"/> component.</param>
        /// <param name="bodyMass">The body's <see cref="PhysicsMass"/> component.</param>
        /// <param name="bodyCollider">The body's <see cref="PhysicsCollider"/> component.</param>
        /// <param name="bodyPosition">The body's <see cref="Translation"/> component.</param>
        /// <param name="bodyOrientation">The body's <see cref="Rotation"/> component.</param>
        /// <param name="explosionForce">The force of the explosion (which may be modified by distance).</param>
        /// <param name="explosionPosition">The centre of the sphere within which the explosion has its effect.</param>
        /// <param name="explosionRadius">The radius of the sphere within which the explosion has its effect.</param>
        /// <param name="timestep">The change in time from the current to the next frame.</param>
        /// <param name="up">A vector defining the up direction, generally a unit vector in the opposite direction to <see cref="PhysicsStep"/>.Gravity.</param>
        /// <param name="explosionFilter">Filter determining whether an explosion should be applied to the body.</param>
        /// <param name="upwardsModifier">Adjustment to the apparent position of the explosion to make it seem to lift objects.</param>
        /// <param name="mode">The method used to apply the force to its targets.</param>
        public static void ApplyExplosionForce(
            ref this PhysicsVelocity bodyVelocity, in PhysicsMass bodyMass, in PhysicsCollider bodyCollider,
            in Translation bodyPosition, in Rotation bodyOrientation,
            sfloat explosionForce, in float3 explosionPosition, in sfloat explosionRadius,
            in sfloat timestep, in float3 up, in CollisionFilter explosionFilter,
            in sfloat upwardsModifier /* = 0 */, ForceMode mode = ForceMode.Force)
        {
            var worldFromBody = new RigidTransform(bodyOrientation.Value, bodyPosition.Value);

            // The explosion is modelled as a sphere with a certain centre position and radius in world space;
            // normally, anything outside the sphere is not affected by the explosion and the force decreases
            // in proportion to distance from the centre.
            // However, if a value of zero is passed for the radius then the full force will be applied
            // regardless of how far the centre is from the rigidbody.
            bool bExplosionProportionalToDistance = !explosionRadius.IsZero();

            var pointDistanceInput = new PointDistanceInput()
            {
                Position = math.transform(math.inverse(worldFromBody), explosionPosition),
                MaxDistance = math.select(sfloat.MaxValue, explosionRadius, bExplosionProportionalToDistance),
                Filter = explosionFilter
            };

            // This function applies a force to the object at the point on the surface of the rigidbody
            // that is closest to explosionPosition. The force acts along the direction from explosionPosition
            // to the surface point on the rigidbody.
            // If explosionPosition is inside the rigidbody, or the rigidbody has no active colliders,
            // then the center of mass is used instead of the closest point on the surface.
            if (!bodyCollider.Value.Value.CalculateDistance(pointDistanceInput, out DistanceHit closestHit))
            {
                // Return now if the collider is out of range.
                return;
            }

            // The magnitude of the force depends on the distance between explosionPosition
            // and the point where the force was applied. As the distance between
            // explosionPosition and the force point increases, the actual applied force decreases.
            if (bExplosionProportionalToDistance)
            {
                var closestHitFraction = closestHit.Distance / pointDistanceInput.MaxDistance;
                explosionForce *= sfloat.One - closestHitFraction;
            }

            var closestHitPositionWorld = math.transform(worldFromBody, closestHit.Position);
            var forceDirection = math.normalizesafe(closestHitPositionWorld - explosionPosition);
            var force = explosionForce * forceDirection;

            // The vertical direction of the force can be modified using upwardsModifier.
            // If this parameter is greater than zero, the force is applied at the point
            // on the surface of the Rigidbody that is closest to explosionPosition but
            // shifted along the y-axis by negative upwardsModifier.Using this parameter,
            // you can make the explosion appear to throw objects up into the air,
            // which can give a more dramatic effect rather than a simple outward force.
            // Force can be applied only to an active rigidbody.
            if (!upwardsModifier.IsZero())
            {
                closestHitPositionWorld -= up * upwardsModifier;
            }

            bodyMass.GetImpulseFromForce(force, mode, timestep, out float3 impulse, out PhysicsMass impulseMass);
            bodyVelocity.ApplyImpulse(impulseMass, bodyPosition, bodyOrientation, impulse, closestHitPositionWorld);
        }

        /// <summary>
        /// Converts a force into an impulse based on the force mode and the bodies mass and inertia properties.
        /// Equivalent to UnityEngine.Rigidbody.AddExplosionForce.
        /// ExplosionFilter is set to CollisionFIlter.Default
        /// <see cref="ApplyExplosionForce(ref PhysicsVelocity, in PhysicsMass, in PhysicsCollider, in Translation, in Rotation, sfloat, in float3, in sfloat, in sfloat, in float3, in CollisionFilter, in sfloat, ForceMode)"></see>
        /// </summary>
        public static void ApplyExplosionForce(
            ref this PhysicsVelocity bodyVelocity, in PhysicsMass bodyMass, in PhysicsCollider bodyCollider,
            in Translation bodyPosition, in Rotation bodyOrientation,
            sfloat explosionForce, in float3 explosionPosition, in sfloat explosionRadius,
            in sfloat timestep, in float3 up,
            in sfloat upwardsModifier /* = 0 */, ForceMode mode = ForceMode.Force)
        {
            bodyVelocity.ApplyExplosionForce(bodyMass, bodyCollider, bodyPosition, bodyOrientation, explosionForce,
                explosionPosition, explosionRadius, timestep, up, CollisionFilter.Default, upwardsModifier, mode);
        }

        public static void ApplyImpulse(ref this PhysicsVelocity pv, in PhysicsMass pm, in Translation t, in Rotation r, in float3 impulse, in float3 point)
        {
            // Linear
            pv.ApplyLinearImpulse(pm, impulse);

            // Angular
            {
                // Calculate point impulse
                var worldFromEntity = new RigidTransform(r.Value, t.Value);
                var worldFromMotion = math.mul(worldFromEntity, pm.Transform);
                float3 angularImpulseWorldSpace = math.cross(point - worldFromMotion.pos, impulse);
                float3 angularImpulseInertiaSpace = math.rotate(math.inverse(worldFromMotion.rot), angularImpulseWorldSpace);

                pv.ApplyAngularImpulse(pm, angularImpulseInertiaSpace);
            }
        }

        public static void ApplyLinearImpulse(ref this PhysicsVelocity velocityData, in PhysicsMass massData, in float3 impulse)
        {
            velocityData.Linear += impulse * massData.InverseMass;
        }

        public static void ApplyAngularImpulse(ref this PhysicsVelocity velocityData, in PhysicsMass massData, in float3 impulse)
        {
            velocityData.Angular += impulse * massData.InverseInertia;
        }

        /// <summary>
        /// Compute a future position and orientation for a dynamic rigid body based on its current trajectory, after a specified amount of time.
        /// </summary>
        /// <param name="physicsVelocity">The body's <see cref="PhysicsVelocity"/> component.</param>
        /// <param name="physicsMass">The body's <see cref="PhysicsMass"/> component.</param>
        /// <param name="timestep">The change in time from the current to the next frame.</param>
        /// <param name="position">The future position of the body.</param>
        /// <param name="orientation">The future orientation of the body.</param>
        public static void Integrate(
            this in PhysicsVelocity physicsVelocity, in PhysicsMass physicsMass, sfloat timestep,
            ref float3 position, ref quaternion orientation)
        {
            var angularVelocityWS =
                physicsVelocity.GetAngularVelocityWorldSpace(physicsMass, new Rotation { Value = orientation });

            Integrator.IntegratePosition(ref position, physicsVelocity.Linear, timestep);
            Integrator.IntegrateOrientation(ref orientation, angularVelocityWS, timestep);
        }
    }
}
