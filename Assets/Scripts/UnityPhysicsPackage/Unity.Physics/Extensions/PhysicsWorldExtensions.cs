using System.Runtime.CompilerServices;
using Unity.Entities;
using UnityS.Mathematics;

namespace UnityS.Physics.Extensions
{
    // Utility functions acting on a physics world
    public static class PhysicsWorldExtensions
    {
        public static CollisionFilter GetCollisionFilter(this in PhysicsWorld world, int rigidBodyIndex)
        {
            CollisionFilter filter = CollisionFilter.Default;
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumBodies)) return filter;

            unsafe { filter = world.Bodies[rigidBodyIndex].Collider.Value.Filter; }

            return filter;
        }

        public static sfloat GetMass(this in PhysicsWorld world, int rigidBodyIndex)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return sfloat.Zero;

            MotionVelocity mv = world.MotionVelocities[rigidBodyIndex];

            return mv.InverseMass.IsZero() ? sfloat.Zero : sfloat.One / mv.InverseMass;
        }

        // Get the effective mass of a Rigid Body in a given direction and from a particular point (in World Space)
        public static sfloat GetEffectiveMass(this in PhysicsWorld world, int rigidBodyIndex, float3 impulse, float3 point)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return sfloat.Zero;

            MotionVelocity mv = world.MotionVelocities[rigidBodyIndex];

            return GetEffectiveMassImpl(GetCenterOfMass(world, rigidBodyIndex), mv.InverseInertia, impulse, point);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static sfloat GetEffectiveMassImpl(float3 centerOfMass, float3 inverseInertia, float3 impulse, float3 point)
        {
            float3 pointDir = math.normalizesafe(point - centerOfMass);
            float3 impulseDir = math.normalizesafe(impulse);

            float3 jacobian = math.cross(pointDir, impulseDir);
            sfloat invEffMass = math.csum(math.dot(jacobian, jacobian) * inverseInertia);
            return math.select(sfloat.One / invEffMass, sfloat.Zero, math.abs(invEffMass) < sfloat.FromRaw(0x3727c5ac));
        }

        // Get the Rigid Bodies Center of Mass (in World Space)
        public static float3 GetCenterOfMass(this in PhysicsWorld world, int rigidBodyIndex)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return float3.zero;
            return world.MotionDatas[rigidBodyIndex].WorldFromMotion.pos;
        }

        public static float3 GetPosition(this in PhysicsWorld world, int rigidBodyIndex)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return float3.zero;

            // Motion to body transform
            MotionData md = world.MotionDatas[rigidBodyIndex];

            RigidTransform worldFromBody = math.mul(md.WorldFromMotion, math.inverse(md.BodyFromMotion));
            return worldFromBody.pos;
        }

        public static quaternion GetRotation(this in PhysicsWorld world, int rigidBodyIndex)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return quaternion.identity;

            // Motion to body transform
            MotionData md = world.MotionDatas[rigidBodyIndex];

            RigidTransform worldFromBody = math.mul(md.WorldFromMotion, math.inverse(md.BodyFromMotion));
            return worldFromBody.rot;
        }

        // Get the linear velocity of a rigid body (in world space)
        public static float3 GetLinearVelocity(this in PhysicsWorld world, int rigidBodyIndex)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return float3.zero;

            return world.MotionVelocities[rigidBodyIndex].LinearVelocity;
        }

        // Set the linear velocity of a rigid body (in world space)
        public static void SetLinearVelocity(this PhysicsWorld world, int rigidBodyIndex, float3 linearVelocity)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return;

            Unity.Collections.NativeArray<MotionVelocity> motionVelocities = world.MotionVelocities;
            MotionVelocity mv = motionVelocities[rigidBodyIndex];
            mv.LinearVelocity = linearVelocity;
            motionVelocities[rigidBodyIndex] = mv;
        }

        // Get the linear velocity of a rigid body at a given point (in world space)
        public static float3 GetLinearVelocity(this in PhysicsWorld world, int rigidBodyIndex, float3 point)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return float3.zero;

            MotionVelocity mv = world.MotionVelocities[rigidBodyIndex];
            MotionData md = world.MotionDatas[rigidBodyIndex];

            return GetLinearVelocityImpl(md.WorldFromMotion, mv.AngularVelocity, mv.LinearVelocity, point);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static float3 GetLinearVelocityImpl(RigidTransform worldFromMotion, float3 angularVelocity, float3 linearVelocity, float3 point)
        {
            angularVelocity = math.rotate(worldFromMotion, angularVelocity);
            return linearVelocity + math.cross(angularVelocity, point - worldFromMotion.pos);
        }

        // Get the angular velocity of a rigid body around it's center of mass (in world space)
        public static float3 GetAngularVelocity(this in PhysicsWorld world, int rigidBodyIndex)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return float3.zero;

            MotionVelocity mv = world.MotionVelocities[rigidBodyIndex];
            MotionData md = world.MotionDatas[rigidBodyIndex];

            return math.rotate(md.WorldFromMotion, mv.AngularVelocity);
        }

        // Set the angular velocity of a rigid body (in world space)
        public static void SetAngularVelocity(this PhysicsWorld world, int rigidBodyIndex, float3 angularVelocity)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return;

            MotionData md = world.MotionDatas[rigidBodyIndex];
            float3 angularVelocityMotionSpace = math.rotate(math.inverse(md.WorldFromMotion.rot), angularVelocity);

            Unity.Collections.NativeArray<MotionVelocity> motionVelocities = world.MotionVelocities;
            MotionVelocity mv = motionVelocities[rigidBodyIndex];
            mv.AngularVelocity = angularVelocityMotionSpace;
            motionVelocities[rigidBodyIndex] = mv;
        }

        // Get the linear and the angular velocity of a rigid body (in world space)
        public static (float3 linearVelocity, float3 angularVelocity) GetLinearAngularVelocity(this in PhysicsWorld world, int rigidBodyIndex)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return (float3.zero, float3.zero);

            MotionVelocity mv = world.MotionVelocities[rigidBodyIndex];
            MotionData md = world.MotionDatas[rigidBodyIndex];

            float3 linear = mv.LinearVelocity;
            float3 angular = math.rotate(md.WorldFromMotion, mv.AngularVelocity);
            return (linear, angular);
        }

        // Set the linear and the angular velocity of a rigid body (in world space)
        public static void SetLinearAngularVelocity(this PhysicsWorld world, int rigidBodyIndex, float3 linearVelocity, float3 angularVelocity)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return;

            MotionData md = world.MotionDatas[rigidBodyIndex];
            float3 angularVelocityMotionSpace = math.rotate(math.inverse(md.WorldFromMotion.rot), angularVelocity);

            Unity.Collections.NativeArray<MotionVelocity> motionVelocities = world.MotionVelocities;
            MotionVelocity mv = motionVelocities[rigidBodyIndex];
            mv.LinearVelocity = linearVelocity;
            mv.AngularVelocity = angularVelocityMotionSpace;
            motionVelocities[rigidBodyIndex] = mv;
        }

        // Apply an impulse to a rigid body at a point (in world space)
        public static void ApplyImpulse(this PhysicsWorld world, int rigidBodyIndex, float3 linearImpulse, float3 point)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return;

            MotionData md = world.MotionDatas[rigidBodyIndex];
            float3 angularImpulseWorldSpace = math.cross(point - md.WorldFromMotion.pos, linearImpulse);
            float3 angularImpulseMotionSpace = math.rotate(math.inverse(md.WorldFromMotion.rot), angularImpulseWorldSpace);

            Unity.Collections.NativeArray<MotionVelocity> motionVelocities = world.MotionVelocities;
            MotionVelocity mv = motionVelocities[rigidBodyIndex];
            mv.ApplyLinearImpulse(linearImpulse);
            mv.ApplyAngularImpulse(angularImpulseMotionSpace);
            motionVelocities[rigidBodyIndex] = mv;
        }

        // Apply a linear impulse to a rigid body (in world space)
        public static void ApplyLinearImpulse(this PhysicsWorld world, int rigidBodyIndex, float3 linearImpulse)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return;

            Unity.Collections.NativeArray<MotionVelocity> motionVelocities = world.MotionVelocities;
            MotionVelocity mv = motionVelocities[rigidBodyIndex];
            mv.ApplyLinearImpulse(linearImpulse);
            motionVelocities[rigidBodyIndex] = mv;
        }

        // Apply an angular impulse to a rigidBodyIndex (in world space)
        public static void ApplyAngularImpulse(this PhysicsWorld world, int rigidBodyIndex, float3 angularImpulse)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies)) return;

            MotionData md = world.MotionDatas[rigidBodyIndex];
            float3 angularImpulseInertiaSpace = math.rotate(math.inverse(md.WorldFromMotion.rot), angularImpulse);

            Unity.Collections.NativeArray<MotionVelocity> motionVelocities = world.MotionVelocities;
            MotionVelocity mv = motionVelocities[rigidBodyIndex];
            mv.ApplyAngularImpulse(angularImpulseInertiaSpace);
            motionVelocities[rigidBodyIndex] = mv;
        }

        // Calculate a linear and angular velocity required to move the given rigid body to the given target transform
        // in the given time step.
        public static void CalculateVelocityToTarget(
            this PhysicsWorld world, int rigidBodyIndex, RigidTransform targetTransform, sfloat timestep,
            out float3 requiredLinearVelocity, out float3 requiredAngularVelocity)
        {
            if (!(0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies))
            {
                requiredLinearVelocity = default;
                requiredAngularVelocity = default;
                return;
            }

            MotionData md = world.MotionDatas[rigidBodyIndex];
            RigidTransform worldFromBody = math.mul(md.WorldFromMotion, math.inverse(md.BodyFromMotion));
            CalculateVelocityToTargetImpl(
                worldFromBody, math.inverse(md.WorldFromMotion.rot), md.BodyFromMotion.pos, targetTransform, timestep,
                out requiredLinearVelocity, out requiredAngularVelocity
            );
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static void CalculateVelocityToTargetImpl(
            RigidTransform worldFromBody, quaternion motionFromWorld, float3 centerOfMass,
            RigidTransform targetTransform, in sfloat stepFrequency,
            out float3 requiredLinearVelocity, out float3 requiredAngularVelocity
        )
        {
            var com = new float4(centerOfMass, sfloat.One);
            requiredLinearVelocity = (math.mul(targetTransform, com) - math.mul(worldFromBody, com)).xyz * stepFrequency;
            var angularVelocity = math.mul(targetTransform.rot, math.inverse(worldFromBody.rot)).ToEulerAngles() * stepFrequency;
            requiredAngularVelocity = math.rotate(motionFromWorld, angularVelocity);
        }
    }
}
