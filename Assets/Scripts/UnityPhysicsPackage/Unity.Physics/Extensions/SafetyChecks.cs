using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using Unity.Collections;
using UnityS.Mathematics;

namespace UnityS.Physics
{
    static class SafetyChecks
    {
        public const string ConditionalSymbol = "ENABLE_UNITY_COLLECTIONS_CHECKS";

        [Conditional(ConditionalSymbol)]
        public static unsafe void Check4ByteAlignmentAndThrow(void* data, in FixedString32 paramName)
        {
            if (((long)data & 0x3) != 0)
                throw new InvalidOperationException($"{paramName} must be 4-byte aligned.");
        }

        [Conditional(ConditionalSymbol)]
        public static void CheckAreEqualAndThrow(SimulationType expected, SimulationType actual)
        {
            if (actual != expected)
                throw new ArgumentException($"Simulation type {actual} is not supported. This method should only be called when using {expected}.");
        }

        [Conditional(ConditionalSymbol)]
        public static void CheckFiniteAndThrow(float3 value, FixedString32 paramName)
        {
            if (math.any(!math.isfinite(value)))
                throw new ArgumentException($"{value} was not finite.", $"{paramName}");
        }

        [Conditional(ConditionalSymbol)]
        public static void CheckFiniteAndPositiveAndThrow(float3 value, in FixedString32 paramName)
        {
            if (math.any(value < sfloat.Zero) || math.any(!math.isfinite(value)))
                throw new ArgumentOutOfRangeException($"{paramName}", $"{value} is not positive and finite.");
        }

        [Conditional(ConditionalSymbol)]
        public static void CheckIndexAndThrow(int index, int length, int min = 0)
        {
            if (index < min || index >= length)
                throw new IndexOutOfRangeException($"Index {index} is out of range [{min}, {length}].");
        }

        [Conditional(ConditionalSymbol)]
        public static void CheckInRangeAndThrow(int value, int2 range, in FixedString32 paramName)
        {
            if (value < range.x || value > range.y)
                throw new ArgumentOutOfRangeException($"{paramName}", $"{value} is out of range [{range.x}, {range.y}].");
        }

        [Conditional(ConditionalSymbol)]
        public static void CheckNotEmptyAndThrow<T>(NativeArray<T> array, in FixedString32 paramName) where T : struct
        {
            if (!array.IsCreated || array.Length == 0)
                throw new ArgumentException("Array is empty.", $"{paramName}");
        }

#region Geometry Validation

        [Conditional(ConditionalSymbol)]
        public static void CheckCoplanarAndThrow(float3 vertex0, float3 vertex1, float3 vertex2, float3 vertex3, in FixedString32 paramName)
        {
            var normal = math.normalize(math.cross(vertex1 - vertex0, vertex2 - vertex0));
            if (math.abs(math.dot(normal, vertex3 - vertex0)) > sfloat.FromRaw(0x3a83126f))
                throw new ArgumentException("Vertices are not co-planar", $"{paramName}");
        }

        [Conditional(ConditionalSymbol)]
        public static void CheckTriangleIndicesInRangeAndThrow(NativeArray<int3> triangles, int numVertices, in FixedString32 paramName)
        {
            for (var i = 0; i < triangles.Length; ++i)
            {
                if (math.any(triangles[i] < 0) || math.any(triangles[i] >= numVertices))
                    throw new ArgumentException($"{paramName}", $"Triangle {triangles[i]} contained index out of range [0, {numVertices - 1}]");
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Geometry_CheckFiniteAndThrow(float3 value, in FixedString32 paramName, in FixedString32 propertyName)
        {
            if (math.any(!math.isfinite(value)))
                throw new ArgumentException($"{propertyName} {value} was not finite.", $"{paramName}");
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Geometry_CheckFiniteAndPositiveAndThrow(sfloat value, in FixedString32 paramName, in FixedString32 propertyName)
        {
            if (value < sfloat.Zero || !math.isfinite(value))
                throw new ArgumentException($"{propertyName} {value} is not positive.", $"{paramName}");
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Geometry_CheckFiniteAndPositiveAndThrow(float3 value, in FixedString32 paramName, in FixedString32 propertyName)
        {
            if (math.any(value < sfloat.Zero) || math.any(!math.isfinite(value)))
                throw new ArgumentException($"{paramName}", $"{propertyName} {value} is not positive.");
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Geometry_CheckValidAndThrow(quaternion q, in FixedString32 paramName, in FixedString32 propertyName)
        {
            if (q.Equals(default) || math.any(!math.isfinite(q.value)))
                throw new ArgumentException($"{propertyName} {q} is not valid.", $"{paramName}");
        }

        [Conditional(ConditionalSymbol)]
        public static void CheckValidAndThrow(NativeArray<float3> points, in FixedString32 pointsName, in ConvexHullGenerationParameters generationParameters, in FixedString32 paramName)
        {
            Geometry_CheckFiniteAndPositiveAndThrow(generationParameters.BevelRadius, paramName, nameof(ConvexHullGenerationParameters.BevelRadius));

            for (int i = 0, count = points.Length; i < count; ++i)
                CheckFiniteAndThrow(points[i], pointsName);
        }

        [Conditional(ConditionalSymbol)]
        public static void CheckValidAndThrow(in BoxGeometry geometry, in FixedString32 paramName)
        {
            Geometry_CheckFiniteAndThrow(geometry.Center, paramName, nameof(BoxGeometry.Center));
            Geometry_CheckValidAndThrow(geometry.Orientation, paramName, nameof(BoxGeometry.Orientation));
            Geometry_CheckFiniteAndPositiveAndThrow(geometry.Size, paramName, nameof(BoxGeometry.Size));
            Geometry_CheckFiniteAndPositiveAndThrow(geometry.BevelRadius, paramName, nameof(BoxGeometry.BevelRadius));
            if (geometry.BevelRadius < sfloat.Zero || geometry.BevelRadius > math.cmin(geometry.Size) * (sfloat)0.5f)
                throw new ArgumentException($"{paramName}", $"{nameof(BoxGeometry.BevelRadius)} must be greater than or equal to and);less than or equal to half the smallest size dimension.");
        }

        [Conditional(ConditionalSymbol)]
        public static void CheckValidAndThrow(in CapsuleGeometry geometry, in FixedString32 paramName)
        {
            Geometry_CheckFiniteAndThrow(geometry.Vertex0, paramName, nameof(CapsuleGeometry.Vertex0));
            Geometry_CheckFiniteAndThrow(geometry.Vertex1, paramName, nameof(CapsuleGeometry.Vertex1));
            Geometry_CheckFiniteAndPositiveAndThrow(geometry.Radius, paramName, nameof(CapsuleGeometry.Radius));
        }

        [Conditional(ConditionalSymbol)]
        public static void CheckValidAndThrow(in CylinderGeometry geometry, in FixedString32 paramName)
        {
            Geometry_CheckFiniteAndThrow(geometry.Center, paramName, nameof(CylinderGeometry.Center));
            Geometry_CheckValidAndThrow(geometry.Orientation, paramName, nameof(CylinderGeometry.Orientation));
            Geometry_CheckFiniteAndPositiveAndThrow(geometry.Height, paramName, nameof(CylinderGeometry.Height));
            Geometry_CheckFiniteAndPositiveAndThrow(geometry.Radius, paramName, nameof(CylinderGeometry.Radius));
            Geometry_CheckFiniteAndPositiveAndThrow(geometry.BevelRadius, paramName, nameof(CylinderGeometry.BevelRadius));
            if (geometry.BevelRadius < sfloat.Zero || geometry.BevelRadius > math.min(geometry.Height * (sfloat)0.5f, geometry.Radius))
                throw new ArgumentException($"{paramName}", $"{nameof(CylinderGeometry.BevelRadius)} must be greater than or equal to 0 and less than or equal to half the smallest size dimension.");
            if (geometry.SideCount < CylinderGeometry.MinSideCount || geometry.SideCount > CylinderGeometry.MaxSideCount)
                throw new ArgumentException($"{paramName}", $"{nameof(CylinderGeometry.SideCount)} must be greater than or equal to {CylinderGeometry.MinSideCount} and less than or equal to {CylinderGeometry.MaxSideCount}.");
        }

        [Conditional(ConditionalSymbol)]
        public static void CheckValidAndThrow(in SphereGeometry geometry, in FixedString32 paramName)
        {
            Geometry_CheckFiniteAndThrow(geometry.Center, paramName, nameof(SphereGeometry.Center));
            Geometry_CheckFiniteAndPositiveAndThrow(geometry.Radius, paramName, nameof(SphereGeometry.Radius));
        }

#endregion

#region Throw Exceptions

        [Conditional(ConditionalSymbol)]
        public static void ThrowInvalidOperationException(FixedString128 message = default) => throw new InvalidOperationException($"{message}");

        [Conditional(ConditionalSymbol)]
        public static void ThrowNotImplementedException() => throw new NotImplementedException();

        [Conditional(ConditionalSymbol)]
        public static void ThrowNotSupportedException(FixedString64 message = default) => throw new NotSupportedException($"{message}");

        [Conditional(ConditionalSymbol)]
        public static void ThrowArgumentException(in FixedString32 paramName, FixedString64 message = default) =>
            throw new ArgumentException($"{message}", $"{paramName}");

#endregion
    }
}
