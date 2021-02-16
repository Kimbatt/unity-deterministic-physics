using Unity.Collections.LowLevel.Unsafe;

namespace UnityS.Physics
{
    internal static class UnsafeEx
    {
        internal unsafe static int CalculateOffset<T, U>(ref T value, ref U baseValue)
            where T : struct
            where U : struct
        {
            return (int) ((byte*) UnsafeUtility.AddressOf(ref value) - (byte*) UnsafeUtility.AddressOf(ref baseValue));
        }
        
        internal unsafe static int CalculateOffset<T>(void* value, ref T baseValue)
            where T : struct
        {
            return (int) ((byte*) value - (byte*)UnsafeUtility.AddressOf(ref baseValue));
        }
    }
}
