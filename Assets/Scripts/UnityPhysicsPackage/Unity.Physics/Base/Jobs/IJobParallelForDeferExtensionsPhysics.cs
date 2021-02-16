using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

namespace Unity.Jobs
{
    internal static class IJobParallelForDeferExtensionsPhysics
    {
        unsafe public static JobHandle ScheduleUnsafeIndex0<T>(this T jobData, NativeArray<int> forEachCount, int innerloopBatchCount, JobHandle dependsOn = new JobHandle()) 
            where T : struct, IJobParallelForDefer
        {
            return IJobParallelForDeferExtensions.Schedule(jobData, (int*)NativeArrayUnsafeUtility.GetUnsafeBufferPointerWithoutChecks(forEachCount), innerloopBatchCount, dependsOn);
        }
    }
}