using System;
using Unity.Burst;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine.Assertions;

namespace Unity.Collections
{
    interface IPoolElement
    {
        bool IsAllocated { get; }
        void MarkFree(int nextFree);
        int NextFree { get; }
    }

    // Underlying implementation of ElementPool
    // This is split into a different structure so that it can be unmanaged (since templated structures are always managed)
    [NoAlias]
    unsafe struct ElementPoolBase
    {
        [NativeDisableContainerSafetyRestriction]
        [NoAlias]
        private void* m_Elements;    // storage for all elements (allocated and free)
        private readonly int m_capacity;      // number of elements
        private int m_FirstFreeIndex;         // the index of the first free element (or -1 if none free)

        public int Capacity => m_capacity;     // the maximum number of elements that can be allocated
        public int PeakCount { get; private set; }  // the maximum number of elements allocated so far
        public bool CanAllocate => m_FirstFreeIndex >= 0 || PeakCount < Capacity;

        public unsafe ElementPoolBase(void* userBuffer, int capacity)
        {
            m_Elements = userBuffer;
            m_capacity = capacity;
            m_FirstFreeIndex = -1;
            PeakCount = 0;
        }

        // Add an element to the pool
        public int Allocate<T>(T element) where T : unmanaged, IPoolElement
        {
            T* elements = ((T*)m_Elements);

            Assert.IsTrue(element.IsAllocated);
            if (m_FirstFreeIndex != -1)
            {
                int index = m_FirstFreeIndex;
                T* freeElement = (T*)m_Elements + index;
                m_FirstFreeIndex = freeElement->NextFree;
                *freeElement = element;
                return index;
            }

            Assert.IsTrue(PeakCount < Capacity);
            elements[PeakCount++] = element;
            return PeakCount - 1;
        }

        // Remove an element from the pool
        public void Release<T>(int index) where T : unmanaged, IPoolElement
        {
            T* elementsTyped = (T*)m_Elements;
            elementsTyped[index].MarkFree(m_FirstFreeIndex);
            m_FirstFreeIndex = index;
        }

        // Empty the pool
        public void Clear()
        {
            PeakCount = 0;
            m_FirstFreeIndex = -1;
        }

        public bool IsAllocated<T>(int index) where T : unmanaged, IPoolElement
        {
            T element = ((T*)m_Elements)[index];
            return element.IsAllocated;
        }

        public T Get<T>(int index) where T : unmanaged, IPoolElement
        {
            Assert.IsTrue(index < Capacity);
            T element = ((T*)m_Elements)[index];
            Assert.IsTrue(element.IsAllocated);
            return element;
        }

        public void Set<T>(int index, T value) where T : unmanaged, IPoolElement
        {
            Assert.IsTrue(index < Capacity);
            ((T*)m_Elements)[index] = value;
        }

        public unsafe void CopyFrom<T>(ElementPoolBase other) where T : unmanaged, IPoolElement
        {
            Assert.IsTrue(other.PeakCount <= Capacity);
            PeakCount = other.PeakCount;
            m_FirstFreeIndex = other.m_FirstFreeIndex;
            UnsafeUtility.MemCpy(m_Elements, other.m_Elements, PeakCount * UnsafeUtility.SizeOf<T>());
        }

        public unsafe void CopyFrom<T>(void* buffer, int length) where T : unmanaged, IPoolElement
        {
            Assert.IsTrue(length <= Capacity);
            PeakCount = length;
            m_FirstFreeIndex = -1;
            UnsafeUtility.MemCpy(m_Elements, buffer, PeakCount * UnsafeUtility.SizeOf<T>());
        }

        // Compacts the pool so that all of the allocated elements are contiguous, and resets PeakCount to the current allocated count.
        // remap may be null or an array of size at least PeakCount, if not null and the return value is true then Compact() sets remap[oldIndex] = newIndex for all allocated elements.
        // Returns true if compact occurred, false if the pool was already compact.
        public unsafe bool Compact<T>(int* remap) where T : unmanaged, IPoolElement
        {
            if (m_FirstFreeIndex == -1)
            {
                return false;
            }
            int numElements = 0;
            for (int i = 0; i < PeakCount; i++)
            {
                T element = ((T*)m_Elements)[i];
                if (element.IsAllocated)
                {
                    if (remap != null) remap[i] = numElements;
                    ((T*)m_Elements)[numElements++] = element;
                }
            }
            PeakCount = numElements;
            m_FirstFreeIndex = -1;
            return true;
        }

        #region Enumerables

        public IndexEnumerable<T> GetIndices<T>() where T : unmanaged, IPoolElement
        {
            NativeArray<T> slice = NativeArrayUnsafeUtility.ConvertExistingDataToNativeArray<T>(m_Elements, PeakCount, Allocator.None);
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            NativeArrayUnsafeUtility.SetAtomicSafetyHandle(ref slice, AtomicSafetyHandle.GetTempUnsafePtrSliceHandle());
#endif
            return new IndexEnumerable<T> { Slice = slice };
        }

        public ElementEnumerable<T> GetElements<T>() where T : unmanaged, IPoolElement
        {
            NativeArray<T> slice = NativeArrayUnsafeUtility.ConvertExistingDataToNativeArray<T>(m_Elements, PeakCount, Allocator.None);
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            NativeArrayUnsafeUtility.SetAtomicSafetyHandle(ref slice, AtomicSafetyHandle.GetTempUnsafePtrSliceHandle());
#endif
            return new ElementEnumerable<T> { Slice = slice };
        }

        public struct IndexEnumerable<T> where T : unmanaged, IPoolElement
        {
            internal NativeArray<T> Slice;

            public IndexEnumerator<T> GetEnumerator() => new IndexEnumerator<T>(ref Slice);
        }

        public struct ElementEnumerable<T> where T : unmanaged, IPoolElement
        {
            internal NativeArray<T> Slice;

            public ElementEnumerator<T> GetEnumerator() => new ElementEnumerator<T>(ref Slice);
        }

        // An enumerator for iterating over the indices
        public struct IndexEnumerator<T> where T : unmanaged, IPoolElement
        {
            internal NativeArray<T> Slice;
            internal int Index;

            public int Current => Index;

            internal IndexEnumerator(ref NativeArray<T> slice)
            {
                Slice = slice;
                Index = -1;
            }

            public bool MoveNext()
            {
                while (true)
                {
                    if (++Index >= Slice.Length)
                    {
                        return false;
                    }
                    if (Slice[Index].IsAllocated)
                    {
                        return true;
                    }
                }
            }
        }

        // An enumerator for iterating over the allocated elements
        public struct ElementEnumerator<T> where T : unmanaged, IPoolElement
        {
            internal NativeArray<T> Slice;
            internal IndexEnumerator<T> IndexEnumerator;

            public T Current => Slice[IndexEnumerator.Current];

            internal ElementEnumerator(ref NativeArray<T> slice)
            {
                Slice = slice;
                IndexEnumerator = new IndexEnumerator<T>(ref slice);
            }

            public bool MoveNext() => IndexEnumerator.MoveNext();
        }

        #endregion
    }

    // A fixed capacity array acting as a pool of allocated/free structs referenced by indices
    unsafe struct ElementPool<T> where T : unmanaged, IPoolElement
    {
        public ElementPoolBase* ElementPoolBase;

        public int Capacity => ElementPoolBase->Capacity;     // the maximum number of elements that can be allocated
        public int PeakCount => ElementPoolBase->PeakCount;  // the maximum number of elements allocated so far
        public bool CanAllocate => ElementPoolBase->CanAllocate;

        // Add an element to the pool
        public int Allocate(T element) { return ElementPoolBase->Allocate<T>(element); }

        // Remove an element from the pool
        public void Release(int index) { ElementPoolBase->Release<T>(index); }

        // Empty the pool
        public void Clear() { ElementPoolBase->Clear(); }

        public bool IsAllocated(int index)
        {
            return ElementPoolBase->IsAllocated<T>(index);
        }

        // Get/set an element
        public T this[int index]
        {
            get { return ElementPoolBase->Get<T>(index); }
            set { ElementPoolBase->Set<T>(index, value); }
        }

        public void Set(int index, T value) { ElementPoolBase->Set<T>(index, value); }

        public unsafe void CopyFrom(ElementPool<T> other) { ElementPoolBase->CopyFrom<T>(*other.ElementPoolBase); }

        public unsafe void CopyFrom(void* buffer, int length) { ElementPoolBase->CopyFrom<T>(buffer, length); }

        public unsafe bool Compact(int* remap) { return ElementPoolBase->Compact<T>(remap); }

        public ElementPoolBase.IndexEnumerable<T> Indices => ElementPoolBase->GetIndices<T>();
        public ElementPoolBase.ElementEnumerable<T> Elements => ElementPoolBase->GetElements<T>();
    }
}
