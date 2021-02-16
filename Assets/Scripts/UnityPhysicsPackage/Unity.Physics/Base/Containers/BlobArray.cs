using Unity.Collections.LowLevel.Unsafe;

namespace UnityS.Physics
{
    // Non-generic temporary stand-in for Unity BlobArray.
    // This is to work around C# wanting to treat any struct containing the generic Unity.BlobArray<T> as a managed struct.
    // TODO: Use Unity.Blobs instead
    public struct BlobArray
    {
        internal int Offset;
        internal int Length;    // number of T, not number of bytes

        // Generic accessor
        public unsafe struct Accessor<T> where T : struct
        {
            private readonly int* m_OffsetPtr;
            public int Length { get; private set; }

            public Accessor(ref BlobArray blobArray)
            {
                fixed (BlobArray* ptr = &blobArray)
                {
                    m_OffsetPtr = &ptr->Offset;
                    Length = ptr->Length;
                }
            }

            public ref T this[int index]
            {
                get
                {
                    SafetyChecks.CheckIndexAndThrow(index, Length);
                    return ref UnsafeUtility.ArrayElementAsRef<T>((byte*)m_OffsetPtr + *m_OffsetPtr, index);
                }
            }

            public Enumerator GetEnumerator() => new Enumerator(m_OffsetPtr, Length);

            public struct Enumerator
            {
                private readonly int* m_OffsetPtr;
                private readonly int m_Length;
                private int m_Index;

                public T Current => UnsafeUtility.ArrayElementAsRef<T>((byte*)m_OffsetPtr + *m_OffsetPtr, m_Index);

                public Enumerator(int* offsetPtr, int length)
                {
                    m_OffsetPtr = offsetPtr;
                    m_Length = length;
                    m_Index = -1;
                }

                public bool MoveNext()
                {
                    return ++m_Index < m_Length;
                }
            }
        }
    }
}
