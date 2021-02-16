using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;

namespace UnityS.Physics
{
    // An event raised when a pair of bodies involving a trigger material have overlapped during solving.
    public struct TriggerEvent
    {
        internal TriggerEventData EventData;

        public Entity EntityB => EventData.Entities.EntityB;
        public Entity EntityA => EventData.Entities.EntityA;
        public int BodyIndexB => EventData.BodyIndices.BodyIndexB;
        public int BodyIndexA => EventData.BodyIndices.BodyIndexA;
        public ColliderKey ColliderKeyB => EventData.ColliderKeys.ColliderKeyB;
        public ColliderKey ColliderKeyA => EventData.ColliderKeys.ColliderKeyA;
    }

    // A stream of trigger events.
    // This is a value type, which means it can be used in Burst jobs (unlike IEnumerable<TriggerEvent>).
    public struct TriggerEvents /* : IEnumerable<TriggerEvent> */
    {
        //@TODO: Unity should have a Allow null safety restriction
        [NativeDisableContainerSafetyRestriction]
        private readonly NativeStream m_EventDataStream;

        internal TriggerEvents(NativeStream eventDataStream)
        {
            m_EventDataStream = eventDataStream;
        }

        public Enumerator GetEnumerator()
        {
            return new Enumerator(m_EventDataStream);
        }

        public struct Enumerator /* : IEnumerator<TriggerEvent> */
        {
            private NativeStream.Reader m_Reader;
            private int m_CurrentWorkItem;
            private readonly int m_NumWorkItems;

            public TriggerEvent Current { get; private set; }

            internal Enumerator(NativeStream stream)
            {
                m_Reader = stream.IsCreated ? stream.AsReader() : new NativeStream.Reader();
                m_CurrentWorkItem = 0;
                m_NumWorkItems = stream.IsCreated ? stream.ForEachCount : 0;
                Current = default;

                AdvanceReader();
            }

            public bool MoveNext()
            {
                if (m_Reader.RemainingItemCount > 0)
                {
                    var eventData = m_Reader.Read<TriggerEventData>();

                    Current = eventData.CreateTriggerEvent();

                    AdvanceReader();
                    return true;
                }
                return false;
            }

            private void AdvanceReader()
            {
                while (m_Reader.RemainingItemCount == 0 && m_CurrentWorkItem < m_NumWorkItems)
                {
                    m_Reader.BeginForEachIndex(m_CurrentWorkItem);
                    m_CurrentWorkItem++;
                }
            }
        }
    }

    // An event raised when a pair of bodies involving a trigger material have overlapped during solving.
    struct TriggerEventData
    {
        public BodyIndexPair BodyIndices;
        public ColliderKeyPair ColliderKeys;
        public EntityPair Entities;

        internal TriggerEvent CreateTriggerEvent()
        {
            return new TriggerEvent
            {
                EventData = this
            };
        }
    }
}
