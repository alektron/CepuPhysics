#pragma once

namespace CepuPhysics
{
  class UntypedList
  {
  public:
    UntypedList(int32_t elementSizeInBytes, int32_t initialCapacityInElements, CepuUtil::BufferPool* pool);
    int32_t Allocate(int32_t elementSizeInBytes, int32_t minimumElementCount, CepuUtil::BufferPool* pool);

    bool Validate() const { return m_ElementSizeInBytes > 0; }

    template<typename T>
    T& GetFromBytes(int32_t byteIndex)
    {
      assert(Validate());
      return *(T*)(m_Buffer.m_Memory + byteIndex);
    }

    template<typename T>
    T& AllocateUnsafely()
    {
      assert(Validate());
      assert(sizeof(T) == m_ElementSizeInBytes);
      auto newSize = m_ByteCount + sizeof(T);
      //If we store only byte count, we'd have to divide to get the element index.
      //If we store only count, we would have to store per-type size somewhere since the PairCache constructor doesn't have an id->type mapping.
      //So we just store both. It's pretty cheap and simple.
      m_Count++;
      auto byteIndex = m_ByteCount;
      m_ByteCount = newSize;
      return GetFromBytes<T>(byteIndex);
    }

    template<typename T>
    int32_t Allocate(int32_t minimumElementCount, CepuUtil::BufferPool* pool)
    {
      return Allocate(sizeof(T), minimumElementCount, pool);
    }

    template<typename T>
    int32_t Add(T& data, int32_t minimumCount, CepuUtil::BufferPool* pool)
    {
      auto byteIndex = Allocate<T>(minimumCount, pool);
      GetFromBytes<T>(byteIndex) = data;
      return byteIndex;
    }

  public:
    CepuUtil::Buffer<uint8_t> m_Buffer;
    int32_t m_Count = 0;
    int32_t m_ByteCount = 0;
    int32_t m_ElementSizeInBytes = 0;
  };


}
