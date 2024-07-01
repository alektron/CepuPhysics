#pragma once

namespace CepuUtil
{
#define VALIDATE_REGION(start, count) ValidateRegion(start, count);

  template<typename T>
  class Buffer
  {
  public:
    Buffer() = default;
    Buffer(T* memory, int length, int id = -1)
      : m_Memory(memory), m_Length(length), m_Id(id) {}

    int  GetLength  () const { return m_Length; }
    bool IsAllocated() const { return m_Memory != nullptr; }
    void Clear(int start, int count) { memset(m_Memory + start, 0, count * sizeof(T)); }

    void CopyTo(int32_t sourceStart, Buffer<T>& target, int32_t targetStart, int32_t count) 
    {
      assert(targetStart >= 0 && targetStart + count <= target .GetLength() && "Can't perform a copy that extends beyond the target.");
      assert(sourceStart >= 0 && sourceStart + count <= (*this).GetLength() && "Can't perform a copy that extends beyond the source.");
      memcpy(target.m_Memory + targetStart, m_Memory + sourceStart, sizeof(T) * count);
    }

    T& operator[](int index) const 
    { 
      return m_Memory[index];
    }

    template<typename TCast>
    Buffer<TCast> As()
    {
      auto count = m_Length * (int)sizeof(T) / (int)sizeof(TCast);
      return Buffer<TCast>((TCast*)m_Memory, count, m_Id);
    }

    T* m_Memory  = nullptr;
    int m_Id     = -1;
  private:
    void ValidateRegion(int start, int count)
    {
      assert(start >= 0 && "The start of a region must be within the buffer's extent.");
      assert(start + count <= m_Length && "The end of a region must be within the buffer's extent.");
    }

    int m_Length = 0;
  };
}
