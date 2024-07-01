#pragma once
#include "Buffer.h"

namespace CepuUtil
{
  class BufferPool
  {
  public:
    template<typename T>  void TakeAtLeast(int count, Buffer<T>& o_buffer)
    {
      //@TODO (alektron) Actually implement a pool
      o_buffer = Buffer<T>(new T[count], count);
    }

    template<typename T>  void Take(int count, Buffer<T>& o_buffer)
    {
      //@TODO (alektron) Actually implement a pool
      o_buffer = Buffer<T>(new T[count], count);
    }

    template<typename T>  void Return(Buffer<T>& buffer)
    {
      //@TODO (alektron) Actually implement a pool
      delete[] buffer.m_Memory;
    }

    template<typename T> static int GetCapacityForCount(int count)
    {
      auto RoundUpToPowerOf2 = [](int32_t v)
      {
        v--;
        v |= v >> 1;
        v |= v >> 2;
        v |= v >> 4;
        v |= v >> 8;
        v |= v >> 16;
        v++;
        return v;
      };

      if (count == 0)
        count = 1;
      return (int32_t)RoundUpToPowerOf2(((uint32_t)(count * sizeof(T))) / sizeof(T));
    }

    template<typename T>  void ResizeToAtLeast(Buffer<T>& o_buffer, int32_t targetSize, int32_t copyCount)
    {
      assert(copyCount <= targetSize && copyCount <= o_buffer.GetLength() && "Can't copy more elements than exist in the source or target buffers.");
      targetSize = GetCapacityForCount<T>(targetSize);
      if (o_buffer.GetLength() != targetSize) { //Note that we don't check for allocated status- for buffers, a length of 0 is the same as being unallocated.
        Buffer<T> newBuffer;
        TakeAtLeast(targetSize, newBuffer);

        //Don't bother copying from or re-pooling empty buffers. They're uninitialized.
        if (o_buffer.GetLength() > 0) {
          o_buffer.CopyTo(0, newBuffer, 0, copyCount);
          Return(o_buffer);
        }
        else
          assert(copyCount == 0 && "Should not be trying to copy elements from an empty span.");

        o_buffer = newBuffer;
      }
    }
  };
}

