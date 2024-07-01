#pragma once

namespace CepuPhysics
{
  class WorkerPairCache
  {
  public:
    struct PreallocationSizes
    {
      int32_t m_ElementCount;
      int32_t m_ElementSizeInBytes;
    };

    //@TODO (alektron) We do not really have this issue in C++ so we can probably replace this in the future. For now we will stick to the original
    //ORIGINAL COMMENT: note that this reference makes the entire worker pair cache nonblittable. That's why the pair cache uses managed arrays to store the worker caches.
    CepuUtil::BufferPool* m_Pool = nullptr;
    int32_t m_MinimumPerTypeCapactiy = 0;
    int32_t m_WorkerIndex = 0;

    //Note that the per-type batches are untyped.
    //The caller will have the necessary type knowledge to interpret the buffer.
    internal Buffer<UntypedList> constraintCaches;
    internal Buffer<UntypedList> collisionCaches;
  };
}