#pragma once
#include "Memory/IdPool.h"
#include "BodyProperties.h"

namespace CepuPhysics
{
  struct RigidPose;
  class Shapes;

  class ShapeBatch
  {
  public:
    ShapeBatch(CepuUtil::BufferPool* pool, int32_t initialShapeCount)
      : m_IdPool(initialShapeCount, pool), m_Pool(pool) {}
    virtual ~ShapeBatch() {};
    void Remove(int32_t index);
    void RemoveAndDsiapose(int32_t index, CepuUtil::BufferPool* pool);
    void RecursivelyRemoveAndDispose(int32_t index, Shapes* shapes, CepuUtil::BufferPool* pool);

    //virtual void ComputeBounds(const BoundingBoxBatcher& batcher) = 0;
    virtual void ComputeBounds(int32_t shapeIndex, const RigidPose& pose, glm::vec3& o_min, glm::vec3& o_max) = 0;
    virtual void ComputeBounds(int32_t shapeIndex, const glm::quat& orientation, float& o_maximumRadius, float& o_maximumAngularExpansion, glm::vec3& o_min, glm::vec3& o_max)
    {
      throw "Nonconvex shapes are not required to have a maximum radius or angular expansion implementation. This should only ever be called on convexes.";
    }

    //template<typename TRayHitHandler>
    //struct RayTester
    //{
    //  virtual void RayTest(int32_t shapeIndex, in RigidPose pose, in RayData ray, ref float maximumT, ref TRayHitHandler hitHandler);
    //  virtual void RayTest(int32_t shapeIndex, in RigidPose rigidPose, ref RaySource rays, ref TRayHitHandler hitHandler);
    //};

    void GetShapeData(int32_t shapeIndex, void** shapePointer, int32_t& o_shapeSize);
    virtual void Clear() = 0;
    virtual void EnsureCapacity(int32_t shapeCapacity) = 0;
    virtual void Resize(int32_t shapeCapacity) = 0;

    void ResizeIdPool(int32_t targetIdCapacity) { m_IdPool.Resize(targetIdCapacity, m_Pool); }

    int32_t GetCapacity     () { return m_ShapesData.GetLength() / m_ShapeDataSize; }
    int32_t GetShapeDataSize() { return m_ShapeDataSize; }
    int32_t GetTypeId       () { return m_TypeId; }
    bool    IsCompound      () { return m_Compound; }

  protected:
    virtual void Dispose(int32_t index, CepuUtil::BufferPool* pool) = 0;
    virtual void RemoveAndDisposeChildren(int32_t index, Shapes* shapes, CepuUtil::BufferPool* pool) = 0;

    CepuUtil::Buffer<uint8_t> m_ShapesData;
    int32_t m_ShapeDataSize = 0;

    CepuUtil::BufferPool* m_Pool;
    CepuUtil::IdPool m_IdPool;

    int32_t m_TypeId = 0;
    bool m_Compound = false;

  };

  template<typename TShape>
  class ShapeBatchT : public ShapeBatch
  {
  public:
    ShapeBatchT(CepuUtil::BufferPool* pool, int32_t initialShapeCount)
      : ShapeBatch(pool, initialShapeCount)
    {
      m_TypeId = TShape().GetTypeId();
      InternalResize(initialShapeCount, 0);
    }

    virtual ~ShapeBatchT() override
    {
      assert(m_ShapesData.m_Id == m_Shapes.m_Id && "If the buffer ids donÄt match, there was some form af failed resize.");
      m_Pool->Return(m_ShapesData);
      m_IdPool.Dispose(m_Pool);
    }

    int32_t Add(const TShape& shape)
    {
      auto shapeIndex = m_IdPool.Take();
      if (m_Shapes.GetLength() <= shapeIndex)
        InternalResize(shapeIndex + 1, m_Shapes.GetLength());
      new (&m_Shapes[shapeIndex]) TShape(shape);
      return shapeIndex;
    }

    void InternalResize(int32_t shapeCount, int32_t oldCopyLength)
    {
      m_ShapeDataSize = sizeof(TShape);
      auto requiredSizeInBytes = shapeCount * (int)sizeof(TShape);
      CepuUtil::Buffer<uint8_t> newShapesData;
      m_Pool->TakeAtLeast<uint8_t>(requiredSizeInBytes, newShapesData);
      CepuUtil::Buffer<TShape> newShapes = newShapesData.As<TShape>();
#ifdef _DEBUG
      //In debug mode, unused slots are kept at the default value. THis helps cathc misues.
      if (newShapes.GetLength() > m_Shapes.GetLength())
        newShapes.Clear(m_Shapes.GetLength(), newShapes.GetLength() - m_Shapes.GetLength());
#endif
      if (m_ShapesData.IsAllocated()) {
        m_Shapes.CopyTo(0, newShapes, 0, oldCopyLength);
        m_Pool->Return(m_ShapesData);
      }
      else {
        assert(oldCopyLength == 0);
      }
      m_Shapes = newShapes;
      m_ShapesData = newShapesData;
    }

    virtual void Clear() override
    {
#ifdef _DEBUG
      m_Shapes.Clear(0, m_IdPool.GetHighestPossiblyClaimedId() + 1);
#endif
      m_IdPool.Clear();
    }

    virtual void EnsureCapacity(int32_t shapeCapacity) override
    {
      if (m_Shapes.GetLength() < shapeCapacity)
        InternalResize(shapeCapacity, m_IdPool.GetHighestPossiblyClaimedId() + 1);
    }

    virtual void Resize(int32_t shapeCapacity) override
    {
      shapeCapacity = CepuUtil::BufferPool::GetCapacityForCount<TShape>(glm::max(m_IdPool.GetHighestPossiblyClaimedId() + 1, shapeCapacity));
      if (shapeCapacity != m_Shapes.GetLength())
        InternalResize(shapeCapacity, m_IdPool.GetHighestPossiblyClaimedId() + 1);
    }

    TShape& operator[](int32_t shapeIndex) const { return m_Shapes[shapeIndex]; }

    CepuUtil::Buffer<TShape> m_Shapes;
  };

  template<typename TShape>
  class ConvexShapeBatch : public ShapeBatchT<TShape>
  {
  public:
    ConvexShapeBatch(CepuUtil::BufferPool* pool, int32_t initialShapeCount) : ShapeBatchT<TShape>(pool, initialShapeCount) {};
    virtual void Dispose(int32_t index, CepuUtil::BufferPool* pool) override { /*Most convex shapes with an associated Wide type doesn't have any internal resources to dispose.*/ };
    virtual void RemoveAndDisposeChildren(int32_t index, Shapes* shapes, CepuUtil::BufferPool* pool) override { /*And they don't have any children*/ };

    //virtual void ComputeBounds(const BoundingBoxBatcher& batcher) override;
    virtual void ComputeBounds(int32_t shapeIndex, const RigidPose& pose, glm::vec3& o_min, glm::vec3& o_max) override
    {
      this->m_Shapes[shapeIndex].ComputeBounds(pose.m_Orientation, o_min, o_max);
      o_min += pose.m_Position;
      o_max += pose.m_Position;
    }

    virtual void ComputeBounds(int32_t shapeIndex, const glm::quat& orientation, float& o_maximumRadius, float& o_angularExpansion, glm::vec3& o_min, glm::vec3& o_max) override
    {
      auto& shape = this->m_Shapes[shapeIndex];
      shape.ComputeBounds(orientation, o_min, o_max);
      shape.ComputeAngularExpansionData(o_maximumRadius, o_angularExpansion);
    }
  };

  class Shapes
  {
  public:
    Shapes(CepuUtil::BufferPool* pool, int32_t initialCapacityPerTypeBatch)
      : m_InitialCapacityPerTypeBatch(initialCapacityPerTypeBatch), m_Pool(pool) 
    {
      memset(m_Batches, 0, sizeof(ShapeBatch*) * MAX_SHAPE_BATCHES);
    }

    //@DEVIATION (alektron) Original name: UpdateBounds
    //I find the name a bit misleading since it implies that we're updating internal state which is not the case.
    //Since it only calls ComputeBounds functions anyways we're going with that for consistency
    void ComputeBounds(const RigidPose& pose, TypedIndex shapeIndex, CepuUtil::BoundingBox& o_bounds) const;

    template<typename TShape>
    TShape& GetShape(int32_t shapeIndex)
    {
      auto typeId = TShape().GetTypeId();
      ShapeBatchT<TShape>* typedBatch = ((ShapeBatchT<TShape>*)m_Batches[typeId]);
      return (*typedBatch)[shapeIndex];
    }

    template<typename TShape>
    TypedIndex Add(const TShape& shape)
    {
      auto typeId = TShape().GetTypeId();
      //@ (alektron) Bepu resizes the array but we are using a fixed size for now (will we really every have more than 16 shape types? (Apparently this might matter for constraints))
      if (typeId >= MAX_SHAPE_BATCHES)
        throw;

      if (m_Batches[typeId] == nullptr)
        m_Batches[typeId] = TShape().CreateShapeBatch(m_Pool, m_InitialCapacityPerTypeBatch);

      assert(dynamic_cast<ShapeBatchT<TShape>*>(m_Batches[typeId]));
      auto index = ((ShapeBatchT<TShape>*)m_Batches[typeId])->Add(shape);
      return TypedIndex(typeId, index);
    }

    static const int MAX_SHAPE_BATCHES = 16;
  private:
    ShapeBatch* m_Batches[MAX_SHAPE_BATCHES]{ nullptr };
    int32_t m_InitialCapacityPerTypeBatch = 0;

    CepuUtil::BufferPool* m_Pool = nullptr;
  };

}
