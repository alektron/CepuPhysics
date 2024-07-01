#pragma once

namespace CepuPhysics
{
  struct NodeChild
  {
    glm::vec3 Min;

    // (alektron) It seems that if Index is positive it is an index into the m_Nodes array.
    //            If it is negative it must be decoded (by calling Encode) and becomes an index into the leaves array.
    int32_t  Index = 0;
    glm::vec3 Max;
    int32_t  LeafCount = 0;
  };

  struct Node
  {
    NodeChild A;
    NodeChild B;
  };

  struct MetaNode
  {
    int32_t Parent;
    int32_t IndexInParent;
    int32_t RefineFlag;
    float   LocalCostChange;
  };

  struct Leaf
  {
    Leaf() = default;
    Leaf(int32_t nodeIndex, int32_t childIndex)
    {
      assert((childIndex & ~1) == 0 && "Binary trees can't have children in slots other than 0 and 1!");
      m_Packed = ((uint32_t)nodeIndex & 0x7FFFFFFF) | ((uint32_t)childIndex << 31);
    }
    int32_t GetNodeIndex () const { return (int32_t)( m_Packed & 0x7FFFFFFF); }
    int32_t GetChildIndex() const { return (int32_t)((m_Packed & 0x80000000) >> 31); }

    std::string to_string() const { return "hi"; }

    uint32_t m_Packed = 0;
  };
}
