#pragma once

namespace CepuPhysics
{
  struct TypedIndex
  {
    TypedIndex() = default;
    TypedIndex(int type, int index)
    {
      assert(type >= 0 && type < 128 && "Do you really have that many type indices, or is the index corrupt?");
      assert(index >= 0 && index < (1 << 24) && "Do you really have that many instances, or is the index corrupt?");
      //Note the inclusion of a set bit in the most significant slot.
      //This encodes that the index was explicitly constructed, so it is a 'real' reference.
      //A default constructed TypeIndex will have a 0 in the MSB, so we can use the default constructor for empty references.
      m_Packed = (uint32_t)((type << 24) | index | (1u << 31));
    }

    int32_t GetType () const { return (int32_t)(m_Packed & 0x7F000000) >> 24; }
    int32_t GetIndex() const { return (int32_t)(m_Packed & 0x00FFFFFF); }
    bool Exists() const { return (m_Packed & (1 << 31)) > 0; }

    uint32_t m_Packed = 0;
  };

}
