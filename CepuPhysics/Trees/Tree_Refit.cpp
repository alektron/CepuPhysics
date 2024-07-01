#include "CepuPhysicsPCH.h"
#include "Tree.h"
#include "Memory/BufferPool.h"
#include "BoundingBox.h"

using namespace CepuUtil;

namespace CepuPhysics
{
  void Tree::RefitForNodeBoundsChange(int32_t nodeIndex)
  {
    //Note that no attempt is made to refit the root node. Note that the root node is the only node that can have a number of children less than 2.
    auto node     = m_Nodes    .m_Memory + nodeIndex;
    auto metanode = m_Metanodes.m_Memory + nodeIndex;
    while (metanode->Parent >= 0)
    {
      //Compute the new bounding box for this node.
      auto parent        = &m_Nodes[metanode->Parent];
      auto& childInParent = *(&parent->A + metanode->IndexInParent);
      BoundingBox::CreateMerged(node->A.Min, node->A.Max, node->B.Min, node->B.Max, childInParent.Min, childInParent.Max);
      node = parent;
      metanode =  m_Metanodes.m_Memory + metanode->Parent;
    }
  }
}