#include "CepuPhysicsPCH.h"
#include "Tree.h"

namespace CepuPhysics
{
  void Tree::SwapNodes(int32_t indexA, int32_t indexB)
  {
    auto& a = m_Nodes[indexA];
    auto& b = m_Nodes[indexB];
    auto& metaA = m_Metanodes[indexA];
    auto& metaB = m_Metanodes[indexB];

    std::swap(a, b);
    std::swap(metaA, metaB);

    if (metaA.Parent == indexA)
    {
      //The original B's parent was A.
      //That parent has moved.
      metaA.Parent = indexB;
    }
    else if (metaB.Parent == indexB)
    {
      //The original A's parent was B.
      //That parent has moved.
      metaB.Parent = indexA;
    }
    (&m_Nodes[metaA.Parent].A)[metaA.IndexInParent].Index = indexA;
    (&m_Nodes[metaB.Parent].A)[metaB.IndexInParent].Index = indexB;


    //Update the parent pointers of the children.
    auto& children = a.A;
    for (int i = 0; i < 2; ++i)
    {
      auto& child = (&children)[i];
      if (child.Index >= 0)
      {
        m_Metanodes[child.Index].Parent = indexA;
      }
      else
      {
        auto leafIndex = Encode(child.Index);
        m_Leaves[leafIndex] = Leaf(indexA, i);
      }
    }
    children = b.A;
    for (int i = 0; i < 2; ++i)
    {
      auto& child = (&children)[i];
      if (child.Index >= 0)
      {
        m_Metanodes[child.Index].Parent = indexB;
      }
      else
      {
        auto leafIndex = Encode(child.Index);
        m_Leaves[leafIndex] = Leaf(indexB, i);
      }
    }

  }

  void Tree::IncrementalCacheOptimize(int32_t nodeIndex)
  {
    if (m_LeafCount <= 2)
    {
      //Don't bother cache optimizing if there are only two leaves. There's no work to be done, and it supplies a guarantee to the rest of the optimization logic
      //so that we don't have to check per-node child counts.
      return;
    }

    auto& node = m_Nodes[nodeIndex];
    auto& children = node.A;
    auto targetIndex = nodeIndex + 1;

    //Note that we pull all children up to their final positions relative to the current node index.
    //This helps ensure that more nodes can converge to their final positions- if we didn't do this,
    //a full top-down cache optimization could end up leaving some nodes near the bottom of the tree and without any room for their children.
    //TODO: N-ary tree support. Tricky without subtree count and without fixed numbers of children per node, but it may be possible
    //to stil choose something which converged.

    for (int i = 0; i < 2; ++i)
    {
      if (targetIndex >= m_NodeCount)
      {
        //This attempted swap would reach beyond the allocated nodes.
        //That means the current node is quite a bit a lower than it should be.
        //Later refinement attempts should fix this, but for now, do nothing.
        //Other options:
        //We could aggressively swap this node upward. More complicated.
        break;
      }
      auto& child = (&children)[i];
      if (child.Index >= 0)
      {
        if (child.Index != targetIndex)
        {
          SwapNodes(child.Index, targetIndex);
        }
        //break;
        targetIndex += child.LeafCount - 1;
      }
    }
  }
}