#include "CepuPhysicsPCH.h"
#include "Tree.h"
#include "Memory/BufferPool.h"
#include "BoundingBox.h"

using namespace CepuUtil;

namespace CepuPhysics
{
  void Tree::RemoveNodeAt(int32_t nodeIndex)
  {
    //Note that this function is a cache scrambling influence. That's okay- the cache optimization routines will take care of it later.
    assert(nodeIndex < m_NodeCount && nodeIndex >= 0);
    //We make no guarantees here about maintaining the tree's coherency after a remove.
    //That's the responsibility of whoever called RemoveAt.
    --m_NodeCount;
    //If the node wasn't the last node in the list, it will be replaced by the last node.
    if (nodeIndex < m_NodeCount)
    {
      //Swap last node for removed node.
      auto& node = m_Nodes[nodeIndex];
      node = m_Nodes[m_NodeCount];
      auto& metanode = m_Metanodes[nodeIndex];
      metanode = m_Metanodes[m_NodeCount];

      //Update the moved node's pointers:
      //its parent's child pointer should change, and...
      (&m_Nodes[metanode.Parent].A + metanode.IndexInParent)->Index = nodeIndex;
      //its children's parent pointers should change.
      auto& nodeChildren = node.A;
      for (int i = 0; i < 2; ++i)
      {
        auto& child = *(&nodeChildren + i);
        if (child.Index >= 0)
        {
          m_Metanodes[child.Index].Parent = nodeIndex;
        }
        else
        {
          //It's a leaf node. It needs to have its pointers updated.
          m_Leaves[Encode(child.Index)] = Leaf(nodeIndex, i);
        }
      }

    }


  }

  void Tree::RefitForRemoval(int32_t nodeIndex)
  {
    //Note that no attempt is made to refit the root node. Note that the root node is the only node that can have a number of children less than 2.
    auto& node =  m_Nodes[nodeIndex];
    auto& metanode =  m_Metanodes[nodeIndex];
    while (metanode.Parent >= 0)
    {
      //Compute the new bounding box for this node.
      auto& parent =  m_Nodes[metanode.Parent];
      auto& childInParent = *(&parent.A + metanode.IndexInParent);
      BoundingBox::CreateMerged(node.A.Min, node.A.Max, node.B.Min, node.B.Max, childInParent.Min, childInParent.Max);
      --childInParent.LeafCount;
      node =  parent;
      metanode =  m_Metanodes[metanode.Parent];
    }
  }

  int32_t Tree::RemoveAt(int32_t leafIndex)
  {
    if (leafIndex < 0 || leafIndex >= m_LeafCount)
      throw ("Leaf index must be a valid index in the tree's leaf array.");

    //Cache the leaf being removed.
    auto leaf = m_Leaves[leafIndex];
    //Delete the leaf from the leaves array.
    --m_LeafCount;
    if (leafIndex < m_LeafCount)
    {
      //The removed leaf was not the last leaf, so we should move the last leaf into its slot.
      //This can result in a form of cache scrambling, but these leaves do not need to be referenced during high performance stages.
      //It does somewhat reduce the performance of AABB updating, but we shouldn't bother with any form of cache optimization for this unless it becomes a proven issue.
      auto& lastLeaf =  m_Leaves[m_LeafCount];
      m_Leaves[leafIndex] = lastLeaf;
      (&m_Nodes[lastLeaf.GetNodeIndex()].A + lastLeaf.GetChildIndex())->Index = Encode(leafIndex);
    }

    auto& node =  m_Nodes[leaf.GetNodeIndex()];
    auto& metanode =  m_Metanodes[leaf.GetNodeIndex()];
    auto& nodeChildren =  node.A;

    //Remove the leaf from this node.
    //Note that the children must remain contiguous. Requires taking the last child of the node and moving into the slot
    //if the removed child was not the last child.
    //Further, if a child is moved and if it is a leaf, that leaf's ChildIndex must be updated.
    //If a child is moved and it is an internal node, all immediate children of that node must have their parent nodes updated.

    auto survivingChildIndexInNode = leaf.GetChildIndex() ^ 1;
    auto& survivingChild =  *(&nodeChildren + survivingChildIndexInNode);

    //Check to see if this node should collapse.
    if (metanode.Parent >= 0)
    {
      //This is a non-root internal node.
      //Since there are only two children in the node, then the node containing the removed leaf will collapse.

      //Move the other node into the slot that used to point to the collapsing internal node.
      auto& childInParent =  *(&m_Nodes[metanode.Parent].A + metanode.IndexInParent);
      childInParent.Min = survivingChild.Min;
      childInParent.Max = survivingChild.Max;
      childInParent.Index = survivingChild.Index;
      childInParent.LeafCount = survivingChild.LeafCount;

      if (survivingChild.Index < 0)
      {
        //It's a leaf. Update the leaf's reference in the leaves array.
        auto otherLeafIndex = Encode(survivingChild.Index);
        m_Leaves[otherLeafIndex] = Leaf(metanode.Parent, metanode.IndexInParent);
      }
      else
      {
        //It's an internal node. Update its parent node.
        auto& survivingMeta =  m_Metanodes[survivingChild.Index];
        survivingMeta.Parent = metanode.Parent;
        survivingMeta.IndexInParent = metanode.IndexInParent;
      }

      //Work up the chain of parent pointers, refitting bounding boxes and decrementing leaf counts.
      //Note that this starts at the parent; we've already done the refit for the current level via collapse.
      RefitForRemoval(metanode.Parent);

      //Remove the now dead node.
      RemoveNodeAt(leaf.GetNodeIndex());


    }
    else
    {
      //This is the root. It cannot collapse, but if the other child is an internal node, then it will overwrite the root node.
      //This maintains the guarantee that any tree with at least 2 leaf nodes has every single child slot filled with a node or leaf.
      assert(leaf.GetNodeIndex() == 0 && "Only the root should have a negative parent, so only the root should show up here.");
      if (m_LeafCount > 0)
      {
        //The post-removal leafCount is still positive, so there must be at least one child in the root node.
        //If it is an internal node, then it will be promoted into the root node's slot.
        if (survivingChild.Index >= 0)
        {
          //The surviving child is an internal node and it should replace the root node.
          auto pulledNodeIndex = survivingChild.Index;
          //TODO: This node movement logic could be unified with other instances of node moving. Nothing too special about the fact that it's the root.
          m_Nodes[0] = m_Nodes[pulledNodeIndex]; //Note that this overwrites the memory pointed to by the otherChild reference.
          m_Metanodes[0].Parent = -1;
          m_Metanodes[0].IndexInParent = -1;
          //Update the parent pointers of the children of the moved internal node.
          for (int i = 0; i < 2; ++i)
          {
            auto& child =  *(&m_Nodes[0].A + i);
            if (child.Index >= 0)
            {
              //Child is an internal node. Note that the index in child doesn't change; we copied the children directly.
              m_Metanodes[child.Index].Parent = 0;
            }
            else
            {
              //Child is a leaf node.
              m_Leaves[Encode(child.Index)] = Leaf(0, i);
            }
          }
          RemoveNodeAt(pulledNodeIndex);
        }
        else
        {
          //The surviving child is a leaf node.
          if (survivingChildIndexInNode > 0)
          {
            //It needs to be moved to keep the lowest slot filled.
            m_Nodes[0].A = m_Nodes[0].B;
            //Update the leaf pointer to reflect the change.
            m_Leaves[Encode(survivingChild.Index)] = Leaf(0, 0);
          }
        }
      }
      //No need to perform a RefitForRemoval here; it's the root. There is no higher bounding box.
    }
    return leafIndex < m_LeafCount ? m_LeafCount : -1;
  }
}