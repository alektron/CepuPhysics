#include "CepuPhysicsPCH.h"
#include "Tree_RefineCommon.h"
#include "Node.h"
#include "Tree.h"

namespace CepuPhysics
{
  class SubtreeBinaryHeap
  {
  public:
    SubtreeBinaryHeap(SubtreeHeapEntry* entries)
      : m_Entries(entries)
    {}

    void Insert(const Node& node, std::vector<int32_t>& subtrees) //@QUICKLIST (alektron)
    {
      auto& children = node.A;
      for (int childIndex = 0; childIndex < 2; ++childIndex)
      {
        auto& child = (&children)[childIndex];
        if (child.Index >= 0)
        {
          int index = m_Count;
          auto cost = Tree::ComputeBoundsMetric(child.Min, child.Max);// - node->PreviousMetric;
          ++m_Count;

          //Sift up.
          while (index > 0)
          {
            auto parentIndex = (index - 1) >> 1;
            auto parent = m_Entries + parentIndex;
            if (parent->m_Cost < cost)
            {
              //Pull the parent down.
              m_Entries[index] = *parent;
              index = parentIndex;
            }
            else
            {
              //Found the insertion spot.
              break;
            }
          }
          auto entry = m_Entries + index;
          entry->m_Index = child.Index;
          entry->m_Cost = cost;

        }
        else
        {
          //Immediately add leaf nodes.
          subtrees.push_back(child.Index); //@QUICKLIST (alektron) AddUnsafely
        }
      }
    }

    void Pop(SubtreeHeapEntry& o_entry)
    {
      o_entry = m_Entries[0];
      --m_Count;
      auto cost = m_Entries[m_Count].m_Cost;

      //Pull the elements up to fill in the gap.
      int index = 0;
      while (true)
      {
        auto childIndexA = (index << 1) + 1;
        auto childIndexB = (index << 1) + 2;
        if (childIndexB < m_Count)
        {
          //Both children are available.
          //Try swapping with the largest one.
          auto childA = m_Entries + childIndexA;
          auto childB = m_Entries + childIndexB;
          if (childA->m_Cost > childB->m_Cost)
          {
            if (cost > childA->m_Cost)
            {
              break;
            }
            m_Entries[index] = m_Entries[childIndexA];
            index = childIndexA;
          }
          else
          {
            if (cost > childB->m_Cost)
            {
              break;
            }
            m_Entries[index] = m_Entries[childIndexB];
            index = childIndexB;
          }
        }
        else if (childIndexA < m_Count)
        {
          //Only one child was available.
          auto childA = m_Entries + childIndexA;
          if (cost > childA->m_Cost)
          {
            break;
          }
          m_Entries[index] = m_Entries[childIndexA];
          index = childIndexA;
        }
        else
        {
          //The children were beyond the heap.
          break;
        }
      }
      //Move the last entry into position.
      m_Entries[index] = m_Entries[m_Count];
    }

    bool TryPop(const CepuUtil::Buffer<MetaNode>& metanodes, int32_t& io_remainingSubtreeSpace, std::vector<int32_t>& subtrees, int32_t& o_index, float& o_cost)
    {
      while (m_Count > 0)
      {
        //Repeatedly pop minimum until you find one that can fit.
        //Given the unique access nature, the fact that you're destroying the heap when there's not much space left doesn't matter.
        //In the event that you consume all the nodes, that just means there aren't any entries which would fit in the subtree set anymore.
        SubtreeHeapEntry entry;
        Pop(entry);
        //Choose to expand this node, or not.
        //Only choose to expand if its children will fit.
        //Any time a node is expanded, the existing node is removed from the set of potential subtrees stored in the priorityQueue.
        //So, the change in remainingSubtreeSpace = maximumSubtreesCount - (priorityQueue.Count + subtrees.Count) is childCount - 1.
        //This is ALWAYS the case.
        if (io_remainingSubtreeSpace >= 1 && metanodes[entry.m_Index].RefineFlag == 0)
        {
          //Debug.Fail("don't forget to reenable the refine flag condition");
          //This node's children can be included successfully in the remaining space.
          o_index = entry.m_Index;
          o_cost  = entry.m_Cost;
          io_remainingSubtreeSpace -= 1;
          return true;
        }
        else
        {
          //Either this node's children did not fit, or it was a refinement target. Refinement targets cannot be expanded.
          //Since we won't be able to find this later, it needs to be added now.
          //We popped the previous entry off the queue, so the remainingSubtreeSpace does not change by re-adding it.
          //(remainingSubtreeSpace = maximumSubtreesCount - (priorityQueue.Count + subtrees.Count))
          subtrees.push_back(entry.m_Index); //@QUICKLIST (alektron) AddUnsafely
        }
      }
      o_index = -1;
      o_cost  = -1;
      return false;
    }

    SubtreeHeapEntry* m_Entries = nullptr;
    int32_t m_Count = 0;
  };

  void Tree::CollectSubtrees(int32_t nodeIndex, int32_t maximumSubtrees, SubtreeHeapEntry* entries, std::vector<int32_t>& subtrees, std::vector<int32_t>& internalNodes, float& o_treeletCost)
  {
    //Collect subtrees iteratively by choosing the highest cost subtree repeatedly.
    //This collects every child of a given node at once- the set of subtrees must not include only SOME of the children of a node.

    //(You could lift this restriction and only take some nodes, but it would complicate things. You could not simply remove
    //the parent and add its children to go deeper; it would require doing some post-fixup on the results of the construction
    //or perhaps constraining the generation process to leave room for the unaffected nodes.)


    auto& node = m_Nodes[nodeIndex];
    assert(maximumSubtrees >= 2 && "Can't only consider some of a node's children, but specified maximumSubtrees precludes the treelet root's children.");
    //All of treelet root's children are included immediately. (Follows from above requirement.)

    auto priorityQueue = SubtreeBinaryHeap(entries);

    priorityQueue.Insert(node, subtrees);

    //Note that the treelet root is NOT added to the internal nodes list.

    //Note that the treelet root's cost is excluded from the treeletCost.
    //That's because the treelet root cannot change.
    o_treeletCost = 0;
    int remainingSubtreeSpace = maximumSubtrees - priorityQueue.m_Count - (int32_t)subtrees.size();
    int32_t highestIndex;
    float highestCost;
    while (priorityQueue.TryPop(m_Metanodes, remainingSubtreeSpace, subtrees, highestIndex, highestCost))
    {
      o_treeletCost += highestCost;
      internalNodes.push_back(highestIndex); //@QUICKLIST (alektron) AddUnsafely

      //Add all the children to the set of subtrees.
      //This is safe because we pre-validated the number of children in the node.
      auto& expandedNode = m_Nodes[highestIndex];
      priorityQueue.Insert(expandedNode, subtrees);
    }

    for (int i = 0; i < priorityQueue.m_Count; ++i)
    {
      subtrees.push_back(priorityQueue.m_Entries[i].m_Index); //@QUICKLIST (alektron) AddUnsafely
    }

    //Sort the internal nodes so that the depth first builder will tend to produce less cache-scrambled results.
    //TODO: Note that the range of values in node indices is VERY often constrained to values below 65535. In other words, a radix sort would be a significant win.
    //Even better, use some form of implicit cache optimization that eliminates the possibility of noncontiguous internal nodes, so that this entire collection process
    //boils down to computing a range of elements based on the root index and leaf count. Dealing with the root node is a little more complex.
    //I suspect this entire function is going to go away later. For example:
    //An internally multithreaded root builder that always runs, treating a set of subtrees as leaves, followed by externally multithreaded subtree refines.
    //The root builder would write out its nodes into a new block of memory rather than working in place.
    //If the root builder terminates with a set of subtrees of known leaf counts and known positions, then multithreaded refines will execute on contiguous regions.
    //In other words, at no point is a sort of target nodes required, because they're all computed analytically and they are known to be in cache optimal locations.
    if (internalNodes.size() > 0) //It's possible for there to be no internal nodes if both children of the target node were leaves.
    {
      std::qsort(internalNodes.data(), internalNodes.size(), sizeof(std::vector<int32_t>::value_type), [](const void *x, const void *y) {
        const int32_t a = *static_cast<const int32_t*>(x);
        const int32_t b = *static_cast<const int32_t*>(y);
        return a > b ? 1 : a < b ? -1 : 0;
      }); //@TODO @QUICKLIST @STD @SORT (alektron)
    }
  }

  void Tree::ValidateStaging(Node* stagingNodes, int32_t stagingNodeIndex,
    std::vector<int32_t>& subtreeNodePointers, std::vector<int32_t>& collectedSubtreeReferences,
    std::vector<int32_t>& internalReferences, CepuUtil::BufferPool* pool, int32_t& foundSubtrees, int32_t& foundLeafCount)
  {
    auto stagingNode = stagingNodes + stagingNodeIndex;
    auto children = &stagingNode->A;
    foundSubtrees = foundLeafCount = 0;
    for (int i = 0; i < 2; ++i)
    {
      auto& child = children[i];
      if (child.Index >= 0)
      {
        if (std::find(internalReferences.begin(), internalReferences.end(), child.Index) != internalReferences.end())
          throw "A child points to an internal node that was visited. Possible loop, or just general invalid.";
        internalReferences.push_back(child.Index);
        int32_t childFoundSubtrees, childFoundLeafCount;
        ValidateStaging(stagingNodes, child.Index, subtreeNodePointers, collectedSubtreeReferences, internalReferences, pool, childFoundSubtrees, childFoundLeafCount);

        if (childFoundLeafCount != child.LeafCount)
          throw "Bad leaf count.";
        foundSubtrees += childFoundSubtrees;
        foundLeafCount += childFoundLeafCount;
      }
      else
      {
        auto subtreeNodePointerIndex = Encode(child.Index);
        auto subtreeNodePointer = subtreeNodePointers[subtreeNodePointerIndex];
        //Rather than looking up the shuffled SweepSubtree for information, just go back to the source.
        if (subtreeNodePointer >= 0)
        {
          auto& node = m_Nodes[subtreeNodePointer];
          auto totalLeafCount = 0;
          for (int childIndex = 0; childIndex < 2; ++childIndex)
          {
            totalLeafCount += (&node.A)[childIndex].LeafCount;
          }

          if (child.LeafCount != totalLeafCount)
            throw "bad leaf count.";
          foundLeafCount += totalLeafCount;
        }
        else
        {
          auto leafIndex = Encode(subtreeNodePointer);
          if (child.LeafCount != 1)
            throw "bad leaf count.";
          foundLeafCount += 1;
        }
        ++foundSubtrees;
        collectedSubtreeReferences.push_back(subtreeNodePointer);
      }
    }
  }


}
