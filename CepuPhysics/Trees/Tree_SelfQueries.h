#pragma once

namespace CepuPhysics
{
  class Tree;
  struct Node;
  struct NodeChild;

  struct IOverlapHandler
  {
    virtual void Handle(int32_t indexA, int32_t indexB) = 0;
  };

  bool Intersects(const NodeChild& a, const NodeChild& b);

  template<typename TOverlapHandler>
  void TestLeafAgainstNode(const Tree&, int32_t, const glm::vec3&, const glm::vec3&, int32_t, TOverlapHandler&);

  //Note that all of these implementations make use of a fully generic handler. It could be dumping to a list, or it could be directly processing the results- at this
  //level of abstraction we don't know or care. It's up to the user to use a handler which maximizes performance if they want it. We'll be using this in the broad phase.
  template<typename TOverlapHandler>
  void DispatchTestForLeaf(const Tree& tree, int32_t leafIndex, const glm::vec3& leafMin, const glm::vec3& leafMax, int32_t nodeIndex, TOverlapHandler& results)
  {
    if (nodeIndex < 0)
    {
      results.Handle(leafIndex, Tree::Encode(nodeIndex));
    }
    else
    {
      TestLeafAgainstNode(tree, leafIndex, leafMin, leafMax, nodeIndex, results);
    }
  }

  template<typename TOverlapHandler>
  void TestLeafAgainstNode(const Tree& tree, int32_t leafIndex, const glm::vec3& leafMin, const glm::vec3& leafMax, int32_t nodeIndex, TOverlapHandler& results)
  {
    auto& node = tree.m_Nodes[nodeIndex];
    auto& a = node.A;
    auto& b = node.B;
    //Despite recursion, leafBounds should remain in L1- it'll be used all the way down the recursion from here.
    //However, while we likely loaded child B when we loaded child A, there's no guarantee that it will stick around.
    //Reloading that in the event of eviction would require more work than keeping the derived data on the stack.
    //TODO: this is some pretty questionable microtuning. It's not often that the post-leaf-found recursion will be long enough to evict L1. Definitely test it.
    auto bIndex = b.Index;
    auto aIntersects = CepuUtil::BoundingBox::Intersects(leafMin, leafMax, a.Min, a.Max);
    auto bIntersects = CepuUtil::BoundingBox::Intersects(leafMin, leafMax, b.Min, b.Max);
    if (aIntersects)
    {
      DispatchTestForLeaf(tree, leafIndex, leafMin, leafMax, a.Index, results);
    }
    if (bIntersects)
    {
      DispatchTestForLeaf(tree, leafIndex, leafMin, leafMax, bIndex, results);
    }
  }

  template<typename TOverlapHandler>
  void DispatchTestForNodes(const Tree&, const NodeChild&, const NodeChild&, TOverlapHandler&);

  template<typename TOverlapHandler>
  void GetOverlapsBetweenDifferentNodes(const Tree& tree, const Node& a, const Node& b, TOverlapHandler& results) 
  {
    //There are no shared children, so test them all.
    auto& aa = a.A;
    auto& ab = a.B;
    auto& ba = b.A;
    auto& bb = b.B;
    auto aaIntersects = Intersects(aa, ba);
    auto abIntersects = Intersects(aa, bb);
    auto baIntersects = Intersects(ab, ba);
    auto bbIntersects = Intersects(ab, bb);

    if (aaIntersects)
    {
        DispatchTestForNodes(tree, aa, ba, results);
    }
    if (abIntersects)
    {
        DispatchTestForNodes(tree, aa, bb, results);
    }
    if (baIntersects)
    {
        DispatchTestForNodes(tree, ab, ba, results);
    }
    if (bbIntersects)
    {
        DispatchTestForNodes(tree, ab, bb, results);
    }
  }

  template<typename TOverlapHandler>
  void DispatchTestForNodes(const Tree& tree, const NodeChild& a, const NodeChild& b, TOverlapHandler& results) 
  {
    if (a.Index >= 0)
    {
      if (b.Index >= 0)
      {
        GetOverlapsBetweenDifferentNodes(tree, tree.m_Nodes[a.Index], tree.m_Nodes[b.Index], results);
      }
      else
      {
        //leaf B versus node A.
        TestLeafAgainstNode(tree, Tree::Encode(b.Index), b.Min, b.Max, a.Index, results);
      }
    }
    else if (b.Index >= 0)
    {
      //leaf A versus node B.
      TestLeafAgainstNode(tree, Tree::Encode(a.Index), a.Min, a.Max, b.Index, results);
    }
    else
    {
      //Two leaves.
      results.Handle(Tree::Encode(a.Index), Tree::Encode(b.Index));
    }
  }

  template<typename TOverlapHandler>
  void GetOverlapsInNode(const Tree& tree, const Node& node, TOverlapHandler& results)
  {
    auto& a = node.A;
    auto& b = node.B;

    bool ab = Intersects(a, b);

    if (a.Index >= 0)
      GetOverlapsInNode(tree, tree.m_Nodes[a.Index], results);
    if (b.Index >= 0)
      GetOverlapsInNode(tree, tree.m_Nodes[b.Index], results);

    //Test all different nodes.
    if (ab)
      DispatchTestForNodes(tree, a, b, results);
  }

  template<typename TOverlapHandler>
  void GetSelfOverlaps(const Tree& tree, TOverlapHandler& results)
  {
    //If there are less than two leaves, there can't be any overlap.
    //This provides a guarantee that there are at least 2 children in each internal node considered by GetOverlapsInNode.
    if (tree.m_LeafCount < 2)
      return;

    GetOverlapsInNode(tree, tree.m_Nodes[0], results);
  }
}
