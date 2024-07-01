#include "CepuPhysicsPCH.h"
#include "Tree.h"
#include "Memory/BufferPool.h"
#include "BoundingBox.h"
#include "Tree_BinnedRefine.h"

using namespace CepuUtil;

namespace CepuPhysics
{
  float Tree::RefitAndMeasure(NodeChild& child)
  {
    auto& node = m_Nodes[child.Index];

    //All nodes are guaranteed to have at least 2 children
    assert(m_LeafCount >= 2);

    auto premetric = ComputeBoundsMetric(child.Min, child.Max);
    float childChange = 0;
    auto& a = node.A;
    if (a.Index >= 0)
      childChange += RefitAndMeasure(a);

    auto& b = node.B;
    if (b.Index >= 0)
      childChange += RefitAndMeasure(b);

    BoundingBox::CreateMerged(a.Min, a.Max, b.Min, b.Max, child.Min, child.Max);

    auto postmetric = ComputeBoundsMetric(child.Min, child.Max);
    return postmetric - premetric + childChange; //TODO: would clamping produce a superior result?
  }


  float Tree::RefitAndMark(NodeChild& child, int32_t leafCountThreshold, std::vector<int32_t>& refinementCandidates, CepuUtil::BufferPool* pool) //@QUICKLIST (alektron)
  {
    assert(leafCountThreshold > 1);

    auto& node = m_Nodes[child.Index];
    assert(m_Metanodes[child.Index].RefineFlag == 0);
    float childChange = 0;

    auto premetric = ComputeBoundsMetric(child.Min, child.Max);
    //The wavefront of internal nodes is defined by the transition from more than threshold to less than threshold.
    //Add them to a list of refinement candidates.
    //Note that leaves are not included, since they can't be refinement candidates.
    auto& a = node.A;
    if (a.Index >= 0)
    {
      if (a.LeafCount <= leafCountThreshold)
      {
        refinementCandidates.push_back(a.Index);
        childChange += RefitAndMeasure(a);
      }
      else
      {
        childChange += RefitAndMark(a, leafCountThreshold, refinementCandidates, pool);
      }
    }
    auto& b = node.B;
    if (b.Index >= 0)
    {
      if (b.LeafCount <= leafCountThreshold)
      {
        refinementCandidates.push_back(b.Index);
        childChange += RefitAndMeasure(b);
      }
      else
      {
        childChange += RefitAndMark(b, leafCountThreshold, refinementCandidates, pool);
      }
    }

    BoundingBox::CreateMerged(a.Min, a.Max, b.Min, b.Max, child.Min, child.Max);

    auto postmetric = ComputeBoundsMetric(child.Min, child.Max);

    return postmetric - premetric + childChange; //TODO: Would clamp provide better results?
  }


  float Tree::RefitAndMark(int32_t leafCountThreshold, std::vector<int32_t>& refinementCandidates, CepuUtil::BufferPool* pool) //@QUICKLIST (alektron)
  {
    assert(m_LeafCount > 2 && "There's no reason to refit a tree with 2 or less elements. Nothing would happen");

    auto& children = m_Nodes[0].A;
    float childChange = 0;
    auto merged = BoundingBox{ glm::vec3(std::numeric_limits<float>::max()), glm::vec3(std::numeric_limits<float>::lowest()) };
    for (int32_t i = 0; i < 2; ++i) {
      //Note: these conditions mean the root will never be considered a wavefront node. That's acceptable;
      //it will be included regardless.
      auto& child = *(&children + i);
      if (child.Index >= 0) {
        if (child.LeafCount <= leafCountThreshold) {
          //The wavefront of internal nodes is defined by the transition from more than threshold to less than threshold.
          //Since we don't traverse into these children, there is no need to check the parent's leaf count.
          refinementCandidates.push_back(child.Index); //@QUICKLIST (alektron)
          childChange += RefitAndMeasure(child);
        }
        else
          childChange += RefitAndMark(child, leafCountThreshold, refinementCandidates, pool);
      }
      BoundingBox::CreateMerged(child.Min, child.Max, merged.m_Min, merged.m_Max, merged.m_Min, merged.m_Max);
    }

    auto postmetric = ComputeBoundsMetric(merged);

    //Note that the root's own change is not included.
    //This cost change is used to determine whether or not to refine.
    //Since refines are unable to change the volume of the root, there's
    //no point in including it in the volume change.
    //It does, however, normalize the child volume changes into a cost metric.
    if (postmetric >= 1e-10)
    {
      return childChange / postmetric;
    }
    return 0;
  }

  void Tree::GetRefineTuning(int32_t frameIndex, int32_t refinementCndidatesCount, float refineAggressivenessScale, float costChange,
    int32_t& o_targetRefinementCount, int32_t& o_refinementPeriod, int32_t& o_refinementOffset) const
  {
    if (glm::isnan(costChange) || glm::isinf(costChange)) {
      throw "The change in tree cost is an invalid value, strongly implying the tree bounds have been corrupted by infinites or NaNs. "
        "If this happened in the broad phase's use of the tree, it's likely that there are invalid poses or velocities in the simulation, "
        "possibly as a result of bugged input state or constraint configuration. "
        "Try running the library with debug asserts enabled to narrow down where the NaNsplosion started.";
    }

    auto refineAggressiveness = glm::max(0.0f, costChange * refineAggressivenessScale);
    float refinePortion = glm::min(1.0f, refineAggressiveness * 0.25f);

    auto targetRefinementScale = glm::min((float)m_NodeCount, glm::max(2.0f, glm::ceil(refinementCndidatesCount * 0.03f)) + refinementCndidatesCount * refinePortion);
    //Note that the refinementCandidatesCount is used as a maximum instead of refinementCandidates + 1 for simplicity, since there's a chance
    //that the root would already be a refinementCandidate. Doesn't really have a significant effect either way.
    o_refinementPeriod = glm::max(1, (int32_t)(refinementCndidatesCount / targetRefinementScale));
    o_refinementOffset = (int32_t)((frameIndex * 236887691LL + 104395303LL) % glm::max(1, refinementCndidatesCount));
    o_targetRefinementCount = glm::min(refinementCndidatesCount, (int32_t)targetRefinementScale);
  }

  void Tree::GetRefitAndMarkTuning(int32_t& o_maximumSubtrees, int32_t& o_estimatedRefinementChandidateCount, int32_t& o_refinementLeafCountThreshold) const
  {
    o_maximumSubtrees = (int32_t)glm::sqrt(m_LeafCount) * 3;
    o_estimatedRefinementChandidateCount = (m_LeafCount * 2) / o_maximumSubtrees;
    o_refinementLeafCountThreshold = glm::min(m_LeafCount, o_maximumSubtrees);
  }

  int32_t Tree::GetCacheOptimizeTuning(int32_t maximumSubtrees, float costChange, float cacheOptimizeAggressivenessScale)
  {
    //TODO: Using cost change as the heuristic for cache optimization isn't a great idea. They don't always or even frequently correlate.
    //The best heuristic would be directly measuring the degree of adjacency. We could do that in the refit. I'm not addressing this yet
    //because there's a good chance the cache optimization approach will change significantly (for example, refit outputting into a new tree with heuristically perfect layout).
    auto cacheOptimizeAggressiveness = glm::max(0.0f, costChange * cacheOptimizeAggressivenessScale);
    float cacheOptimizePortion = glm::min(1.0f, 0.03f + 85.f * (maximumSubtrees / (float)m_LeafCount) * cacheOptimizeAggressiveness);
    //float cacheOptimizePortion = Math.Min(1, 0.03f + cacheOptimizeAggressiveness * 0.5f);
    //Console.WriteLine($"cache optimization portion: {cacheOptimizePortion}");
    return (int)glm::ceil(cacheOptimizePortion * m_NodeCount);
  }

  void Tree::RefitAndRefine(BufferPool* pool, int32_t frameIndex, float refineAggressivenessScale, float cacheOptimizeAggressivenessScale)
  {
    //Don't proceed if the tree has no refitting refinement required. This also guarantees that any nodes that do exist have two children
    if (m_LeafCount <= 2)
      return;

    int32_t maximumSubtrees, estimatedRefinementCandidateCount, leafCountThreshold;
    GetRefitAndMarkTuning(maximumSubtrees, estimatedRefinementCandidateCount, leafCountThreshold);
    std::vector<int32_t> refinementCandidates; //@QUICKLIST (alektron)
    refinementCandidates.reserve(estimatedRefinementCandidateCount);

    //Collect the refinement candidates
    auto costChange = RefitAndMark(leafCountThreshold, refinementCandidates, pool);
    int32_t targetRefinementCount, period, offset;
    GetRefineTuning(frameIndex, (int32_t)refinementCandidates.size(), refineAggressivenessScale, costChange, targetRefinementCount, period, offset);

    std::vector<int32_t> refinementTargets; //@QUICKLIST (alektron)
    refinementTargets.reserve(targetRefinementCount);
    int32_t index = offset;
    for (int32_t i = 0; i < targetRefinementCount - 1; ++i) {
      index += period;
      if (index >= (int32_t)refinementCandidates.size())
        index -= (int32_t)refinementCandidates.size();
      refinementTargets.push_back(refinementCandidates[index]); //@TODO (alektron) AddUnsafely
      assert(m_Metanodes[refinementCandidates[index]].RefineFlag == 0 && "Refinement target seach shouldn't run into the same node twice");
      m_Metanodes[refinementCandidates[index]].RefineFlag = 1;
    }
    //refinementCandidates.Dispose @TODO (alektron)
    if (m_Metanodes[0].RefineFlag == 0) {
      refinementTargets.push_back(0); //@TODO (alektron) AddUnsafely
      m_Metanodes[0].RefineFlag = 1;
    }



    //Refine all marked targets.

    std::vector<int32_t> subtreeReferences   ; //@QUICKLIST (alektron)
    std::vector<int32_t> treeletInternalNodes; //@QUICKLIST (alektron)
    subtreeReferences   .reserve(maximumSubtrees); 
    treeletInternalNodes.reserve(maximumSubtrees); 

    BinnedResources resources;
    Buffer<uint8_t> buffer;
    CreateBinnedResources(pool, maximumSubtrees, buffer, resources);

    for (size_t i = 0; i < refinementTargets.size(); ++i)
    {

      subtreeReferences.resize(0);
      treeletInternalNodes.resize(0);
      BinnedRefine(refinementTargets[i], subtreeReferences, maximumSubtrees, treeletInternalNodes, resources, pool);
      //TODO: Should this be moved into a post-loop? It could permit some double work, but that's not terrible.
      //It's not invalid from a multithreading perspective, either- setting the refine flag to zero is essentially an unlock.
      //If other threads don't see it updated due to cache issues, it doesn't really matter- it's not a signal or anything like that.
      m_Metanodes[refinementTargets[i]].RefineFlag = 0;

    }

    for (int32_t i = 1; i < m_NodeCount; i++)
    {
      ValidateBounds(i);
    }

    pool->Return(buffer);
    //subtreeReferences.Dispose(pool);    //@TODO (alektron)
    //treeletInternalNodes.Dispose(pool); //@TODO (alektron)
    //refinementTargets.Dispose(pool);    //@TODO (alektron)

    auto cacheOptimizeCount = GetCacheOptimizeTuning(maximumSubtrees, costChange, cacheOptimizeAggressivenessScale);

    auto startIndex = (int)(((long)frameIndex * cacheOptimizeCount) % m_NodeCount);

    //We could wrap around. But we could also not do that because it doesn't really matter!
    auto end = glm::min(m_NodeCount, startIndex + cacheOptimizeCount);
    for (int i = startIndex; i < end; ++i)
    {
      //IncrementalCacheOptimize(i);
    }
  }
}