#pragma once

namespace CepuPhysics
{
  enum class ContinuousDetectionMode : int
  {
    DISCRETE   = 0,
    PASSIVE    = 1,
    CONTINUOUS = 2,
  };

  struct ContinuousDetection
  {
    ContinuousDetectionMode m_Mode = ContinuousDetectionMode::DISCRETE;
    float m_MinimumSpeculativeMargin = 0;
    float m_MaximumSpeculativeMargin = 0;
    float m_MinimumSweepTimestep = 0;
    float m_SweepConvergenceThreshold = 0;

    bool AllowExpansionBeyondSpeculativeMargin() const { return (int)m_Mode > 0; }
  };

  struct Collidable
  {
    ContinuousDetection m_Continuity;
    TypedIndex m_Shape;
    float m_SpeculativeMargin = 0;
    int32_t m_BroadPhaseIndex;
  };
}
