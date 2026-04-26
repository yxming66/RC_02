#ifndef ARM_LIB_SOLVER_SEED_POLICY_H
#define ARM_LIB_SOLVER_SEED_POLICY_H

#include "../model/serial_chain.h"
#include "solver_common.h"

namespace arm_lib {
namespace solver {

enum class SeedSource : uint8_t {
  kUnknown = 0,
  kUserSeed,
  kCurrentState,
  kReference,
  kZeroConfiguration,
  kLimitMidpoint,
  kAnalyticSolution,
  kPerturbation,
};

struct SeedRankingWeights {
  Scalar distance_to_current;
  Scalar distance_to_reference;
  Scalar limit_margin_reward;
  Scalar singularity_penalty;
  Scalar source_bias;

  SeedRankingWeights()
      : distance_to_current(1.0f),
        distance_to_reference(0.5f),
        limit_margin_reward(0.2f),
        singularity_penalty(0.0f),
        source_bias(0.1f) {}
};

template <int N>
struct SeedCandidate {
  JointVec<N> q;
  SeedSource source;
  Scalar score;
  bool valid;

  SeedCandidate()
      : q(toolbox_adapter::zero_joint_vec<N>()),
        source(SeedSource::kUnknown),
        score(0.0f),
        valid(false) {}
};

template <int N, int MaxCandidates = 12>
struct SeedCandidateSet {
  SeedCandidate<N> items[MaxCandidates];
  uint16_t count;

  SeedCandidateSet() : count(0U) {}
};

template <int N, int MaxCandidates = 12>
inline bool append_seed_candidate(SeedCandidateSet<N, MaxCandidates>* set,
                                  const JointVec<N>& q,
                                  SeedSource source) {
  if (set == nullptr || set->count >= MaxCandidates) {
    return false;
  }

  set->items[set->count].q = q;
  set->items[set->count].source = source;
  set->items[set->count].score = 0.0f;
  set->items[set->count].valid = is_joint_vector_finite(q);
  ++set->count;
  return true;
}

template <int N>
inline JointVec<N> select_initial_seed(const SerialChain<N>& chain,
                                       const JointVec<N>& seed,
                                       bool use_seed) {
  if (use_seed) {
    return seed;
  }
  return chain.zero_configuration();
}

inline Scalar nearest_equivalent_angle(Scalar angle, Scalar reference) {
  return reference + angle_distance(reference, angle);
}

template <int N>
inline JointVec<N> align_revolute_to_reference(const SerialChain<N>& chain,
                                               const JointVec<N>& candidate,
                                               const JointVec<N>& reference) {
  JointVec<N> aligned = candidate;
  for (uint16_t i = 0; i < N; ++i) {
    const Link& link = chain.link(i);
    if (link.joint_type() != ChainJointType::kRevolute) {
      continue;
    }
    if (link.limit_enabled() &&
        (link.upper_limit() - link.lower_limit()) < (2.0f * ARM_LIB_PI)) {
      continue;
    }
    aligned[i][0] = nearest_equivalent_angle(candidate[i][0], reference[i][0]);
  }
  return aligned;
}

template <int N>
inline JointVec<N> clamp_to_chain_limits(const SerialChain<N>& chain,
                                         const JointVec<N>& q) {
  return clamp_to_joint_limits(chain.joint_limits(), q);
}

template <int N>
inline JointVec<N> joint_limit_midpoint_seed(const SerialChain<N>& chain) {
  JointVec<N> seed = chain.zero_configuration();
  for (uint16_t i = 0; i < N; ++i) {
    const Link& link = chain.link(i);
    if (link.limit_enabled()) {
      seed[i][0] = 0.5f * (link.lower_limit() + link.upper_limit());
    }
  }
  return seed;
}

template <int N, int MaxCandidates = 12>
inline void append_standard_seed_candidates(
    const SerialChain<N>& chain,
    const JointVec<N>* current,
    bool has_current,
    const JointVec<N>* reference,
    bool has_reference,
    const JointVec<N>* user_seed,
    bool use_user_seed,
    SeedCandidateSet<N, MaxCandidates>* out) {
  if (out == nullptr) {
    return;
  }

  if (use_user_seed && user_seed != nullptr) {
    (void)append_seed_candidate(out, *user_seed, SeedSource::kUserSeed);
  }
  if (has_current && current != nullptr) {
    (void)append_seed_candidate(out, *current, SeedSource::kCurrentState);
  }
  if (has_reference && reference != nullptr) {
    (void)append_seed_candidate(out, *reference, SeedSource::kReference);
  }

  (void)append_seed_candidate(out, chain.zero_configuration(),
                              SeedSource::kZeroConfiguration);
  (void)append_seed_candidate(out, joint_limit_midpoint_seed(chain),
                              SeedSource::kLimitMidpoint);
}

template <int N>
inline JointVec<N> perturb_seed(const SerialChain<N>& chain,
                                const JointVec<N>& base,
                                uint16_t attempt_index,
                                Scalar offset) {
  JointVec<N> seed = base;
  if (N == 0 || abs_scalar(offset) <= ARM_LIB_EPSILON) {
    return seed;
  }

  const uint16_t joint_index = attempt_index % static_cast<uint16_t>(N);
  const Scalar sign = ((attempt_index / static_cast<uint16_t>(N)) % 2U == 0U)
                          ? 1.0f
                          : -1.0f;
  const Link& link = chain.link(joint_index);
  seed[joint_index][0] += sign * offset;
  if (link.limit_enabled()) {
    seed[joint_index][0] =
        clamp_scalar(seed[joint_index][0], link.lower_limit(), link.upper_limit());
  }
  return seed;
}

template <int N>
inline Scalar configuration_distance(const SerialChain<N>& chain,
                                     const JointVec<N>& lhs,
                                     const JointVec<N>& rhs) {
  Scalar distance = 0.0f;
  for (uint16_t i = 0; i < N; ++i) {
    Scalar delta = lhs[i][0] - rhs[i][0];
    if (chain.link(i).joint_type() == ChainJointType::kRevolute) {
      delta = angle_distance(rhs[i][0], lhs[i][0]);
    }
    distance += delta * delta;
  }
  return sqrtf(distance);
}

template <int N, int MaxCandidates = 12>
inline void deduplicate_seed_candidates(const SerialChain<N>& chain,
                                        SeedCandidateSet<N, MaxCandidates>* set) {
  if (set == nullptr) {
    return;
  }

  for (uint16_t i = 0; i < set->count; ++i) {
    if (!set->items[i].valid) {
      continue;
    }
    for (uint16_t j = static_cast<uint16_t>(i + 1U); j < set->count; ++j) {
      if (!set->items[j].valid) {
        continue;
      }
      if (configuration_distance(chain, set->items[i].q, set->items[j].q) <=
          1.0e-5f) {
        set->items[j].valid = false;
      }
    }
  }
}

inline Scalar seed_source_bias(SeedSource source) {
  switch (source) {
    case SeedSource::kCurrentState:
      return -0.30f;
    case SeedSource::kReference:
      return -0.20f;
    case SeedSource::kUserSeed:
      return -0.10f;
    case SeedSource::kAnalyticSolution:
      return -0.15f;
    default:
      return 0.0f;
  }
}

template <int N>
inline Scalar estimate_limit_margin_score(const SerialChain<N>& chain,
                                          const JointVec<N>& q) {
  Scalar score = 0.0f;
  for (uint16_t i = 0; i < N; ++i) {
    const Link& link = chain.link(i);
    if (!link.limit_enabled()) {
      continue;
    }

    const Scalar lower_margin = q[i][0] - link.lower_limit();
    const Scalar upper_margin = link.upper_limit() - q[i][0];
    score += clamp_scalar((lower_margin < upper_margin) ? lower_margin
                                                        : upper_margin,
                          0.0f, 1.0e6f);
  }
  return score;
}

template <int N, int MaxCandidates = 12>
inline void rank_seed_candidates(const SerialChain<N>& chain,
                                 const JointVec<N>* current,
                                 bool has_current,
                                 const JointVec<N>* reference,
                                 bool has_reference,
                                 const SeedRankingWeights& weights,
                                 SeedCandidateSet<N, MaxCandidates>* set) {
  if (set == nullptr) {
    return;
  }

  for (uint16_t i = 0; i < set->count; ++i) {
    SeedCandidate<N>& item = set->items[i];
    if (!item.valid) {
      item.score = 1.0e30f;
      continue;
    }

    Scalar score = 0.0f;
    if (has_current && current != nullptr) {
      score += weights.distance_to_current *
               configuration_distance(chain, item.q, *current);
    }
    if (has_reference && reference != nullptr) {
      score += weights.distance_to_reference *
               configuration_distance(chain, item.q, *reference);
    }

    score -= weights.limit_margin_reward *
             estimate_limit_margin_score(chain, item.q);
    score += weights.source_bias * seed_source_bias(item.source);
    item.score = score;
  }

  for (uint16_t i = 0; i < set->count; ++i) {
    uint16_t best = i;
    for (uint16_t j = static_cast<uint16_t>(i + 1U); j < set->count; ++j) {
      if (set->items[j].score < set->items[best].score) {
        best = j;
      }
    }
    if (best != i) {
      const SeedCandidate<N> tmp = set->items[i];
      set->items[i] = set->items[best];
      set->items[best] = tmp;
    }
  }
}

}  // namespace solver
}  // namespace arm_lib

#endif
