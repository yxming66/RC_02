#ifndef ARM_LIB_KINEMATICS_IK_SOLUTION_SCORING_H
#define ARM_LIB_KINEMATICS_IK_SOLUTION_SCORING_H

#include "../core/arm_limits.h"
#include "../solver/seed_policy.h"
#include "ik_redundancy.h"

namespace mr::robotics::arm {
namespace kinematics {

struct IkCandidateScoreWeights {
  Scalar max_joint_delta;
  Scalar squared_joint_delta_sum;
  Scalar limit_violation;
  Scalar limit_margin_reward;
  Scalar wrist_flip;
  Scalar config_switch;
  Scalar singularity;
  Scalar singularity_threshold;

  IkCandidateScoreWeights()
      : max_joint_delta(1.0f),
        squared_joint_delta_sum(0.25f),
        limit_violation(1000.0f),
        limit_margin_reward(0.05f),
        wrist_flip(0.35f),
        config_switch(0.15f),
        singularity(0.20f),
        singularity_threshold(1.0e-3f) {}
};

struct IkCandidateScore {
  Scalar total;
  Scalar max_joint_delta;
  Scalar squared_joint_delta_sum;
  Scalar limit_violation;
  Scalar limit_margin;
  Scalar wrist_flip_penalty;
  Scalar config_switch_penalty;
  Scalar singularity_metric;
  Scalar singularity_penalty;
  bool within_limits;

  IkCandidateScore()
      : total(0.0f),
        max_joint_delta(0.0f),
        squared_joint_delta_sum(0.0f),
        limit_violation(0.0f),
        limit_margin(0.0f),
        wrist_flip_penalty(0.0f),
        config_switch_penalty(0.0f),
        singularity_metric(0.0f),
        singularity_penalty(0.0f),
        within_limits(true) {}
};

template <int N>
inline Scalar ik_joint_delta(const SerialChain<N>& chain,
                             const JointVec<N>& from,
                             const JointVec<N>& to,
                             uint16_t index) {
  Scalar delta = to[index][0] - from[index][0];
  if (chain.link(index).joint_type() == ChainJointType::kRevolute) {
    delta = angle_distance(from[index][0], to[index][0]);
  }
  return delta;
}

template <int N>
inline Scalar max_ik_joint_delta(const SerialChain<N>& chain,
                                 const JointVec<N>& from,
                                 const JointVec<N>& to) {
  Scalar max_delta = 0.0f;
  for (uint16_t i = 0; i < N; ++i) {
    const Scalar delta = abs_scalar(ik_joint_delta(chain, from, to, i));
    if (delta > max_delta) {
      max_delta = delta;
    }
  }
  return max_delta;
}

template <int N>
inline bool ik_solution_step_within_limit(const SerialChain<N>& chain,
                                          const JointVec<N>& reference,
                                          const JointVec<N>& q,
                                          Scalar max_joint_step) {
  if (max_joint_step <= ARM_LIB_EPSILON) {
    return true;
  }
  return max_ik_joint_delta(chain, reference, q) <=
         max_joint_step + ARM_LIB_EPSILON;
}

template <int N>
inline Scalar ik_limit_margin_sum(const JointLimits<N>& limits,
                                  const JointVec<N>& q) {
  Scalar margin = 0.0f;
  for (uint16_t i = 0; i < N; ++i) {
    if (!limits.enabled[i]) {
      continue;
    }
    const Scalar lower_margin = q[i][0] - limits.lower[i][0];
    const Scalar upper_margin = limits.upper[i][0] - q[i][0];
    const Scalar joint_margin =
        (lower_margin < upper_margin) ? lower_margin : upper_margin;
    margin += clamp_scalar(joint_margin, 0.0f, 1.0e6f);
  }
  return margin;
}

template <int N>
inline Scalar ik_wrist_flip_penalty(const SerialChain<N>& chain,
                                    const JointVec<N>& q,
                                    const JointVec<N>& reference,
                                    bool has_reference) {
  if (!has_reference || N < 6) {
    return 0.0f;
  }

  Scalar penalty = 0.0f;
  for (uint16_t i = 3U; i < 6U; ++i) {
    const Scalar delta = abs_scalar(ik_joint_delta(chain, reference, q, i));
    if (delta > (0.5f * ARM_LIB_PI)) {
      penalty += (delta - 0.5f * ARM_LIB_PI);
    }
  }
  return penalty;
}

template <int N>
inline Scalar ik_config_switch_penalty(const SerialChain<N>& chain,
                                       const JointVec<N>& q,
                                       const JointVec<N>& reference,
                                       bool has_reference) {
  if (!has_reference) {
    return 0.0f;
  }

  Scalar penalty = 0.0f;
  for (uint16_t i = 0; i < N; ++i) {
    if (chain.link(i).joint_type() != ChainJointType::kRevolute) {
      continue;
    }
    const Scalar ref = reference[i][0];
    const Scalar cur = q[i][0];
    if ((ref > 0.25f && cur < -0.25f) ||
        (ref < -0.25f && cur > 0.25f)) {
      penalty += 1.0f;
    }
  }
  return penalty;
}

template <int N>
inline IkCandidateScore score_ik_candidate(
    const SerialChain<N>& chain,
    const JointVec<N>& q,
    const JointVec<N>& reference,
    bool has_reference,
    const JointLimits<N>& limits,
    const IkCandidateScoreWeights& weights = IkCandidateScoreWeights()) {
  IkCandidateScore score;

  if (has_reference) {
    for (uint16_t i = 0; i < N; ++i) {
      const Scalar delta = ik_joint_delta(chain, reference, q, i);
      const Scalar abs_delta = abs_scalar(delta);
      if (abs_delta > score.max_joint_delta) {
        score.max_joint_delta = abs_delta;
      }
      score.squared_joint_delta_sum += delta * delta;
    }
  } else {
    score.squared_joint_delta_sum = q.norm() * q.norm();
    score.max_joint_delta = q.norm();
  }

  score.limit_violation = total_limit_violation(limits, q);
  score.within_limits = score.limit_violation <= ARM_LIB_EPSILON;
  score.limit_margin = ik_limit_margin_sum(limits, q);
  score.wrist_flip_penalty =
      ik_wrist_flip_penalty(chain, q, reference, has_reference);
  score.config_switch_penalty =
      ik_config_switch_penalty(chain, q, reference, has_reference);

  score.singularity_metric = singularity_metric(jacobian(chain, q));
  if (weights.singularity_threshold > ARM_LIB_EPSILON &&
      score.singularity_metric < weights.singularity_threshold) {
    score.singularity_penalty =
        (weights.singularity_threshold - score.singularity_metric) /
        weights.singularity_threshold;
  }

  score.total =
      weights.max_joint_delta * score.max_joint_delta +
      weights.squared_joint_delta_sum * score.squared_joint_delta_sum +
      weights.limit_violation * score.limit_violation -
      weights.limit_margin_reward * score.limit_margin +
      weights.wrist_flip * score.wrist_flip_penalty +
      weights.config_switch * score.config_switch_penalty +
      weights.singularity * score.singularity_penalty;
  return score;
}

inline IkCandidateScoreWeights make_ik_candidate_score_weights(
    Scalar reference_weight,
    Scalar singularity_threshold) {
  IkCandidateScoreWeights weights;
  weights.max_joint_delta *= reference_weight;
  weights.squared_joint_delta_sum *= reference_weight;
  weights.singularity_threshold = singularity_threshold;
  return weights;
}

}  // namespace kinematics
}  // namespace mr::robotics::arm

#endif
