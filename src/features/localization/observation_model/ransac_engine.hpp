#pragma once

#include <algorithm>
#include <optional>
#include <utility>

namespace ad::localization::observation_model::util {

template <typename Candidate> class RansacEngine {
public:
  template <typename SampleGenerator, typename CandidateEvaluator, typename CandidateComparator>
  static auto run(const int maxIterations, SampleGenerator &&generateSample,
                  CandidateEvaluator &&evaluateSample, CandidateComparator &&isBetter)
      -> std::optional<Candidate> {
    auto best = std::optional<Candidate>{};
    const auto iterations = std::max(1, maxIterations);

    for (int iteration = 0; iteration < iterations; ++iteration) {
      const auto sample = generateSample();
      if (!sample) {
        continue;
      }

      auto candidate = evaluateSample(*sample);
      if (!candidate) {
        continue;
      }

      if (!best || isBetter(*candidate, *best)) {
        best.emplace(std::move(*candidate));
      }
    }

    return best;
  }
};

} // namespace ad::localization::observation_model::util
