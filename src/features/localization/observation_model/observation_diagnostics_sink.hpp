#pragma once

#include "ransac_core.hpp"

#include <cstddef>
#include <memory>
#include <string>

namespace ad::localization::observation_diagnostics_sink {

struct RansacLineAssociationUpdateRecord {
  std::size_t scanPointCount = 0U;
  std::size_t scanLineCount = 0U;
  std::size_t candidatePairCount = 0U;
  std::size_t ransacInlierCount = 0U;
  int observationCandidates = 0;
  int observationGatePassed = 0;
  std::size_t finalObservationCount = 0U;
  std::size_t minObservations = 0U;
  ransac::LinePairRansacDiagnostics ransacDiagnostics{};
  std::string reason;
};

class IRansacLineAssociationDiagnosticsSink {
public:
  virtual ~IRansacLineAssociationDiagnosticsSink() = default;
  virtual auto append(const RansacLineAssociationUpdateRecord &record) -> void = 0;
};

class NullRansacLineAssociationDiagnosticsSink final
    : public IRansacLineAssociationDiagnosticsSink {
public:
  auto append(const RansacLineAssociationUpdateRecord &record) -> void override;
};

class CsvRansacLineAssociationDiagnosticsSink final : public IRansacLineAssociationDiagnosticsSink {
public:
  explicit CsvRansacLineAssociationDiagnosticsSink(std::string outputPath);
  auto append(const RansacLineAssociationUpdateRecord &record) -> void override;

private:
  std::string outputPath_;
};

[[nodiscard]] auto defaultRansacLineAssociationDiagnosticsSink()
    -> std::shared_ptr<IRansacLineAssociationDiagnosticsSink>;

} // namespace ad::localization::observation_diagnostics_sink
