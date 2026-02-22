#include "observation_diagnostics_sink.hpp"

#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>

namespace ad::localization::observation_diagnostics_sink {

auto NullRansacLineAssociationDiagnosticsSink::append(
    const RansacLineAssociationUpdateRecord &record) -> void {
  static_cast<void>(record);
}

CsvRansacLineAssociationDiagnosticsSink::CsvRansacLineAssociationDiagnosticsSink(
    std::string outputPath)
    : outputPath_(std::move(outputPath)) {}

auto CsvRansacLineAssociationDiagnosticsSink::append(
    const RansacLineAssociationUpdateRecord &record) -> void {
  static std::mutex mutex;
  const auto lock = std::lock_guard<std::mutex>{mutex};

  std::error_code error;
  const auto parentPath = std::filesystem::path{outputPath_}.parent_path();
  std::filesystem::create_directories(parentPath, error);

  const auto needHeader = !std::filesystem::exists(outputPath_);
  auto stream = std::ofstream(outputPath_, std::ios::app);
  if (!stream.is_open()) {
    return;
  }

  if (needHeader) {
    stream << "scan_point_count,scan_line_count,candidate_pair_count,ransac_inlier_count,"
              "obs_candidates,obs_gate_passed,final_observation_count,min_observations,"
              "reason,ransac_config_valid,ransac_iterations_requested,"
              "ransac_duplicate_sample_rejects,ransac_hypothesis_rejects,"
              "ransac_min_inlier_rejects,ransac_ratio_rejects,ransac_pose_prior_rejects,"
              "ransac_residual_rejects,ransac_accepted_hypotheses,"
              "ransac_best_inlier_count\n";
  }

  stream << record.scanPointCount << ',' << record.scanLineCount << ',' << record.candidatePairCount
         << ',' << record.ransacInlierCount << ',' << record.observationCandidates << ','
         << record.observationGatePassed << ',' << record.finalObservationCount << ','
         << record.minObservations << ',' << record.reason << ','
         << (record.ransacDiagnostics.configurationValid ? 1 : 0) << ','
         << record.ransacDiagnostics.iterationsRequested << ','
         << record.ransacDiagnostics.duplicateSampleRejects << ','
         << record.ransacDiagnostics.hypothesisRejects << ','
         << record.ransacDiagnostics.minInlierRejects << ','
         << record.ransacDiagnostics.ratioRejects << ','
         << record.ransacDiagnostics.posePriorRejects << ','
         << record.ransacDiagnostics.residualRejects << ','
         << record.ransacDiagnostics.acceptedHypotheses << ','
         << record.ransacDiagnostics.bestInlierCount << '\n';
}

auto defaultRansacLineAssociationDiagnosticsSink()
    -> std::shared_ptr<IRansacLineAssociationDiagnosticsSink> {
  static const auto sink = std::make_shared<CsvRansacLineAssociationDiagnosticsSink>(
      "test/localization/logs/ransac_line_association_update_debug.csv");
  return sink;
}

} // namespace ad::localization::observation_diagnostics_sink
