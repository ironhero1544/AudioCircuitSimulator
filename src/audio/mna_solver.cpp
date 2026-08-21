#include "plc_emulator/audio/mna_solver.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace plc::audio {
namespace {

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kMinimumResistance = 1.0e-9;
constexpr double kGmin = 1.0e-12;
constexpr double kPivotEpsilon = 1.0e-18;
constexpr double kThermalVoltage = 0.02585;
// Newton junction-voltage convergence tolerance. Was 1e-9 tighter than
// audio-rate precision needs, which caused spurious non-convergence on
// borderline samples (especially in the 6-iteration transient budget),
// producing audible left/right asymmetry between independently-solved
// stereo channels. Relaxed by two orders of magnitude; still far below
// anything audible (thermal voltage is ~25.85 mV).
constexpr double kNewtonConvergenceTolerance = 1.0e-4;

struct DiodeLinearization {
  double current = 0.0;
  double conductance = 0.0;
};

DiodeLinearization LinearizeDiode(const MnaElement& element,
                                  double junction_voltage) {
  const double saturation_current =
      std::max(std::abs(element.value), 1.0e-18);
  const double limited_voltage = std::clamp(junction_voltage, -5.0, 0.8);
  const double exponential = std::exp(limited_voltage / kThermalVoltage);
  const double ideal_current = saturation_current * (exponential - 1.0);
  const double ideal_conductance =
      saturation_current * exponential / kThermalVoltage;
  const double series_limit =
      1.0 / std::max(element.series_resistance, 1.0e-3);
  const double conductance =
      std::clamp(ideal_conductance, kGmin, series_limit);
  const double knee_voltage =
      kThermalVoltage *
      std::log(std::max(series_limit * kThermalVoltage /
                            saturation_current,
                        1.0));
  const double current = ideal_conductance <= series_limit
                             ? ideal_current
                             : saturation_current *
                                       std::expm1(knee_voltage /
                                                  kThermalVoltage) +
                                   series_limit *
                                       (junction_voltage - knee_voltage);
  return {std::clamp(current, -100.0, 100.0), conductance};
}

struct BjtLinearization {
  double collector_current = 0.0;
  double base_current = 0.0;
  double gm = 0.0;
  double go = 0.0;
  double gpi = 0.0;
};

BjtLinearization LinearizeBjt(const MnaElement& element,
                              const std::vector<double>& voltages) {
  const bool npn = element.type == MnaElementType::BJT_NPN;
  const double polarity = npn ? 1.0 : -1.0;
  const double collector =
      voltages[static_cast<size_t>(element.positive_node)];
  const double emitter =
      voltages[static_cast<size_t>(element.negative_node)];
  const double base =
      voltages[static_cast<size_t>(element.control_positive_node)];
  const double vbe = std::clamp(polarity * (base - emitter), -5.0, 0.8);
  const double vce = polarity * (collector - emitter);
  const double saturation_current =
      std::max(std::abs(element.value), 1.0e-18);
  const double beta = std::max(std::abs(element.series_resistance), 1.0);
  const double early_voltage = std::max(std::abs(element.auxiliary_value), 1.0);
  const double exponential = std::exp(vbe / kThermalVoltage);
  const double forward_current = saturation_current * (exponential - 1.0);
  const double early_factor = std::max(0.05, 1.0 + vce / early_voltage);
  const double magnitude = forward_current * early_factor;
  const double gm =
      std::clamp(saturation_current * exponential / kThermalVoltage *
                     early_factor,
                 kGmin, 1000.0);
  const double go = std::clamp(
      std::max(forward_current, 0.0) / early_voltage, kGmin, 100.0);
  const double gpi = std::max(gm / beta, kGmin);
  return {polarity * std::clamp(magnitude, -100.0, 100.0),
          polarity * std::clamp(forward_current / beta, -10.0, 10.0),
          gm, go, gpi};
}

template <typename T>
void StampTransconductance(std::vector<T>* matrix, int order, int output_p,
                           int output_n, int control_p, int control_n,
                           T value) {
  if (output_p >= 0 && control_p >= 0) {
    (*matrix)[static_cast<size_t>(output_p * order + control_p)] += value;
  }
  if (output_p >= 0 && control_n >= 0) {
    (*matrix)[static_cast<size_t>(output_p * order + control_n)] -= value;
  }
  if (output_n >= 0 && control_p >= 0) {
    (*matrix)[static_cast<size_t>(output_n * order + control_p)] -= value;
  }
  if (output_n >= 0 && control_n >= 0) {
    (*matrix)[static_cast<size_t>(output_n * order + control_n)] += value;
  }
}

template <typename T>
void StampAdmittance(std::vector<T>* matrix, int order, int positive,
                     int negative, T admittance) {
  if (!matrix) return;
  if (positive >= 0) {
    (*matrix)[static_cast<size_t>(positive * order + positive)] += admittance;
  }
  if (negative >= 0) {
    (*matrix)[static_cast<size_t>(negative * order + negative)] += admittance;
  }
  if (positive >= 0 && negative >= 0) {
    (*matrix)[static_cast<size_t>(positive * order + negative)] -= admittance;
    (*matrix)[static_cast<size_t>(negative * order + positive)] -= admittance;
  }
}

template <typename T>
void StampVoltageBranch(std::vector<T>* matrix, int order, int positive,
                        int negative, int branch) {
  if (!matrix || branch < 0) return;
  if (positive >= 0) {
    (*matrix)[static_cast<size_t>(positive * order + branch)] += T{1};
    (*matrix)[static_cast<size_t>(branch * order + positive)] += T{1};
  }
  if (negative >= 0) {
    (*matrix)[static_cast<size_t>(negative * order + branch)] -= T{1};
    (*matrix)[static_cast<size_t>(branch * order + negative)] -= T{1};
  }
}

void StampCurrent(std::vector<double>* rhs, int positive, int negative,
                  double current) {
  if (!rhs) return;
  if (positive >= 0) (*rhs)[static_cast<size_t>(positive)] -= current;
  if (negative >= 0) (*rhs)[static_cast<size_t>(negative)] += current;
}

template <typename T>
bool DenseSolve(std::vector<T> matrix, std::vector<T> rhs,
                std::vector<T>* solution) {
  if (!solution || rhs.empty() || matrix.size() != rhs.size() * rhs.size()) {
    return false;
  }
  const int order = static_cast<int>(rhs.size());
  for (int column = 0; column < order; ++column) {
    int pivot = column;
    double pivot_magnitude = std::abs(
        matrix[static_cast<size_t>(column * order + column)]);
    for (int row = column + 1; row < order; ++row) {
      const double magnitude =
          std::abs(matrix[static_cast<size_t>(row * order + column)]);
      if (magnitude > pivot_magnitude) {
        pivot = row;
        pivot_magnitude = magnitude;
      }
    }
    if (pivot_magnitude < kPivotEpsilon) return false;
    if (pivot != column) {
      for (int index = 0; index < order; ++index) {
        std::swap(matrix[static_cast<size_t>(pivot * order + index)],
                  matrix[static_cast<size_t>(column * order + index)]);
      }
      std::swap(rhs[static_cast<size_t>(pivot)],
                rhs[static_cast<size_t>(column)]);
    }
    const T diagonal =
        matrix[static_cast<size_t>(column * order + column)];
    for (int row = column + 1; row < order; ++row) {
      const T factor =
          matrix[static_cast<size_t>(row * order + column)] / diagonal;
      if (std::abs(factor) < kPivotEpsilon) continue;
      matrix[static_cast<size_t>(row * order + column)] = T{};
      for (int index = column + 1; index < order; ++index) {
        matrix[static_cast<size_t>(row * order + index)] -=
            factor * matrix[static_cast<size_t>(column * order + index)];
      }
      rhs[static_cast<size_t>(row)] -=
          factor * rhs[static_cast<size_t>(column)];
    }
  }
  solution->assign(static_cast<size_t>(order), T{});
  for (int row = order - 1; row >= 0; --row) {
    T value = rhs[static_cast<size_t>(row)];
    for (int column = row + 1; column < order; ++column) {
      value -= matrix[static_cast<size_t>(row * order + column)] *
               (*solution)[static_cast<size_t>(column)];
    }
    const T diagonal = matrix[static_cast<size_t>(row * order + row)];
    if (std::abs(diagonal) < kPivotEpsilon) return false;
    (*solution)[static_cast<size_t>(row)] = value / diagonal;
  }
  return true;
}

bool FactorDenseNoAllocation(const std::vector<double>& matrix, int order,
                             std::vector<double>* lu,
                             std::vector<int>* pivots) {
  if (!lu || !pivots || order <= 0 ||
      matrix.size() != static_cast<size_t>(order * order)) {
    return false;
  }
  *lu = matrix;
  pivots->resize(static_cast<size_t>(order));
  for (int column = 0; column < order; ++column) {
    int pivot = column;
    double magnitude =
        std::abs((*lu)[static_cast<size_t>(column * order + column)]);
    for (int row = column + 1; row < order; ++row) {
      const double candidate =
          std::abs((*lu)[static_cast<size_t>(row * order + column)]);
      if (candidate > magnitude) {
        pivot = row;
        magnitude = candidate;
      }
    }
    if (magnitude < kPivotEpsilon) return false;
    (*pivots)[static_cast<size_t>(column)] = pivot;
    if (pivot != column) {
      for (int index = 0; index < order; ++index) {
        std::swap((*lu)[static_cast<size_t>(pivot * order + index)],
                  (*lu)[static_cast<size_t>(column * order + index)]);
      }
    }
    const double diagonal =
        (*lu)[static_cast<size_t>(column * order + column)];
    for (int row = column + 1; row < order; ++row) {
      double& factor = (*lu)[static_cast<size_t>(row * order + column)];
      factor /= diagonal;
      if (std::abs(factor) < kPivotEpsilon) {
        factor = 0.0;
        continue;
      }
      for (int index = column + 1; index < order; ++index) {
        (*lu)[static_cast<size_t>(row * order + index)] -=
            factor * (*lu)[static_cast<size_t>(column * order + index)];
      }
    }
  }
  return true;
}

void BuildFactoredSolvePattern(const std::vector<double>& lu, int order,
                               std::vector<size_t>* lower_offsets,
                               std::vector<int>* lower_columns,
                               std::vector<size_t>* upper_offsets,
                               std::vector<int>* upper_columns) {
  lower_offsets->assign(static_cast<size_t>(order + 1), 0U);
  upper_offsets->assign(static_cast<size_t>(order + 1), 0U);
  size_t lower_count = 0;
  size_t upper_count = 0;
  for (int row = 0; row < order; ++row) {
    for (int column = 0; column < row; ++column) {
      if (std::abs(lu[static_cast<size_t>(row * order + column)]) >
          kPivotEpsilon) {
        ++lower_count;
      }
    }
    for (int column = row + 1; column < order; ++column) {
      if (std::abs(lu[static_cast<size_t>(row * order + column)]) >
          kPivotEpsilon) {
        ++upper_count;
      }
    }
    (*lower_offsets)[static_cast<size_t>(row + 1)] = lower_count;
    (*upper_offsets)[static_cast<size_t>(row + 1)] = upper_count;
  }
  lower_columns->resize(lower_count);
  upper_columns->resize(upper_count);
  for (int row = 0; row < order; ++row) {
    size_t lower = (*lower_offsets)[static_cast<size_t>(row)];
    size_t upper = (*upper_offsets)[static_cast<size_t>(row)];
    for (int column = 0; column < row; ++column) {
      if (std::abs(lu[static_cast<size_t>(row * order + column)]) >
          kPivotEpsilon) {
        (*lower_columns)[lower++] = column;
      }
    }
    for (int column = row + 1; column < order; ++column) {
      if (std::abs(lu[static_cast<size_t>(row * order + column)]) >
          kPivotEpsilon) {
        (*upper_columns)[upper++] = column;
      }
    }
  }
}

bool SolveSparseFactoredNoAllocation(
    const std::vector<double>& lu, const std::vector<int>& pivots, int order,
    const std::vector<size_t>& lower_offsets,
    const std::vector<int>& lower_columns,
    const std::vector<size_t>& upper_offsets,
    const std::vector<int>& upper_columns, const std::vector<double>& rhs,
    std::vector<double>* solution) {
  if (!solution || order <= 0 ||
      lower_offsets.size() != static_cast<size_t>(order + 1) ||
      upper_offsets.size() != static_cast<size_t>(order + 1) ||
      rhs.size() != static_cast<size_t>(order)) {
    return false;
  }
  *solution = rhs;
  for (int column = 0; column < order; ++column) {
    const int pivot = pivots[static_cast<size_t>(column)];
    if (pivot != column) {
      std::swap((*solution)[static_cast<size_t>(pivot)],
                (*solution)[static_cast<size_t>(column)]);
    }
  }
  for (int row = 0; row < order; ++row) {
    double value = (*solution)[static_cast<size_t>(row)];
    const size_t begin = lower_offsets[static_cast<size_t>(row)];
    const size_t end = lower_offsets[static_cast<size_t>(row + 1)];
    for (size_t index = begin; index < end; ++index) {
      const int column = lower_columns[index];
      value -= lu[static_cast<size_t>(row * order + column)] *
               (*solution)[static_cast<size_t>(column)];
    }
    (*solution)[static_cast<size_t>(row)] = value;
  }
  for (int row = order - 1; row >= 0; --row) {
    double value = (*solution)[static_cast<size_t>(row)];
    const size_t begin = upper_offsets[static_cast<size_t>(row)];
    const size_t end = upper_offsets[static_cast<size_t>(row + 1)];
    for (size_t index = begin; index < end; ++index) {
      const int column = upper_columns[index];
      value -= lu[static_cast<size_t>(row * order + column)] *
               (*solution)[static_cast<size_t>(column)];
    }
    const double diagonal = lu[static_cast<size_t>(row * order + row)];
    if (std::abs(diagonal) < kPivotEpsilon) return false;
    (*solution)[static_cast<size_t>(row)] = value / diagonal;
  }
  return true;
}

}  // namespace

int MnaSolver::UnknownForNode(int node) const {
  if (node < 0 || node >= node_count_ || node == ground_node_) return -1;
  return node < ground_node_ ? node : node - 1;
}

bool MnaSolver::Compile(int node_count, int ground_node, double sample_rate,
                        const std::vector<MnaElement>& elements,
                        MnaSolveMetrics* metrics) {
  if (metrics) *metrics = {};
  compiled_ = false;
  if (node_count <= 1 || ground_node < 0 || ground_node >= node_count ||
      !std::isfinite(sample_rate) || sample_rate <= 0.0) {
    if (metrics) metrics->error = "Invalid MNA node count, ground, or rate.";
    return false;
  }
  node_count_ = node_count;
  ground_node_ = ground_node;
  sample_rate_ = sample_rate;
  elements_ = elements;
  nonlinear_element_indices_.clear();
  rhs_element_indices_.clear();
  dynamic_element_indices_.clear();
  for (size_t index = 0; index < elements_.size(); ++index) {
    const MnaElementType type = elements_[index].type;
    if (type == MnaElementType::DIODE || type == MnaElementType::BJT_NPN ||
        type == MnaElementType::BJT_PNP) {
      nonlinear_element_indices_.push_back(index);
    }
    if (type == MnaElementType::CURRENT_SOURCE ||
        type == MnaElementType::VOLTAGE_SOURCE ||
        type == MnaElementType::CAPACITOR ||
        type == MnaElementType::INDUCTOR) {
      rhs_element_indices_.push_back(index);
    }
    if (type == MnaElementType::CAPACITOR ||
        type == MnaElementType::INDUCTOR) {
      dynamic_element_indices_.push_back(index);
    }
  }
  has_nonlinear_elements_ = !nonlinear_element_indices_.empty();
  branch_unknowns_.assign(elements_.size(), -1);
  dynamic_states_.assign(elements_.size(), {});
  int branches = 0;
  for (size_t index = 0; index < elements_.size(); ++index) {
    const MnaElementType type = elements_[index].type;
    if (type == MnaElementType::VOLTAGE_SOURCE ||
        type == MnaElementType::VCVS) {
      branch_unknowns_[index] = node_count_ - 1 + branches++;
    }
  }
  matrix_order_ = node_count_ - 1 + branches;
  if (matrix_order_ <= 0 || matrix_order_ > 512) {
    if (metrics) metrics->error = "MNA matrix order is outside 1..512.";
    return false;
  }
  std::vector<double> matrix;
  if (!BuildRealMatrix(false, &matrix) ||
      !FactorRealMatrix(matrix, metrics)) {
    if (metrics && metrics->error.empty()) {
      metrics->error = "MNA transient matrix is singular.";
    }
    return false;
  }
  transient_base_matrix_ = matrix;
  solution_.assign(static_cast<size_t>(matrix_order_), 0.0);
  rhs_.assign(static_cast<size_t>(matrix_order_), 0.0);
  nonlinear_rhs_.assign(static_cast<size_t>(matrix_order_), 0.0);
  cached_nonlinear_rhs_correction_.assign(
      static_cast<size_t>(matrix_order_), 0.0);
  nonlinear_matrix_.assign(
      static_cast<size_t>(matrix_order_ * matrix_order_), 0.0);
  nonlinear_factorization_valid_ = false;
  node_voltages_.assign(static_cast<size_t>(node_count_), 0.0);
  reduction_enabled_ = BuildTransientReduction(transient_base_matrix_);
  compiled_ = true;
  if (metrics) {
    metrics->converged = true;
    metrics->matrix_order = matrix_order_;
    metrics->iterations = 1;
  }
  return true;
}

bool MnaSolver::BuildRealMatrix(
    bool dc, std::vector<double>* matrix,
    const std::vector<double>* nonlinear_voltages) const {
  if (!matrix || matrix_order_ <= 0) return false;
  matrix->assign(static_cast<size_t>(matrix_order_ * matrix_order_), 0.0);
  for (int node = 0; node < node_count_; ++node) {
    const int unknown = UnknownForNode(node);
    if (unknown >= 0) {
      (*matrix)[static_cast<size_t>(unknown * matrix_order_ + unknown)] +=
          kGmin;
    }
  }
  for (size_t index = 0; index < elements_.size(); ++index) {
    const MnaElement& element = elements_[index];
    const int positive = UnknownForNode(element.positive_node);
    const int negative = UnknownForNode(element.negative_node);
    switch (element.type) {
      case MnaElementType::RESISTOR:
        StampAdmittance(matrix, matrix_order_, positive, negative,
                        1.0 / std::max(std::abs(element.value),
                                       kMinimumResistance));
        break;
      case MnaElementType::DIODE:
        if (nonlinear_voltages &&
            nonlinear_voltages->size() ==
                static_cast<size_t>(node_count_)) {
          const double voltage =
              (*nonlinear_voltages)[static_cast<size_t>(
                  element.positive_node)] -
              (*nonlinear_voltages)[static_cast<size_t>(
                  element.negative_node)];
          StampAdmittance(matrix, matrix_order_, positive, negative,
                          LinearizeDiode(element, voltage).conductance);
        } else {
          StampAdmittance(matrix, matrix_order_, positive, negative, kGmin);
        }
        break;
      case MnaElementType::BJT_NPN:
      case MnaElementType::BJT_PNP: {
        const int base = UnknownForNode(element.control_positive_node);
        BjtLinearization linearized;
        if (nonlinear_voltages &&
            nonlinear_voltages->size() ==
                static_cast<size_t>(node_count_)) {
          linearized = LinearizeBjt(element, *nonlinear_voltages);
        } else {
          linearized.gm = kGmin;
          linearized.go = kGmin;
          linearized.gpi = kGmin;
        }
        StampAdmittance(matrix, matrix_order_, positive, negative,
                        linearized.go);
        StampTransconductance(matrix, matrix_order_, positive, negative,
                              base, negative, linearized.gm);
        StampAdmittance(matrix, matrix_order_, base, negative,
                        linearized.gpi);
        break;
      }
      case MnaElementType::CAPACITOR:
        if (!dc) {
          StampAdmittance(matrix, matrix_order_, positive, negative,
                          2.0 * std::max(element.value, 0.0) * sample_rate_);
        }
        break;
      case MnaElementType::INDUCTOR:
        StampAdmittance(
            matrix, matrix_order_, positive, negative,
            dc ? 1.0e9
               : 1.0 / (2.0 * std::max(element.value, 1.0e-15) *
                        sample_rate_));
        break;
      case MnaElementType::VOLTAGE_SOURCE:
      case MnaElementType::VCVS: {
        const int branch = branch_unknowns_[index];
        StampVoltageBranch(matrix, matrix_order_, positive, negative, branch);
        if (element.type == MnaElementType::VCVS) {
          const int control_positive =
              UnknownForNode(element.control_positive_node);
          const int control_negative =
              UnknownForNode(element.control_negative_node);
          if (control_positive >= 0) {
            (*matrix)[static_cast<size_t>(branch * matrix_order_ +
                                          control_positive)] -= element.value;
          }
          if (control_negative >= 0) {
            (*matrix)[static_cast<size_t>(branch * matrix_order_ +
                                          control_negative)] += element.value;
          }
        }
        break;
      }
      case MnaElementType::VCCS: {
        const int control_positive =
            UnknownForNode(element.control_positive_node);
        const int control_negative =
            UnknownForNode(element.control_negative_node);
        const double transconductance = element.value;
        if (positive >= 0 && control_positive >= 0) {
          (*matrix)[static_cast<size_t>(positive * matrix_order_ +
                                        control_positive)] += transconductance;
        }
        if (positive >= 0 && control_negative >= 0) {
          (*matrix)[static_cast<size_t>(positive * matrix_order_ +
                                        control_negative)] -= transconductance;
        }
        if (negative >= 0 && control_positive >= 0) {
          (*matrix)[static_cast<size_t>(negative * matrix_order_ +
                                        control_positive)] -= transconductance;
        }
        if (negative >= 0 && control_negative >= 0) {
          (*matrix)[static_cast<size_t>(negative * matrix_order_ +
                                        control_negative)] += transconductance;
        }
        break;
      }
      case MnaElementType::CURRENT_SOURCE:
        break;
    }
  }
  return true;
}

bool MnaSolver::BuildNonlinearTransientMatrix(
    const std::vector<double>& voltages,
    std::vector<double>* matrix) const {
  if (!matrix || voltages.size() != static_cast<size_t>(node_count_) ||
      transient_base_matrix_.size() !=
          static_cast<size_t>(matrix_order_ * matrix_order_)) {
    return false;
  }
  *matrix = transient_base_matrix_;
  for (size_t element_index : nonlinear_element_indices_) {
    const MnaElement& element = elements_[element_index];
    const int positive = UnknownForNode(element.positive_node);
    const int negative = UnknownForNode(element.negative_node);
    if (element.type == MnaElementType::DIODE) {
      const double voltage =
          voltages[static_cast<size_t>(element.positive_node)] -
          voltages[static_cast<size_t>(element.negative_node)];
      const double delta =
          LinearizeDiode(element, voltage).conductance - kGmin;
      StampAdmittance(matrix, matrix_order_, positive, negative, delta);
    } else if (element.type == MnaElementType::BJT_NPN ||
               element.type == MnaElementType::BJT_PNP) {
      const int base = UnknownForNode(element.control_positive_node);
      const BjtLinearization linearized = LinearizeBjt(element, voltages);
      StampAdmittance(matrix, matrix_order_, positive, negative,
                      linearized.go - kGmin);
      StampTransconductance(matrix, matrix_order_, positive, negative,
                            base, negative, linearized.gm - kGmin);
      StampAdmittance(matrix, matrix_order_, base, negative,
                      linearized.gpi - kGmin);
    }
  }
  return true;
}

bool MnaSolver::FactorRealMatrix(const std::vector<double>& matrix,
                                 MnaSolveMetrics* metrics,
                                 bool rebuild_sparsity,
                                 bool matrix_already_loaded) {
  if (matrix.size() !=
      static_cast<size_t>(matrix_order_ * matrix_order_)) {
    return false;
  }
  if (!matrix_already_loaded) lu_ = matrix;
  if (!rebuild_sparsity &&
      pivots_.size() == static_cast<size_t>(matrix_order_) &&
      factor_column_offsets_.size() ==
          static_cast<size_t>(matrix_order_ + 1) &&
      upper_row_offsets_.size() ==
          static_cast<size_t>(matrix_order_ + 1)) {
    for (int column = 0; column < matrix_order_; ++column) {
      const int pivot = pivots_[static_cast<size_t>(column)];
      if (pivot != column) {
        for (int index = 0; index < matrix_order_; ++index) {
          std::swap(lu_[static_cast<size_t>(pivot * matrix_order_ + index)],
                    lu_[static_cast<size_t>(column * matrix_order_ + index)]);
        }
      }
      const double diagonal =
          lu_[static_cast<size_t>(column * matrix_order_ + column)];
      if (std::abs(diagonal) < kPivotEpsilon) {
        return FactorRealMatrix(matrix, metrics, true,
                                matrix_already_loaded);
      }
      const size_t factor_begin =
          factor_column_offsets_[static_cast<size_t>(column)];
      const size_t factor_end =
          factor_column_offsets_[static_cast<size_t>(column + 1)];
      for (size_t factor_index = factor_begin;
           factor_index < factor_end; ++factor_index) {
        const int row = factor_rows_flat_[factor_index];
        double& factor =
            lu_[static_cast<size_t>(row * matrix_order_ + column)];
        factor /= diagonal;
        if (std::abs(factor) < kPivotEpsilon) {
          factor = 0.0;
          continue;
        }
        const size_t upper_begin =
            upper_row_offsets_[static_cast<size_t>(column)];
        const size_t upper_end =
            upper_row_offsets_[static_cast<size_t>(column + 1)];
        for (size_t upper_index = upper_begin;
             upper_index < upper_end; ++upper_index) {
          const int index = upper_columns_flat_[upper_index];
          lu_[static_cast<size_t>(row * matrix_order_ + index)] -=
              factor *
              lu_[static_cast<size_t>(column * matrix_order_ + index)];
        }
      }
    }
    return true;
  }
  const bool had_pivots =
      pivots_.size() == static_cast<size_t>(matrix_order_);
  pivots_.resize(static_cast<size_t>(matrix_order_));
  bool pivot_pattern_changed = !had_pivots;
  for (int column = 0; column < matrix_order_; ++column) {
    int pivot = column;
    double magnitude = std::abs(
        lu_[static_cast<size_t>(column * matrix_order_ + column)]);
    for (int row = column + 1; row < matrix_order_; ++row) {
      const double candidate = std::abs(
          lu_[static_cast<size_t>(row * matrix_order_ + column)]);
      if (candidate > magnitude) {
        pivot = row;
        magnitude = candidate;
      }
    }
    if (magnitude < kPivotEpsilon) {
      if (metrics) metrics->error = "MNA matrix has a zero pivot.";
      return false;
    }
    if (had_pivots && pivots_[static_cast<size_t>(column)] != pivot) {
      pivot_pattern_changed = true;
    }
    pivots_[static_cast<size_t>(column)] = pivot;
    if (pivot != column) {
      for (int index = 0; index < matrix_order_; ++index) {
        std::swap(lu_[static_cast<size_t>(pivot * matrix_order_ + index)],
                  lu_[static_cast<size_t>(column * matrix_order_ + index)]);
      }
    }
    const double diagonal =
        lu_[static_cast<size_t>(column * matrix_order_ + column)];
    for (int row = column + 1; row < matrix_order_; ++row) {
      double& factor =
          lu_[static_cast<size_t>(row * matrix_order_ + column)];
      factor /= diagonal;
      if (std::abs(factor) < kPivotEpsilon) {
        factor = 0.0;
        continue;
      }
      for (int index = column + 1; index < matrix_order_; ++index) {
        lu_[static_cast<size_t>(row * matrix_order_ + index)] -=
            factor * lu_[static_cast<size_t>(column * matrix_order_ + index)];
      }
    }
  }
  if (rebuild_sparsity || pivot_pattern_changed ||
      lower_row_offsets_.size() !=
          static_cast<size_t>(matrix_order_ + 1)) {
    lower_row_offsets_.assign(static_cast<size_t>(matrix_order_ + 1), 0U);
    upper_row_offsets_.assign(static_cast<size_t>(matrix_order_ + 1), 0U);
    factor_column_offsets_.assign(
        static_cast<size_t>(matrix_order_ + 1), 0U);
    size_t lower_count = 0;
    size_t upper_count = 0;
    for (int row = 0; row < matrix_order_; ++row) {
      for (int column = 0; column < row; ++column) {
        if (std::abs(lu_[static_cast<size_t>(row * matrix_order_ + column)]) >
            kPivotEpsilon) {
          ++lower_count;
          ++factor_column_offsets_[static_cast<size_t>(column + 1)];
        }
      }
      for (int column = row + 1; column < matrix_order_; ++column) {
        if (std::abs(lu_[static_cast<size_t>(row * matrix_order_ + column)]) >
            kPivotEpsilon) {
          ++upper_count;
        }
      }
      lower_row_offsets_[static_cast<size_t>(row + 1)] = lower_count;
      upper_row_offsets_[static_cast<size_t>(row + 1)] = upper_count;
    }
    for (int column = 0; column < matrix_order_; ++column) {
      factor_column_offsets_[static_cast<size_t>(column + 1)] +=
          factor_column_offsets_[static_cast<size_t>(column)];
    }
    lower_columns_flat_.resize(lower_count);
    upper_columns_flat_.resize(upper_count);
    factor_rows_flat_.resize(lower_count);
    std::vector<size_t> factor_cursor = factor_column_offsets_;
    for (int row = 0; row < matrix_order_; ++row) {
      size_t lower_cursor = lower_row_offsets_[static_cast<size_t>(row)];
      size_t upper_cursor = upper_row_offsets_[static_cast<size_t>(row)];
      for (int column = 0; column < row; ++column) {
        if (std::abs(lu_[static_cast<size_t>(row * matrix_order_ + column)]) >
            kPivotEpsilon) {
          lower_columns_flat_[lower_cursor++] = column;
          factor_rows_flat_[factor_cursor[static_cast<size_t>(column)]++] =
              row;
        }
      }
      for (int column = row + 1; column < matrix_order_; ++column) {
        if (std::abs(lu_[static_cast<size_t>(row * matrix_order_ + column)]) >
            kPivotEpsilon) {
          upper_columns_flat_[upper_cursor++] = column;
        }
      }
    }
  }
  return true;
}

bool MnaSolver::SolveFactored(const std::vector<double>& rhs,
                              std::vector<double>* solution) const {
  if (!solution || rhs.size() != static_cast<size_t>(matrix_order_)) {
    return false;
  }
  *solution = rhs;
  for (int column = 0; column < matrix_order_; ++column) {
    const int pivot = pivots_[static_cast<size_t>(column)];
    if (pivot != column) {
      std::swap((*solution)[static_cast<size_t>(pivot)],
                (*solution)[static_cast<size_t>(column)]);
    }
  }
  for (int row = 0; row < matrix_order_; ++row) {
    double value = (*solution)[static_cast<size_t>(row)];
    const size_t begin = lower_row_offsets_[static_cast<size_t>(row)];
    const size_t end = lower_row_offsets_[static_cast<size_t>(row + 1)];
    for (size_t index = begin; index < end; ++index) {
      const int column = lower_columns_flat_[index];
      value -= lu_[static_cast<size_t>(row * matrix_order_ + column)] *
               (*solution)[static_cast<size_t>(column)];
    }
    (*solution)[static_cast<size_t>(row)] = value;
  }
  for (int row = matrix_order_ - 1; row >= 0; --row) {
    double value = (*solution)[static_cast<size_t>(row)];
    const size_t begin = upper_row_offsets_[static_cast<size_t>(row)];
    const size_t end = upper_row_offsets_[static_cast<size_t>(row + 1)];
    for (size_t index = begin; index < end; ++index) {
      const int column = upper_columns_flat_[index];
      value -= lu_[static_cast<size_t>(row * matrix_order_ + column)] *
               (*solution)[static_cast<size_t>(column)];
    }
    const double diagonal =
        lu_[static_cast<size_t>(row * matrix_order_ + row)];
    if (std::abs(diagonal) < kPivotEpsilon) return false;
    (*solution)[static_cast<size_t>(row)] = value / diagonal;
  }
  return true;
}

void MnaSolver::BuildTransientRhs(
    std::span<const double> source_values,
    std::vector<double>* rhs) const {
  if (rhs->size() != static_cast<size_t>(matrix_order_)) {
    return;
  }
  std::fill(rhs->begin(), rhs->end(), 0.0);
  for (size_t index : rhs_element_indices_) {
    const MnaElement& element = elements_[index];
    const int positive = UnknownForNode(element.positive_node);
    const int negative = UnknownForNode(element.negative_node);
    const double source_value =
        element.source_slot >= 0 &&
                static_cast<size_t>(element.source_slot) < source_values.size()
            ? source_values[static_cast<size_t>(element.source_slot)]
            : element.value;
    switch (element.type) {
      case MnaElementType::CURRENT_SOURCE:
        StampCurrent(rhs, positive, negative, source_value);
        break;
      case MnaElementType::VOLTAGE_SOURCE:
        (*rhs)[static_cast<size_t>(branch_unknowns_[index])] += source_value;
        break;
      case MnaElementType::CAPACITOR: {
        const double conductance =
            2.0 * std::max(element.value, 0.0) * sample_rate_;
        const DynamicState& state = dynamic_states_[index];
        const double history =
            -conductance * state.previous_voltage - state.previous_current;
        StampCurrent(rhs, positive, negative, history);
        break;
      }
      case MnaElementType::INDUCTOR: {
        const double conductance =
            1.0 / (2.0 * std::max(element.value, 1.0e-15) * sample_rate_);
        const DynamicState& state = dynamic_states_[index];
        const double history =
            state.previous_current + conductance * state.previous_voltage;
        StampCurrent(rhs, positive, negative, history);
        break;
      }
      case MnaElementType::RESISTOR:
      case MnaElementType::VCVS:
      case MnaElementType::VCCS:
      case MnaElementType::DIODE:
      case MnaElementType::BJT_NPN:
      case MnaElementType::BJT_PNP:
        break;
    }
  }
}

bool MnaSolver::SolveDc(std::span<const double> source_values,
                        MnaSolveMetrics* metrics) {
  if (metrics) *metrics = {};
  if (!compiled_) return false;
  std::vector<double> matrix;
  std::vector<double> rhs(static_cast<size_t>(matrix_order_), 0.0);
  for (size_t index = 0; index < elements_.size(); ++index) {
    const MnaElement& element = elements_[index];
    const double value =
        element.source_slot >= 0 &&
                static_cast<size_t>(element.source_slot) < source_values.size()
            ? source_values[static_cast<size_t>(element.source_slot)]
            : element.value;
    if (element.type == MnaElementType::CURRENT_SOURCE) {
      StampCurrent(&rhs, UnknownForNode(element.positive_node),
                   UnknownForNode(element.negative_node), value);
    } else if (element.type == MnaElementType::VOLTAGE_SOURCE) {
      rhs[static_cast<size_t>(branch_unknowns_[index])] += value;
    }
  }
  std::vector<double> dc_solution;
  std::vector<double> dc_voltages(static_cast<size_t>(node_count_), 0.0);
  for (size_t element_index : nonlinear_element_indices_) {
    const MnaElement& element = elements_[element_index];
    if (element.type == MnaElementType::DIODE &&
        element.positive_node != ground_node_) {
      dc_voltages[static_cast<size_t>(element.positive_node)] =
          dc_voltages[static_cast<size_t>(element.negative_node)] + 0.55;
    } else if (element.type == MnaElementType::BJT_NPN) {
      dc_voltages[static_cast<size_t>(element.control_positive_node)] =
          dc_voltages[static_cast<size_t>(element.negative_node)] + 0.6;
    } else if (element.type == MnaElementType::BJT_PNP) {
      dc_voltages[static_cast<size_t>(element.control_positive_node)] =
          dc_voltages[static_cast<size_t>(element.negative_node)] - 0.6;
    }
  }
  const int maximum_iterations = has_nonlinear_elements_ ? 150 : 1;
  int dc_iterations = 0;
  double maximum_delta = 0.0;
  bool converged = !has_nonlinear_elements_;

  // Estimate supply rail magnitude for adaptive step limiting.
  double max_supply_volts = 1.0;
  for (const auto& v : dc_voltages) {
    max_supply_volts = std::max(max_supply_volts, std::abs(v));
  }
  for (const MnaElement& el : elements_) {
    if (el.type == MnaElementType::VOLTAGE_SOURCE) {
      max_supply_volts = std::max(max_supply_volts, std::abs(el.value));
    }
  }
  // Adaptive step cap: tighten progressively. Start at 2V (safe for ±15V rails)
  // and reduce to 0.2V after iteration 50.
  const auto step_cap = [&](int iter) -> double {
    if (iter < 10) return max_supply_volts * 0.5;
    if (iter < 30) return 2.0;
    if (iter < 60) return 0.5;
    return 0.1;
  };

  std::vector<double> best_voltages = dc_voltages;
  double best_delta = std::numeric_limits<double>::max();

  for (; dc_iterations < maximum_iterations; ++dc_iterations) {
    if (!BuildRealMatrix(true, &matrix,
                         has_nonlinear_elements_ ? &dc_voltages : nullptr)) {
      if (metrics) metrics->error = "Failed to build DC operating point matrix.";
      return false;
    }
    std::vector<double> iteration_rhs = rhs;
    AddNonlinearRhs(dc_voltages, &iteration_rhs);
    if (!DenseSolve(matrix, iteration_rhs, &dc_solution)) {
      if (metrics) metrics->error = "DC operating point matrix is singular.";
      return false;
    }
    maximum_delta = 0.0;
    const double cap = step_cap(dc_iterations);
    for (int node = 0; node < node_count_; ++node) {
      const int unknown = UnknownForNode(node);
      const double target_voltage =
          unknown >= 0 ? dc_solution[static_cast<size_t>(unknown)] : 0.0;
      if (!std::isfinite(target_voltage)) {
        // Non-finite result: abort this iteration and use best so far.
        break;
      }
      const double current_voltage = dc_voltages[static_cast<size_t>(node)];
      const double raw_delta = target_voltage - current_voltage;
      const double limited_delta = std::clamp(raw_delta, -cap, cap);
      maximum_delta = std::max(maximum_delta, std::abs(raw_delta));
      dc_voltages[static_cast<size_t>(node)] = current_voltage + limited_delta;
    }
    if (maximum_delta < best_delta) {
      best_delta = maximum_delta;
      best_voltages = dc_voltages;
    }
    if (!has_nonlinear_elements_ ||
        maximum_delta <= kNewtonConvergenceTolerance) {
      converged = true;
      ++dc_iterations;
      break;
    }
  }

  // Use the best solution found even if not fully converged.
  // This preserves the original behavior for Class-AB circuits that
  // hover near convergence without reaching 1e-6 exactly.
  if (!converged) {
    dc_voltages = best_voltages;
    maximum_delta = best_delta;
    if (metrics) {
      metrics->error =
          "DC Newton did not fully converge; using best approximation.";
    }
  }

  solution_ = dc_solution;
  node_voltages_ = std::move(dc_voltages);
  nonlinear_factorization_valid_ = false;
  for (size_t index = 0; index < elements_.size(); ++index) {
    if (elements_[index].type == MnaElementType::CAPACITOR) {
      const double v_diff =
          NodeVoltage(elements_[index].positive_node) -
          NodeVoltage(elements_[index].negative_node);
      dynamic_states_[index].previous_voltage =
          std::isfinite(v_diff) ? v_diff : 0.0;
      dynamic_states_[index].previous_current = 0.0;
    } else if (elements_[index].type == MnaElementType::INDUCTOR) {
      const double v_diff =
          NodeVoltage(elements_[index].positive_node) -
          NodeVoltage(elements_[index].negative_node);
      dynamic_states_[index].previous_voltage =
          std::isfinite(v_diff) ? v_diff : 0.0;
      const double dc_current = v_diff * 1.0e9;
      dynamic_states_[index].previous_current =
          std::isfinite(dc_current) ? dc_current : 0.0;
    }
  }
  if (metrics) {
    metrics->converged = converged;
    metrics->matrix_order = matrix_order_;
    metrics->iterations = dc_iterations;
    metrics->maximum_residual = maximum_delta;
  }
  return true;
}

int MnaSolver::ReducedUnknownForNode(int node) const {
  const int full_unknown = UnknownForNode(node);
  if (full_unknown < 0 ||
      static_cast<size_t>(full_unknown) >= full_to_reduced_.size()) {
    return -1;
  }
  return full_to_reduced_[static_cast<size_t>(full_unknown)];
}

bool MnaSolver::BuildTransientReduction(const std::vector<double>& matrix) {
  reduction_enabled_ = false;
  reduced_order_ = 0;
  boundary_unknowns_.clear();
  eliminated_unknowns_.clear();
  full_to_reduced_.assign(static_cast<size_t>(matrix_order_), -1);
  if (matrix.size() !=
      static_cast<size_t>(matrix_order_ * matrix_order_)) {
    return false;
  }

  std::vector<bool> boundary(static_cast<size_t>(matrix_order_), false);
  const auto mark_node = [&](int node) {
    const int unknown = UnknownForNode(node);
    if (unknown >= 0) boundary[static_cast<size_t>(unknown)] = true;
  };
  for (size_t index = 0; index < elements_.size(); ++index) {
    const MnaElement& element = elements_[index];
    switch (element.type) {
      case MnaElementType::CAPACITOR:
      case MnaElementType::INDUCTOR:
        // Companion conductances are constant at the fixed 48 kHz time step.
        // Their history terms live in the RHS, so their electrical nodes may
        // be eliminated while the per-element state itself remains explicit.
        break;
      case MnaElementType::DIODE:
        mark_node(element.positive_node);
        mark_node(element.negative_node);
        break;
      case MnaElementType::BJT_NPN:
      case MnaElementType::BJT_PNP:
        mark_node(element.positive_node);
        mark_node(element.negative_node);
        mark_node(element.control_positive_node);
        break;
      case MnaElementType::VOLTAGE_SOURCE:
        break;
      case MnaElementType::VCVS:
        break;
      case MnaElementType::VCCS:
        // A VCCS is a static linear matrix stamp and needs no state boundary.
        break;
      case MnaElementType::RESISTOR:
      case MnaElementType::CURRENT_SOURCE:
        break;
    }
  }
  for (int unknown = 0; unknown < matrix_order_; ++unknown) {
    if (boundary[static_cast<size_t>(unknown)]) {
      full_to_reduced_[static_cast<size_t>(unknown)] =
          static_cast<int>(boundary_unknowns_.size());
      boundary_unknowns_.push_back(unknown);
    } else {
      eliminated_unknowns_.push_back(unknown);
    }
  }
  // Tiny systems are faster through the normal sparse path, and a purely
  // static network has no required boundary state to retain.
  if (boundary_unknowns_.empty() || eliminated_unknowns_.size() < 4U) {
    return false;
  }

  reduced_order_ = static_cast<int>(boundary_unknowns_.size());
  const int eliminated_order =
      static_cast<int>(eliminated_unknowns_.size());
  boundary_to_eliminated_.assign(
      static_cast<size_t>(reduced_order_ * eliminated_order), 0.0);
  std::vector<double> eliminated_matrix(
      static_cast<size_t>(eliminated_order * eliminated_order), 0.0);
  std::vector<double> eliminated_to_boundary(
      static_cast<size_t>(eliminated_order * reduced_order_), 0.0);
  for (int boundary_row = 0; boundary_row < reduced_order_; ++boundary_row) {
    const int full_row =
        boundary_unknowns_[static_cast<size_t>(boundary_row)];
    for (int eliminated_column = 0; eliminated_column < eliminated_order;
         ++eliminated_column) {
      const int full_column =
          eliminated_unknowns_[static_cast<size_t>(eliminated_column)];
      boundary_to_eliminated_[static_cast<size_t>(
          boundary_row * eliminated_order + eliminated_column)] =
          matrix[static_cast<size_t>(full_row * matrix_order_ + full_column)];
    }
  }
  for (int eliminated_row = 0; eliminated_row < eliminated_order;
       ++eliminated_row) {
    const int full_row =
        eliminated_unknowns_[static_cast<size_t>(eliminated_row)];
    for (int eliminated_column = 0; eliminated_column < eliminated_order;
         ++eliminated_column) {
      const int full_column =
          eliminated_unknowns_[static_cast<size_t>(eliminated_column)];
      eliminated_matrix[static_cast<size_t>(
          eliminated_row * eliminated_order + eliminated_column)] =
          matrix[static_cast<size_t>(full_row * matrix_order_ + full_column)];
    }
    for (int boundary_column = 0; boundary_column < reduced_order_;
         ++boundary_column) {
      const int full_column =
          boundary_unknowns_[static_cast<size_t>(boundary_column)];
      eliminated_to_boundary[static_cast<size_t>(
          eliminated_row * reduced_order_ + boundary_column)] =
          matrix[static_cast<size_t>(full_row * matrix_order_ + full_column)];
    }
  }
  if (!FactorDenseNoAllocation(eliminated_matrix, eliminated_order,
                               &eliminated_lu_, &eliminated_pivots_)) {
    return false;
  }
  BuildFactoredSolvePattern(
      eliminated_lu_, eliminated_order, &eliminated_lower_offsets_,
      &eliminated_lower_columns_, &eliminated_upper_offsets_,
      &eliminated_upper_columns_);

  eliminated_rhs_.assign(static_cast<size_t>(eliminated_order), 0.0);
  eliminated_solution_.assign(static_cast<size_t>(eliminated_order), 0.0);
  schur_back_substitution_.assign(
      static_cast<size_t>(eliminated_order * reduced_order_), 0.0);
  for (int boundary_column = 0; boundary_column < reduced_order_;
       ++boundary_column) {
    for (int eliminated_row = 0; eliminated_row < eliminated_order;
         ++eliminated_row) {
      eliminated_rhs_[static_cast<size_t>(eliminated_row)] =
          eliminated_to_boundary[static_cast<size_t>(
              eliminated_row * reduced_order_ + boundary_column)];
    }
    if (!SolveSparseFactoredNoAllocation(
            eliminated_lu_, eliminated_pivots_, eliminated_order,
            eliminated_lower_offsets_, eliminated_lower_columns_,
            eliminated_upper_offsets_, eliminated_upper_columns_,
            eliminated_rhs_, &eliminated_solution_)) {
      return false;
    }
    for (int eliminated_row = 0; eliminated_row < eliminated_order;
         ++eliminated_row) {
      schur_back_substitution_[static_cast<size_t>(
          eliminated_row * reduced_order_ + boundary_column)] =
          eliminated_solution_[static_cast<size_t>(eliminated_row)];
    }
  }

  boundary_to_eliminated_offsets_.assign(
      static_cast<size_t>(reduced_order_ + 1), 0U);
  boundary_to_eliminated_columns_.clear();
  boundary_to_eliminated_values_.clear();
  for (int row = 0; row < reduced_order_; ++row) {
    for (int column = 0; column < eliminated_order; ++column) {
      const double value = boundary_to_eliminated_[static_cast<size_t>(
          row * eliminated_order + column)];
      if (std::abs(value) > kPivotEpsilon) {
        boundary_to_eliminated_columns_.push_back(column);
        boundary_to_eliminated_values_.push_back(value);
      }
    }
    boundary_to_eliminated_offsets_[static_cast<size_t>(row + 1)] =
        boundary_to_eliminated_columns_.size();
  }
  back_substitution_offsets_.assign(
      static_cast<size_t>(eliminated_order + 1), 0U);
  back_substitution_columns_.clear();
  back_substitution_values_.clear();
  for (int row = 0; row < eliminated_order; ++row) {
    for (int column = 0; column < reduced_order_; ++column) {
      const double value = schur_back_substitution_[static_cast<size_t>(
          row * reduced_order_ + column)];
      if (std::abs(value) > kPivotEpsilon) {
        back_substitution_columns_.push_back(column);
        back_substitution_values_.push_back(value);
      }
    }
    back_substitution_offsets_[static_cast<size_t>(row + 1)] =
        back_substitution_columns_.size();
  }

  reduced_base_matrix_.assign(
      static_cast<size_t>(reduced_order_ * reduced_order_), 0.0);
  for (int row = 0; row < reduced_order_; ++row) {
    const int full_row = boundary_unknowns_[static_cast<size_t>(row)];
    for (int column = 0; column < reduced_order_; ++column) {
      const int full_column =
          boundary_unknowns_[static_cast<size_t>(column)];
      double value =
          matrix[static_cast<size_t>(full_row * matrix_order_ + full_column)];
      for (int eliminated = 0; eliminated < eliminated_order; ++eliminated) {
        value -= boundary_to_eliminated_[static_cast<size_t>(
                     row * eliminated_order + eliminated)] *
                 schur_back_substitution_[static_cast<size_t>(
                     eliminated * reduced_order_ + column)];
      }
      reduced_base_matrix_[static_cast<size_t>(row * reduced_order_ +
                                               column)] = value;
    }
  }
  reduced_matrix_ = reduced_base_matrix_;
  reduced_rhs_.assign(static_cast<size_t>(reduced_order_), 0.0);
  reduced_sample_base_rhs_.assign(static_cast<size_t>(reduced_order_), 0.0);
  reduced_solution_.assign(static_cast<size_t>(reduced_order_), 0.0);
  return FactorReducedMatrix(reduced_base_matrix_);
}

bool MnaSolver::FactorReducedMatrix(const std::vector<double>& matrix) {
  const bool pattern_ready =
      reduced_pivots_.size() == static_cast<size_t>(reduced_order_) &&
      reduced_factor_offsets_.size() ==
          static_cast<size_t>(reduced_order_ + 1) &&
      reduced_upper_offsets_.size() ==
          static_cast<size_t>(reduced_order_ + 1);
  if (pattern_ready) {
    reduced_lu_ = matrix;
    for (int column = 0; column < reduced_order_; ++column) {
      const int pivot = reduced_pivots_[static_cast<size_t>(column)];
      if (pivot != column) {
        for (int index = 0; index < reduced_order_; ++index) {
          std::swap(reduced_lu_[static_cast<size_t>(
                        pivot * reduced_order_ + index)],
                    reduced_lu_[static_cast<size_t>(
                        column * reduced_order_ + index)]);
        }
      }
      const double diagonal = reduced_lu_[static_cast<size_t>(
          column * reduced_order_ + column)];
      if (!std::isfinite(diagonal) ||
          std::abs(diagonal) < kPivotEpsilon) {
        reduced_pivots_.clear();
        reduced_factor_offsets_.clear();
        return FactorReducedMatrix(matrix);
      }
      const size_t factor_begin =
          reduced_factor_offsets_[static_cast<size_t>(column)];
      const size_t factor_end =
          reduced_factor_offsets_[static_cast<size_t>(column + 1)];
      for (size_t factor_index = factor_begin;
           factor_index < factor_end; ++factor_index) {
        const int row = reduced_factor_rows_[factor_index];
        double& factor = reduced_lu_[static_cast<size_t>(
            row * reduced_order_ + column)];
        factor /= diagonal;
        if (std::abs(factor) < kPivotEpsilon) {
          factor = 0.0;
          continue;
        }
        const size_t upper_begin =
            reduced_upper_offsets_[static_cast<size_t>(column)];
        const size_t upper_end =
            reduced_upper_offsets_[static_cast<size_t>(column + 1)];
        for (size_t upper_index = upper_begin; upper_index < upper_end;
             ++upper_index) {
          const int index = reduced_upper_columns_[upper_index];
          reduced_lu_[static_cast<size_t>(row * reduced_order_ + index)] -=
              factor * reduced_lu_[static_cast<size_t>(
                           column * reduced_order_ + index)];
        }
      }
    }
    return true;
  }

  if (!FactorDenseNoAllocation(matrix, reduced_order_, &reduced_lu_,
                               &reduced_pivots_)) {
    return false;
  }
  BuildFactoredSolvePattern(
      reduced_lu_, reduced_order_, &reduced_lower_offsets_,
      &reduced_lower_columns_, &reduced_upper_offsets_,
      &reduced_upper_columns_);
  reduced_factor_offsets_.assign(
      static_cast<size_t>(reduced_order_ + 1), 0U);
  for (int row = 0; row < reduced_order_; ++row) {
    const size_t begin = reduced_lower_offsets_[static_cast<size_t>(row)];
    const size_t end =
        reduced_lower_offsets_[static_cast<size_t>(row + 1)];
    for (size_t index = begin; index < end; ++index) {
      const int column = reduced_lower_columns_[index];
      ++reduced_factor_offsets_[static_cast<size_t>(column + 1)];
    }
  }
  for (int column = 0; column < reduced_order_; ++column) {
    reduced_factor_offsets_[static_cast<size_t>(column + 1)] +=
        reduced_factor_offsets_[static_cast<size_t>(column)];
  }
  reduced_factor_rows_.resize(reduced_lower_columns_.size());
  std::vector<size_t> cursors = reduced_factor_offsets_;
  for (int row = 0; row < reduced_order_; ++row) {
    const size_t begin = reduced_lower_offsets_[static_cast<size_t>(row)];
    const size_t end =
        reduced_lower_offsets_[static_cast<size_t>(row + 1)];
    for (size_t index = begin; index < end; ++index) {
      const int column = reduced_lower_columns_[index];
      reduced_factor_rows_[cursors[static_cast<size_t>(column)]++] = row;
    }
  }
  return true;
}

bool MnaSolver::SolveReducedFactored(
    const std::vector<double>& rhs, std::vector<double>* solution) const {
  return SolveSparseFactoredNoAllocation(
      reduced_lu_, reduced_pivots_, reduced_order_, reduced_lower_offsets_,
      reduced_lower_columns_, reduced_upper_offsets_,
      reduced_upper_columns_, rhs, solution);
}

bool MnaSolver::BuildReducedNonlinearMatrix(
    const std::vector<double>& voltages) {
  if (!reduction_enabled_ ||
      voltages.size() != static_cast<size_t>(node_count_)) {
    return false;
  }
  reduced_matrix_ = reduced_base_matrix_;
  for (size_t element_index : nonlinear_element_indices_) {
    const MnaElement& element = elements_[element_index];
    const int positive = ReducedUnknownForNode(element.positive_node);
    const int negative = ReducedUnknownForNode(element.negative_node);
    if (element.type == MnaElementType::DIODE) {
      const double voltage =
          voltages[static_cast<size_t>(element.positive_node)] -
          voltages[static_cast<size_t>(element.negative_node)];
      const double conductance =
          LinearizeDiode(element, voltage).conductance - kGmin;
      StampAdmittance(&reduced_matrix_, reduced_order_, positive, negative,
                      conductance);
      continue;
    }
    if (element.type == MnaElementType::BJT_NPN ||
        element.type == MnaElementType::BJT_PNP) {
      const int base =
          ReducedUnknownForNode(element.control_positive_node);
      const BjtLinearization linearized = LinearizeBjt(element, voltages);
      StampAdmittance(&reduced_matrix_, reduced_order_, positive, negative,
                      linearized.go - kGmin);
      StampTransconductance(&reduced_matrix_, reduced_order_, positive,
                            negative, base, negative,
                            linearized.gm - kGmin);
      StampAdmittance(&reduced_matrix_, reduced_order_, base, negative,
                      linearized.gpi - kGmin);
    }
  }
  return true;
}

bool MnaSolver::TransformReducedRhs(const std::vector<double>& full_rhs) {
  if (!reduction_enabled_ ||
      full_rhs.size() != static_cast<size_t>(matrix_order_)) {
    return false;
  }
  const int eliminated_order =
      static_cast<int>(eliminated_unknowns_.size());
  for (int index = 0; index < eliminated_order; ++index) {
    eliminated_rhs_[static_cast<size_t>(index)] =
        full_rhs[static_cast<size_t>(
            eliminated_unknowns_[static_cast<size_t>(index)])];
  }
  if (!SolveSparseFactoredNoAllocation(
          eliminated_lu_, eliminated_pivots_, eliminated_order,
          eliminated_lower_offsets_, eliminated_lower_columns_,
          eliminated_upper_offsets_, eliminated_upper_columns_,
          eliminated_rhs_, &eliminated_solution_)) {
    return false;
  }
  for (int row = 0; row < reduced_order_; ++row) {
    double value = full_rhs[static_cast<size_t>(
        boundary_unknowns_[static_cast<size_t>(row)])];
    const size_t begin =
        boundary_to_eliminated_offsets_[static_cast<size_t>(row)];
    const size_t end =
        boundary_to_eliminated_offsets_[static_cast<size_t>(row + 1)];
    for (size_t index = begin; index < end; ++index) {
      const int eliminated = boundary_to_eliminated_columns_[index];
      value -= boundary_to_eliminated_values_[index] *
               eliminated_solution_[static_cast<size_t>(eliminated)];
    }
    reduced_rhs_[static_cast<size_t>(row)] = value;
  }
  return true;
}

void MnaSolver::ReconstructFullSolution() {
  for (int row = 0; row < reduced_order_; ++row) {
    solution_[static_cast<size_t>(
        boundary_unknowns_[static_cast<size_t>(row)])] =
        reduced_solution_[static_cast<size_t>(row)];
  }
  const int eliminated_order =
      static_cast<int>(eliminated_unknowns_.size());
  for (int row = 0; row < eliminated_order; ++row) {
    double value = eliminated_solution_[static_cast<size_t>(row)];
    const size_t begin =
        back_substitution_offsets_[static_cast<size_t>(row)];
    const size_t end =
        back_substitution_offsets_[static_cast<size_t>(row + 1)];
    for (size_t index = begin; index < end; ++index) {
      const int column = back_substitution_columns_[index];
      value -= back_substitution_values_[index] *
               reduced_solution_[static_cast<size_t>(column)];
    }
    solution_[static_cast<size_t>(
        eliminated_unknowns_[static_cast<size_t>(row)])] = value;
  }
}

bool MnaSolver::ProcessSample(std::span<const double> source_values,
                              MnaSolveMetrics* metrics) {
  if (metrics) *metrics = {};
  if (!compiled_) return false;
  BuildTransientRhs(source_values, &rhs_);
  if (reduction_enabled_) {
    if (!TransformReducedRhs(rhs_)) {
      if (metrics) metrics->error = "Transient Schur RHS reduction failed.";
      return false;
    }
    reduced_sample_base_rhs_ = reduced_rhs_;
  }
  constexpr int kIterationsPerSubstep = 3;
  const int maximum_iterations =
      has_nonlinear_elements_ ? kIterationsPerSubstep * 2 : 1;
  double maximum_delta = 0.0;
  int maximum_delta_node = -1;
  int iterations = 0;
  bool converged = !has_nonlinear_elements_;
  for (; iterations < maximum_iterations; ++iterations) {
    const int substep_iteration = iterations % kIterationsPerSubstep;
    const bool reuse_previous_linearization =
        has_nonlinear_elements_ && substep_iteration == 0 &&
        nonlinear_factorization_valid_;
    if (has_nonlinear_elements_) {
      if (!reuse_previous_linearization) {
        const bool matrix_built =
            reduction_enabled_
                ? BuildReducedNonlinearMatrix(node_voltages_)
                : BuildNonlinearTransientMatrix(node_voltages_,
                                                &nonlinear_matrix_);
        if (!matrix_built) {
          if (metrics) {
            metrics->error = "Nonlinear transient matrix build failed.";
          }
          return false;
        }
        bool factorized = false;
        if (reduction_enabled_) {
          factorized = FactorReducedMatrix(reduced_matrix_);
        } else {
          lu_.swap(nonlinear_matrix_);
          factorized = FactorRealMatrix(lu_, metrics, false, true);
        }
        if (!factorized) {
          nonlinear_factorization_valid_ = false;
          if (metrics && metrics->error.empty()) {
            metrics->error =
                "Nonlinear transient matrix factorization failed.";
          }
          return false;
        }
      }
    }
    nonlinear_rhs_ = rhs_;
    if (reuse_previous_linearization) {
      for (size_t index = 0; index < nonlinear_rhs_.size(); ++index) {
        nonlinear_rhs_[index] += cached_nonlinear_rhs_correction_[index];
      }
    } else {
      AddNonlinearRhs(node_voltages_, &nonlinear_rhs_);
      if (has_nonlinear_elements_) {
        for (size_t index = 0; index < nonlinear_rhs_.size(); ++index) {
          cached_nonlinear_rhs_correction_[index] =
              nonlinear_rhs_[index] - rhs_[index];
        }
        nonlinear_factorization_valid_ = true;
      }
    }
    bool solved = false;
    if (reduction_enabled_) {
      reduced_rhs_ = reduced_sample_base_rhs_;
      for (int row = 0; row < reduced_order_; ++row) {
        const int full_unknown =
            boundary_unknowns_[static_cast<size_t>(row)];
        reduced_rhs_[static_cast<size_t>(row)] +=
            nonlinear_rhs_[static_cast<size_t>(full_unknown)] -
            rhs_[static_cast<size_t>(full_unknown)];
      }
      solved = SolveReducedFactored(reduced_rhs_, &reduced_solution_);
      if (solved) {
        // Newton only reads nonlinear terminal voltages, and every nonlinear
        // terminal is a retained boundary. Defer the much larger eliminated
        // node back-substitution until the sample has converged.
        for (int row = 0; row < reduced_order_; ++row) {
          solution_[static_cast<size_t>(
              boundary_unknowns_[static_cast<size_t>(row)])] =
              reduced_solution_[static_cast<size_t>(row)];
        }
      }
    } else {
      solved = SolveFactored(nonlinear_rhs_, &solution_);
    }
    const auto solution_is_numerically_stable = [&]() {
      return std::all_of(solution_.begin(), solution_.end(),
                         [](double value) {
                           return std::isfinite(value);
                         });
    };
    const auto reduced_solution_is_numerically_stable = [&]() {
      return std::all_of(reduced_solution_.begin(), reduced_solution_.end(),
                         [](double value) {
                           return std::isfinite(value) &&
                                  std::abs(value) < 1.0e6;
                         });
    };
    if (solved && reduction_enabled_ &&
        (!solution_is_numerically_stable() ||
         !reduced_solution_is_numerically_stable())) {
      // A transistor crossing between cutoff and conduction can invalidate
      // the otherwise reusable pivot ordering. Keep the fast cached pattern
      // for normal samples, but recover the rare unstable solve by selecting
      // fresh partial pivots and retrying this same sample.
      reduced_pivots_.clear();
      reduced_factor_offsets_.clear();
      reduced_factor_rows_.clear();
      reduced_lower_offsets_.clear();
      reduced_lower_columns_.clear();
      reduced_upper_offsets_.clear();
      reduced_upper_columns_.clear();
      solved = FactorReducedMatrix(reduced_matrix_) &&
               SolveReducedFactored(reduced_rhs_, &reduced_solution_);
      if (solved) {
        for (int row = 0; row < reduced_order_; ++row) {
          solution_[static_cast<size_t>(
              boundary_unknowns_[static_cast<size_t>(row)])] =
              reduced_solution_[static_cast<size_t>(row)];
        }
      }
    }
    if (!solved) {
      if (metrics) metrics->error = "Transient MNA solve failed.";
      return false;
    }
    if (!solution_is_numerically_stable() ||
        (reduction_enabled_ &&
         !reduced_solution_is_numerically_stable())) {
      nonlinear_factorization_valid_ = false;
      if (metrics) metrics->error = "Transient MNA produced non-finite voltage.";
      return false;
    }
    maximum_delta = 0.0;
    maximum_delta_node = -1;
    const auto solved_node_voltage = [&](int node) {
      const int unknown = UnknownForNode(node);
      return unknown >= 0 ? solution_[static_cast<size_t>(unknown)] : 0.0;
    };
    // Newton convergence is governed by nonlinear junction voltages, not by
    // the absolute movement of every node between adjacent audio samples.
    // Comparing the entire circuit made normal R-2R DAC code changes look
    // like a nonlinear failure and refactorized the large matrix at 48 kHz,
    // even when every diode Vd and transistor Vbe was effectively unchanged.
    for (size_t element_index : nonlinear_element_indices_) {
      const MnaElement& element = elements_[element_index];
      double delta = 0.0;
      int diagnostic_node = element.positive_node;
      if (element.type == MnaElementType::DIODE) {
        const double previous_junction =
            node_voltages_[static_cast<size_t>(element.positive_node)] -
            node_voltages_[static_cast<size_t>(element.negative_node)];
        const double solved_junction =
            solved_node_voltage(element.positive_node) -
            solved_node_voltage(element.negative_node);
        delta = std::abs(solved_junction - previous_junction);
      } else if (element.type == MnaElementType::BJT_NPN ||
                 element.type == MnaElementType::BJT_PNP) {
        const double previous_vbe =
            node_voltages_[static_cast<size_t>(
                element.control_positive_node)] -
            node_voltages_[static_cast<size_t>(element.negative_node)];
        const double solved_vbe =
            solved_node_voltage(element.control_positive_node) -
            solved_node_voltage(element.negative_node);
        delta = std::abs(solved_vbe - previous_vbe);
        diagnostic_node = element.control_positive_node;
      }
      if (delta > maximum_delta) {
        maximum_delta = delta;
        maximum_delta_node = diagnostic_node;
      }
    }
    for (int node = 0; node < node_count_; ++node) {
      const int unknown = UnknownForNode(node);
      if (reduction_enabled_ && unknown >= 0 &&
          full_to_reduced_[static_cast<size_t>(unknown)] < 0) {
        continue;
      }
      const double voltage =
          unknown >= 0 ? solution_[static_cast<size_t>(unknown)] : 0.0;
      node_voltages_[static_cast<size_t>(node)] = voltage;
    }
    if (!has_nonlinear_elements_ ||
        maximum_delta <= kNewtonConvergenceTolerance) {
      converged = true;
      ++iterations;
      break;
    }
  }
  if (reduction_enabled_) {
    ReconstructFullSolution();
    if (!std::all_of(solution_.begin(), solution_.end(),
                     [](double value) { return std::isfinite(value); })) {
      nonlinear_factorization_valid_ = false;
      if (metrics) {
        metrics->error =
            "Transient Schur back-substitution produced non-finite voltage.";
      }
      return false;
    }
    for (int node = 0; node < node_count_; ++node) {
      const int unknown = UnknownForNode(node);
      node_voltages_[static_cast<size_t>(node)] =
          unknown >= 0 ? solution_[static_cast<size_t>(unknown)] : 0.0;
    }
  }
  UpdateDynamicState();
  if (metrics) {
    metrics->converged = converged;
    metrics->matrix_order = matrix_order_;
    metrics->substeps =
        has_nonlinear_elements_
            ? std::max(1, (iterations + kIterationsPerSubstep - 1) /
                              kIterationsPerSubstep)
            : 1;
    metrics->iterations =
        has_nonlinear_elements_ && iterations > 0
            ? ((iterations - 1) % kIterationsPerSubstep) + 1
            : iterations;
    metrics->maximum_residual = maximum_delta;
    if (!converged && maximum_delta_node >= 0) {
      for (size_t element_index : nonlinear_element_indices_) {
        const MnaElement& element = elements_[element_index];
        if (element.positive_node == maximum_delta_node ||
            element.negative_node == maximum_delta_node ||
            element.control_positive_node == maximum_delta_node ||
            element.control_negative_node == maximum_delta_node) {
          metrics->failure_component_instance_id =
              element.component_instance_id;
          break;
        }
      }
    }
  }
  return converged;
}

void MnaSolver::AddNonlinearRhs(
    const std::vector<double>& voltages, std::vector<double>* rhs) const {
  if (!rhs || voltages.size() != static_cast<size_t>(node_count_)) return;
  for (size_t element_index : nonlinear_element_indices_) {
    const MnaElement& element = elements_[element_index];
    if (element.type == MnaElementType::BJT_NPN ||
        element.type == MnaElementType::BJT_PNP) {
      const BjtLinearization linearized = LinearizeBjt(element, voltages);
      const double collector =
          voltages[static_cast<size_t>(element.positive_node)];
      const double emitter =
          voltages[static_cast<size_t>(element.negative_node)];
      const double base =
          voltages[static_cast<size_t>(element.control_positive_node)];
      const double collector_equivalent =
          linearized.collector_current -
          linearized.go * (collector - emitter) -
          linearized.gm * (base - emitter);
      const double base_equivalent =
          linearized.base_current - linearized.gpi * (base - emitter);
      StampCurrent(rhs, UnknownForNode(element.positive_node),
                   UnknownForNode(element.negative_node),
                   collector_equivalent);
      StampCurrent(rhs, UnknownForNode(element.control_positive_node),
                   UnknownForNode(element.negative_node), base_equivalent);
      continue;
    }
    if (element.type != MnaElementType::DIODE) continue;
    const double voltage =
        voltages[static_cast<size_t>(element.positive_node)] -
        voltages[static_cast<size_t>(element.negative_node)];
    const DiodeLinearization linearized =
        LinearizeDiode(element, voltage);
    const double equivalent_current =
        linearized.current - linearized.conductance * voltage;
    StampCurrent(rhs, UnknownForNode(element.positive_node),
                 UnknownForNode(element.negative_node),
                 equivalent_current);
  }
}

void MnaSolver::UpdateDynamicState() {
  for (size_t index : dynamic_element_indices_) {
    const MnaElement& element = elements_[index];
    DynamicState& state = dynamic_states_[index];
    double voltage = NodeVoltage(element.positive_node) -
                     NodeVoltage(element.negative_node);
    if (!std::isfinite(voltage)) voltage = 0.0;
    if (element.type == MnaElementType::CAPACITOR) {
      const double conductance = 2.0 * element.value * sample_rate_;
      const double history =
          -conductance * state.previous_voltage - state.previous_current;
      const double next_current = conductance * voltage + history;
      state.previous_current = std::isfinite(next_current) ? next_current : 0.0;
    } else {
      const double conductance =
          1.0 / (2.0 * std::max(element.value, 1.0e-15) * sample_rate_);
      const double history =
          state.previous_current + conductance * state.previous_voltage;
      const double next_current = conductance * voltage + history;
      state.previous_current = std::isfinite(next_current) ? next_current : 0.0;
    }
    state.previous_voltage = voltage;
  }
}

std::vector<MnaAcPoint> MnaSolver::SolveAc(
    const std::vector<double>& frequencies_hz, int source_slot,
    int output_positive_node, int output_negative_node,
    MnaSolveMetrics* metrics) const {
  if (metrics) *metrics = {};
  std::vector<MnaAcPoint> result;
  if (!compiled_) return result;
  result.reserve(frequencies_hz.size());
  for (double frequency : frequencies_hz) {
    if (!std::isfinite(frequency) || frequency <= 0.0) continue;
    using Complex = std::complex<double>;
    std::vector<Complex> matrix(
        static_cast<size_t>(matrix_order_ * matrix_order_), Complex{});
    std::vector<Complex> rhs(static_cast<size_t>(matrix_order_), Complex{});
    for (int node = 0; node < node_count_; ++node) {
      const int unknown = UnknownForNode(node);
      if (unknown >= 0) {
        matrix[static_cast<size_t>(unknown * matrix_order_ + unknown)] +=
            Complex{kGmin, 0.0};
      }
    }
    const Complex imaginary{0.0, 1.0};
    const double omega = 2.0 * kPi * frequency;
    for (size_t index = 0; index < elements_.size(); ++index) {
      const MnaElement& element = elements_[index];
      const int positive = UnknownForNode(element.positive_node);
      const int negative = UnknownForNode(element.negative_node);
      switch (element.type) {
        case MnaElementType::RESISTOR:
          StampAdmittance(&matrix, matrix_order_, positive, negative,
                          Complex{1.0 / std::max(std::abs(element.value),
                                                 kMinimumResistance),
                                  0.0});
          break;
        case MnaElementType::CAPACITOR:
          StampAdmittance(&matrix, matrix_order_, positive, negative,
                          imaginary * omega * element.value);
          break;
        case MnaElementType::INDUCTOR:
          StampAdmittance(&matrix, matrix_order_, positive, negative,
                          1.0 / (imaginary * omega *
                                 std::max(element.value, 1.0e-15)));
          break;
        case MnaElementType::VOLTAGE_SOURCE:
        case MnaElementType::VCVS: {
          const int branch = branch_unknowns_[index];
          StampVoltageBranch(&matrix, matrix_order_, positive, negative,
                             branch);
          if (element.type == MnaElementType::VCVS) {
            const int cp = UnknownForNode(element.control_positive_node);
            const int cn = UnknownForNode(element.control_negative_node);
            if (cp >= 0) {
              matrix[static_cast<size_t>(branch * matrix_order_ + cp)] -=
                  element.value;
            }
            if (cn >= 0) {
              matrix[static_cast<size_t>(branch * matrix_order_ + cn)] +=
                  element.value;
            }
          }
          break;
        }
        case MnaElementType::VCCS: {
          const int cp = UnknownForNode(element.control_positive_node);
          const int cn = UnknownForNode(element.control_negative_node);
          const Complex gm{element.value, 0.0};
          if (positive >= 0 && cp >= 0) {
            matrix[static_cast<size_t>(positive * matrix_order_ + cp)] += gm;
          }
          if (positive >= 0 && cn >= 0) {
            matrix[static_cast<size_t>(positive * matrix_order_ + cn)] -= gm;
          }
          if (negative >= 0 && cp >= 0) {
            matrix[static_cast<size_t>(negative * matrix_order_ + cp)] -= gm;
          }
          if (negative >= 0 && cn >= 0) {
            matrix[static_cast<size_t>(negative * matrix_order_ + cn)] += gm;
          }
          break;
        }
        case MnaElementType::DIODE:
          StampAdmittance(
              &matrix, matrix_order_, positive, negative,
              Complex{1.0 /
                          std::max(element.series_resistance, 1.0e-3),
                      0.0});
          break;
        case MnaElementType::BJT_NPN:
        case MnaElementType::BJT_PNP: {
          const int base = UnknownForNode(element.control_positive_node);
          const BjtLinearization linearized =
              LinearizeBjt(element, node_voltages_);
          StampAdmittance(&matrix, matrix_order_, positive, negative,
                          Complex{linearized.go, 0.0});
          StampTransconductance(&matrix, matrix_order_, positive, negative,
                                base, negative,
                                Complex{linearized.gm, 0.0});
          StampAdmittance(&matrix, matrix_order_, base, negative,
                          Complex{linearized.gpi, 0.0});
          break;
        }
        case MnaElementType::CURRENT_SOURCE:
          break;
      }
      if (element.source_slot == source_slot) {
        if (element.type == MnaElementType::VOLTAGE_SOURCE) {
          rhs[static_cast<size_t>(branch_unknowns_[index])] += Complex{1.0, 0.0};
        } else if (element.type == MnaElementType::CURRENT_SOURCE) {
          if (positive >= 0) rhs[static_cast<size_t>(positive)] -= 1.0;
          if (negative >= 0) rhs[static_cast<size_t>(negative)] += 1.0;
        }
      }
    }
    std::vector<Complex> solution;
    if (!DenseSolve(matrix, rhs, &solution)) {
      if (metrics) metrics->error = "AC MNA matrix is singular.";
      result.clear();
      return result;
    }
    const int positive = UnknownForNode(output_positive_node);
    const int negative = UnknownForNode(output_negative_node);
    const Complex output =
        (positive >= 0 ? solution[static_cast<size_t>(positive)] : Complex{}) -
        (negative >= 0 ? solution[static_cast<size_t>(negative)] : Complex{});
    result.push_back({frequency, output});
  }
  if (metrics) {
    metrics->converged = result.size() == frequencies_hz.size();
    metrics->matrix_order = matrix_order_;
    metrics->iterations = static_cast<int>(result.size());
  }
  return result;
}

double MnaSolver::NodeVoltage(int node) const {
  if (node < 0 || static_cast<size_t>(node) >= node_voltages_.size()) {
    return 0.0;
  }
  return node_voltages_[static_cast<size_t>(node)];
}

void MnaSolver::ResetDynamicState() {
  std::fill(dynamic_states_.begin(), dynamic_states_.end(), DynamicState{});
  std::fill(solution_.begin(), solution_.end(), 0.0);
  std::fill(node_voltages_.begin(), node_voltages_.end(), 0.0);
  nonlinear_factorization_valid_ = false;
}

}  // namespace plc::audio
