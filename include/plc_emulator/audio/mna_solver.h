#ifndef PLC_EMULATOR_AUDIO_MNA_SOLVER_H_
#define PLC_EMULATOR_AUDIO_MNA_SOLVER_H_

#include <complex>
#include <cstddef>
#include <span>
#include <string>
#include <vector>

namespace plc::audio {

enum class MnaElementType {
  RESISTOR,
  CAPACITOR,
  INDUCTOR,
  CURRENT_SOURCE,
  VOLTAGE_SOURCE,
  VCVS,
  VCCS,
  DIODE,
  BJT_NPN,
  BJT_PNP,
};

struct MnaElement {
  MnaElementType type = MnaElementType::RESISTOR;
  int positive_node = 0;
  int negative_node = 0;
  int control_positive_node = 0;
  int control_negative_node = 0;
  double value = 0.0;
  double series_resistance = 0.0;
  int source_slot = -1;
  int component_instance_id = -1;
  double auxiliary_value = 0.0;
};

struct MnaAcPoint {
  double frequency_hz = 0.0;
  std::complex<double> output{};
};

struct MnaSolveMetrics {
  bool converged = false;
  int matrix_order = 0;
  int iterations = 0;
  int substeps = 1;
  double maximum_residual = 0.0;
  int failure_component_instance_id = -1;
  std::string error;
};

class MnaSolver {
 public:
  bool Compile(int node_count, int ground_node, double sample_rate,
               const std::vector<MnaElement>& elements,
               MnaSolveMetrics* metrics = nullptr);

  bool SolveDc(std::span<const double> source_values,
               MnaSolveMetrics* metrics = nullptr);

  bool ProcessSample(std::span<const double> source_values,
                     MnaSolveMetrics* metrics = nullptr);

  std::vector<MnaAcPoint> SolveAc(
      const std::vector<double>& frequencies_hz, int source_slot,
      int output_positive_node, int output_negative_node,
      MnaSolveMetrics* metrics = nullptr) const;

  double NodeVoltage(int node) const;
  const std::vector<double>& NodeVoltages() const { return node_voltages_; }
  int NodeCount() const { return node_count_; }
  int MatrixOrder() const { return matrix_order_; }
  int ReducedMatrixOrder() const {
    return reduction_enabled_ ? reduced_order_ : matrix_order_;
  }
  int EliminatedUnknownCount() const {
    return reduction_enabled_
               ? static_cast<int>(eliminated_unknowns_.size())
               : 0;
  }
  bool IsCompiled() const { return compiled_; }

  void ResetDynamicState();

 private:
  struct DynamicState {
    double previous_voltage = 0.0;
    double previous_current = 0.0;
  };

  int UnknownForNode(int node) const;
  bool BuildRealMatrix(bool dc, std::vector<double>* matrix,
                       const std::vector<double>* nonlinear_voltages = nullptr) const;
  bool BuildNonlinearTransientMatrix(
      const std::vector<double>& voltages,
      std::vector<double>* matrix) const;
  bool FactorRealMatrix(const std::vector<double>& matrix,
                        MnaSolveMetrics* metrics,
                        bool rebuild_sparsity = true,
                        bool matrix_already_loaded = false);
  bool SolveFactored(const std::vector<double>& rhs,
                     std::vector<double>* solution) const;
  void BuildTransientRhs(std::span<const double> source_values,
                         std::vector<double>* rhs) const;
  void UpdateDynamicState();
  void AddNonlinearRhs(const std::vector<double>& voltages,
                       std::vector<double>* rhs) const;
  bool BuildTransientReduction(const std::vector<double>& matrix);
  bool BuildReducedNonlinearMatrix(const std::vector<double>& voltages);
  bool TransformReducedRhs(const std::vector<double>& full_rhs);
  void ReconstructFullSolution();
  bool FactorReducedMatrix(const std::vector<double>& matrix);
  bool SolveReducedFactored(const std::vector<double>& rhs,
                            std::vector<double>* solution) const;
  int ReducedUnknownForNode(int node) const;

  int node_count_ = 0;
  int ground_node_ = 0;
  int matrix_order_ = 0;
  double sample_rate_ = 48000.0;
  bool compiled_ = false;
  bool has_nonlinear_elements_ = false;
  std::vector<MnaElement> elements_;
  std::vector<size_t> nonlinear_element_indices_;
  std::vector<size_t> rhs_element_indices_;
  std::vector<size_t> dynamic_element_indices_;
  std::vector<int> branch_unknowns_;
  std::vector<DynamicState> dynamic_states_;
  std::vector<double> lu_;
  std::vector<int> pivots_;
  std::vector<size_t> lower_row_offsets_;
  std::vector<int> lower_columns_flat_;
  std::vector<size_t> upper_row_offsets_;
  std::vector<int> upper_columns_flat_;
  std::vector<size_t> factor_column_offsets_;
  std::vector<int> factor_rows_flat_;
  std::vector<double> solution_;
  std::vector<double> rhs_;
  std::vector<double> nonlinear_rhs_;
  std::vector<double> cached_nonlinear_rhs_correction_;
  std::vector<double> nonlinear_matrix_;
  std::vector<double> transient_base_matrix_;
  std::vector<double> node_voltages_;
  bool nonlinear_factorization_valid_ = false;

  // Exact Schur reduction of resistor-only internal nodes. The public node
  // voltages remain full-sized; only the per-sample solve uses this system.
  bool reduction_enabled_ = false;
  int reduced_order_ = 0;
  std::vector<int> boundary_unknowns_;
  std::vector<int> eliminated_unknowns_;
  std::vector<int> full_to_reduced_;
  std::vector<double> boundary_to_eliminated_;
  std::vector<size_t> boundary_to_eliminated_offsets_;
  std::vector<int> boundary_to_eliminated_columns_;
  std::vector<double> boundary_to_eliminated_values_;
  std::vector<double> schur_back_substitution_;
  std::vector<size_t> back_substitution_offsets_;
  std::vector<int> back_substitution_columns_;
  std::vector<double> back_substitution_values_;
  std::vector<double> eliminated_lu_;
  std::vector<int> eliminated_pivots_;
  std::vector<size_t> eliminated_lower_offsets_;
  std::vector<int> eliminated_lower_columns_;
  std::vector<size_t> eliminated_upper_offsets_;
  std::vector<int> eliminated_upper_columns_;
  std::vector<double> eliminated_rhs_;
  std::vector<double> eliminated_solution_;
  std::vector<double> reduced_base_matrix_;
  std::vector<double> reduced_matrix_;
  std::vector<double> reduced_lu_;
  std::vector<int> reduced_pivots_;
  std::vector<size_t> reduced_lower_offsets_;
  std::vector<int> reduced_lower_columns_;
  std::vector<size_t> reduced_upper_offsets_;
  std::vector<int> reduced_upper_columns_;
  std::vector<size_t> reduced_factor_offsets_;
  std::vector<int> reduced_factor_rows_;
  std::vector<double> reduced_rhs_;
  std::vector<double> reduced_sample_base_rhs_;
  std::vector<double> reduced_solution_;
};

}  // namespace plc::audio

#endif
