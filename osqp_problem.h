#include <array>
#include <vector>
#include <iostream>
#include "osqp.h"

template <typename T>
T* CopyData(const std::vector<T>& vec) {
  T* data = new T[vec.size()];
  memcpy(data, vec.data(), sizeof(T) * vec.size());
  return data;
}

class OSQPProblem {
 public:
  OSQPProblem(const size_t num_of_knots, const double delta_s,
              const std::array<double, 3>& x_init);
  void CalculateKernel(std::vector<c_float>* P_data,
                       std::vector<c_int>* P_indices,
                       std::vector<c_int>* P_indptr);
  void CalculateAffineConstraint(std::vector<c_float>* A_data,
                                 std::vector<c_int>* A_indices,
                                 std::vector<c_int>* A_indptr,
                                 std::vector<c_float>* lower_bounds,
                                 std::vector<c_float>* upper_bounds);
  void CalculateOffset(std::vector<c_float>* q);
  OSQPSettings* SolverDefaultSettings();
  OSQPData* FormulateProblem();
  bool Optimize(const int max_iter);
  void FreeData(OSQPData* data);

  inline void set_weight_x(double w) { weight_x_ = w; }
  inline void set_weight_dx(double w) { weight_dx_ = w; }
  inline void set_weight_ddx(double w) { weight_ddx_ = w; }
  inline void set_weight_dddx(double w) { weight_dddx_ = w; }

  void set_end_state_ref(const std::array<double, 3>& weight_end_state,
                         const std::array<double, 3>& end_state_ref);

  void set_x_ref(const double weight_x_ref, std::vector<double> x_ref);

  inline void set_scale_factor(const std::array<double, 3>& factor) { scale_factor_ = factor; }

  inline void set_x_bounds(const std::vector<std::pair<double, double>>& x_bounds) { x_bounds_ = x_bounds; }
  void set_dx_bounds(double low_bound, double up_bound);
  void set_ddx_bounds(double low_bound, double up_bound);
  void set_dddx_bounds(double low, double up);

 private:
  size_t num_of_knots_;

  // output
  std::vector<double> x_;
  std::vector<double> dx_;
  std::vector<double> ddx_;

  std::array<double, 3> x_init_;
  std::array<double, 3> scale_factor_ = {{1.0, 1.0, 1.0}};

  std::vector<std::pair<double, double>> x_bounds_;
  std::vector<std::pair<double, double>> dx_bounds_;
  std::vector<std::pair<double, double>> ddx_bounds_;
  std::pair<double, double> dddx_bound_;

  double weight_x_ = 0.0;
  double weight_dx_ = 0.0;
  double weight_ddx_ = 0.0;
  double weight_dddx_ = 0.0;

  double delta_s_ = 0.5;

  bool has_x_ref_ = false;
  double weight_x_ref_ = 0.0;
  std::vector<double> x_ref_;

  bool has_end_state_ref_ = false;
  std::array<double, 3> weight_end_state_ = {{0.0, 0.0, 0.0}};
  std::array<double, 3> end_state_ref_;
};
