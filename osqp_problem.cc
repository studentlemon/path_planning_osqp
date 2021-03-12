#include "osqp_problem.h"

namespace {
constexpr double kMaxVarRange = 1.0e10;
}

OSQPProblem::OSQPProblem(const size_t num_of_knots, const double delta_s,
                         const std::array<double, 3>& x_init) {
  if (num_of_knots <= 2) {
    std::cout << "Error when init problem." << std::endl;
    return;
  }
  num_of_knots_ = num_of_knots;
  x_init_ = x_init;
  delta_s_ = delta_s;
  x_bounds_.resize(num_of_knots_, std::make_pair(-kMaxVarRange, kMaxVarRange));
  dx_bounds_.resize(num_of_knots_, std::make_pair(-kMaxVarRange, kMaxVarRange));
  ddx_bounds_.resize(num_of_knots_,
                     std::make_pair(-kMaxVarRange, kMaxVarRange));
}

void OSQPProblem::CalculateKernel(std::vector<c_float>* P_data,
                                  std::vector<c_int>* P_indices,
                                  std::vector<c_int>* P_indptr) {
  const int n = static_cast<int>(num_of_knots_);
  const int num_of_variables = 3 * n;
  const int num_of_nonzeros = num_of_variables + (n - 1);
  std::vector<std::vector<std::pair<c_int, c_float>>> columns(num_of_variables);
  int value_index = 0;

  // x(i)^2 * (w_x + w_x_ref)
  for (int i = 0; i < n - 1; ++i) {
    columns[i].emplace_back(
        i, (weight_x_ + weight_x_ref_) / (scale_factor_[0] * scale_factor_[0]));
    ++value_index;
  }
  // x(n-1)^2 * (w_x + w_x_ref + w_end_x)
  columns[n - 1].emplace_back(
      n - 1, (weight_x_ + weight_x_ref_ + weight_end_state_[0]) /
                 (scale_factor_[0] * scale_factor_[0]));
  ++value_index;

  // x(i)'^2 * w_dx
  for (int i = 0; i < n - 1; ++i) {
    columns[n + i].emplace_back(
        n + i, weight_dx_ / (scale_factor_[1] * scale_factor_[1]));
    ++value_index;
  }
  // x(n-1)'^2 * (w_dx + w_end_dx)
  columns[2 * n - 1].emplace_back(2 * n - 1,
                                  (weight_dx_ + weight_end_state_[1]) /
                                      (scale_factor_[1] * scale_factor_[1]));
  ++value_index;

  auto delta_s_square = delta_s_ * delta_s_;
  // x(i)''^2 * (w_ddx + 2 * w_dddx / delta_s^2)
  columns[2 * n].emplace_back(2 * n,
                              (weight_ddx_ + weight_dddx_ / delta_s_square) /
                                  (scale_factor_[2] * scale_factor_[2]));
  ++value_index;
  for (int i = 1; i < n - 1; ++i) {
    columns[2 * n + i].emplace_back(
        2 * n + i, (weight_ddx_ + 2.0 * weight_dddx_ / delta_s_square) /
                       (scale_factor_[2] * scale_factor_[2]));
    ++value_index;
  }


  columns[3 * n - 1].emplace_back(
      3 * n - 1,
      (weight_ddx_ + 1.0*weight_dddx_ / delta_s_square + weight_end_state_[2]) /
          (scale_factor_[2] * scale_factor_[2]));
  ++value_index;

  // -2 * w_dddx / delta_s^2 * x(i)'' * x(i + 1)''
//   for (int i = 1; i < n; ++i) {
//     columns[2 * n + i].emplace_back(2 * n + i - 1,
//                                     (-1.0 * weight_dddx_ / delta_s_square) /
//                                         (scale_factor_[2] * scale_factor_[2]));
//     ++value_index;
//   }

  if (value_index != num_of_nonzeros) {
    std::cout << "Error in calculate kernel!" << std::endl;
  }

  int ind_p = 0;
  for (int i = 0; i < num_of_variables; ++i) {
    P_indptr->push_back(ind_p);
    for (const auto& row_data_pair : columns[i]) {
      P_data->push_back(row_data_pair.second * 2.0);
      P_indices->push_back(row_data_pair.first);
      ++ind_p;
    }
  }
  P_indptr->push_back(ind_p);
}

void OSQPProblem::CalculateAffineConstraint(
    std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
    std::vector<c_int>* A_indptr, std::vector<c_float>* lower_bounds,
    std::vector<c_float>* upper_bounds) {
  // 3N params bounds on x, x', x''
  // 3(N-1) constraints on x, x', x''
  // 3 constraints on x_init_

  const int n = static_cast<int>(num_of_knots_);
  const int num_of_variables = 3 * n;
  const int num_of_constraints = num_of_variables + 3 * (n - 1) + 3;
  lower_bounds->resize(num_of_constraints);
  upper_bounds->resize(num_of_constraints);

  std::vector<std::vector<std::pair<c_int, c_float>>> variables(
      num_of_variables);

  int constraint_index = 0;
  // set x, x', x'' bounds
  for (int i = 0; i < num_of_variables; ++i) {
    if (i < n) {
      variables[i].emplace_back(constraint_index, 1.0);
      lower_bounds->at(constraint_index) =
          x_bounds_[i].first * scale_factor_[0];
      upper_bounds->at(constraint_index) =
          x_bounds_[i].second * scale_factor_[0];
    } else if (i < 2 * n) {
      variables[i].emplace_back(constraint_index, 1.0);
      lower_bounds->at(constraint_index) =
          dx_bounds_[i - n].first * scale_factor_[1];
      upper_bounds->at(constraint_index) =
          dx_bounds_[i - n].second * scale_factor_[1];
    } else {
      variables[i].emplace_back(constraint_index, 1.0);
      lower_bounds->at(constraint_index) =
          ddx_bounds_[i - 2 * n].first * scale_factor_[2];
      upper_bounds->at(constraint_index) =
          ddx_bounds_[i - 2 * n].second * scale_factor_[2];
    }
    ++constraint_index;
  }

  if (constraint_index != num_of_variables) {
    std::cout << "Error in Calculate Affine Constraint 1" << std::endl;
  }

  // x(i->i+1)''' = (x(i+1)'' - x(i)'') / delta_s
  for (int i = 0; i + 1 < n; ++i) {
    variables[2 * n + i].emplace_back(constraint_index, -1.0);
    variables[2 * n + i + 1].emplace_back(constraint_index, 1.0);
    lower_bounds->at(constraint_index) =
        dddx_bound_.first * delta_s_ * scale_factor_[2];
    upper_bounds->at(constraint_index) =
        dddx_bound_.second * delta_s_ * scale_factor_[2];
    ++constraint_index;
    if(lower_bounds->at(constraint_index) > 0){
      std::cout<<"error"<<std::endl;
    }
  }

  // x(i+1)' - x(i)' - 0.5 * delta_s * x(i)'' - 0.5 * delta_s * x(i+1)'' = 0
  for (int i = 0; i + 1 < n; ++i) {
    variables[n + i].emplace_back(constraint_index, -1.0 * scale_factor_[2]);
    variables[n + i + 1].emplace_back(constraint_index, 1.0 * scale_factor_[2]);
    variables[2 * n + i].emplace_back(constraint_index,
                                      -0.5 * delta_s_ * scale_factor_[1]);
    variables[2 * n + i + 1].emplace_back(constraint_index,
                                          -0.5 * delta_s_ * scale_factor_[1]);
    lower_bounds->at(constraint_index) = 0.0;
    upper_bounds->at(constraint_index) = 0.0;
    ++constraint_index;
  }

  // x(i+1) - x(i) - delta_s * x(i)'
  // - 1/3 * delta_s^2 * x(i)'' - 1/6 * delta_s^2 * x(i+1)''
  auto delta_s_sq_ = delta_s_ * delta_s_;
  for (int i = 0; i + 1 < n; ++i) {
    variables[i].emplace_back(constraint_index,
                              -1.0 * scale_factor_[1] * scale_factor_[2]);
    variables[i + 1].emplace_back(constraint_index,
                                  1.0 * scale_factor_[1] * scale_factor_[2]);
    variables[n + i].emplace_back(
        constraint_index, -delta_s_ * scale_factor_[0] * scale_factor_[2]);
    variables[2 * n + i].emplace_back(
        constraint_index,
        -delta_s_sq_ / 3.0 * scale_factor_[0] * scale_factor_[1]);
    variables[2 * n + i + 1].emplace_back(
        constraint_index,
        -delta_s_sq_ / 6.0 * scale_factor_[0] * scale_factor_[1]);

    lower_bounds->at(constraint_index) = 0.0;
    upper_bounds->at(constraint_index) = 0.0;
    ++constraint_index;
  }

  // constrain on x_init
  variables[0].emplace_back(constraint_index, 1.0);
  lower_bounds->at(constraint_index) = x_init_[0] * scale_factor_[0];
  upper_bounds->at(constraint_index) = x_init_[0] * scale_factor_[0];
  ++constraint_index;

  variables[n].emplace_back(constraint_index, 1.0);
  lower_bounds->at(constraint_index) = x_init_[1] * scale_factor_[1];
  upper_bounds->at(constraint_index) = x_init_[1] * scale_factor_[1];
  ++constraint_index;

  variables[2 * n].emplace_back(constraint_index, 1.0);
  lower_bounds->at(constraint_index) = x_init_[2] * scale_factor_[2];
  upper_bounds->at(constraint_index) = x_init_[2] * scale_factor_[2];
  ++constraint_index;

  if (constraint_index != num_of_constraints) {
    std::cout << "Error in Calculate Affine Constraint 2" << std::endl;
  }

  int ind_p = 0;
  for (int i = 0; i < num_of_variables; ++i) {
    A_indptr->push_back(ind_p);
    for (const auto& variable_nz : variables[i]) {
      // coefficient
      A_data->push_back(variable_nz.second);

      // constraint index
      A_indices->push_back(variable_nz.first);
      ++ind_p;
    }
  }

  A_indptr->push_back(ind_p);
}

void OSQPProblem::CalculateOffset(std::vector<c_float>* q) {
  if (q == nullptr) {
    std::cout << "q should not be nullptr!!" << std::endl;
    return;
  }
  const int n = static_cast<int>(num_of_knots_);
  const int kNumParam = 3 * n;
  q->resize(kNumParam, 0.0);

  if (has_x_ref_) {
    for (int i = 0; i < n; ++i) {
      q->at(i) += -2.0 * weight_x_ref_ * x_ref_[i] / scale_factor_[0];
    }
  }

  if (has_end_state_ref_) {
    q->at(n - 1) +=
        -2.0 * weight_end_state_[0] * end_state_ref_[0] / scale_factor_[0];
    q->at(2 * n - 1) +=
        -2.0 * weight_end_state_[1] * end_state_ref_[1] / scale_factor_[1];
    q->at(3 * n - 1) +=
        -2.0 * weight_end_state_[2] * end_state_ref_[2] / scale_factor_[2];
  }
}

OSQPData* OSQPProblem::FormulateProblem() {
  // calculate kernel
  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;
  CalculateKernel(&P_data, &P_indices, &P_indptr);

  // calculate affine constraints
  std::vector<c_float> A_data;
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
  std::vector<c_float> lower_bounds;
  std::vector<c_float> upper_bounds;
  CalculateAffineConstraint(&A_data, &A_indices, &A_indptr, &lower_bounds,
                            &upper_bounds);
  // calculate offset
  std::vector<c_float> q;
  CalculateOffset(&q);

  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  if (lower_bounds.size() != upper_bounds.size()) {
    std::cout << "Formulate data failed!" << std::endl;
  }

  size_t kernel_dim = 3 * num_of_knots_;
  size_t num_affine_constraint = lower_bounds.size();

  data->n = kernel_dim;
  data->m = num_affine_constraint;
  data->P = csc_matrix(kernel_dim, kernel_dim, P_data.size(), CopyData(P_data),
                       CopyData(P_indices), CopyData(P_indptr));
  data->q = CopyData(q);
  data->A =
      csc_matrix(num_affine_constraint, kernel_dim, A_data.size(),
                 CopyData(A_data), CopyData(A_indices), CopyData(A_indptr));
  data->l = CopyData(lower_bounds);
  data->u = CopyData(upper_bounds);
  return data;
}

OSQPSettings* OSQPProblem::SolverDefaultSettings() {
  // Define Solver default settings
  OSQPSettings* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
  osqp_set_default_settings(settings);
  settings->eps_abs = 5e-3;
  settings->eps_rel = 5e-3;
  settings->eps_prim_inf = 5e-4;
  settings->eps_dual_inf = 5e-4;
  settings->polish = true;
  settings->verbose = false;
  settings->scaled_termination = true;
  return settings;
}

bool OSQPProblem::Optimize(const int max_iter) {
  OSQPData* data = FormulateProblem();

  OSQPSettings* settings = SolverDefaultSettings();
  settings->max_iter = max_iter;

  OSQPWorkspace* osqp_work;

  c_int status = 0;

  status = osqp_setup(&osqp_work, data, settings);

  osqp_solve(osqp_work);

  if (status != 0) {
    std::cout << "failed optimization status:\t" << osqp_work->info->status
              << std::endl;
    osqp_cleanup(osqp_work);
    FreeData(data);
    c_free(settings);
    return false;
  } else if (osqp_work->solution == nullptr) {
    std::cout << "The solution from OSQP is nullptr" << std::endl;
    osqp_cleanup(osqp_work);
    FreeData(data);
    c_free(settings);
    return false;
  }

  // extract primal results
  x_.resize(num_of_knots_);
  dx_.resize(num_of_knots_);
  ddx_.resize(num_of_knots_);
  for (size_t i = 0; i < num_of_knots_; ++i) {
    x_.at(i) = osqp_work->solution->x[i] / scale_factor_[0];
    dx_.at(i) = osqp_work->solution->x[i + num_of_knots_] / scale_factor_[1];
    ddx_.at(i) = osqp_work->solution->x[i + 2 * num_of_knots_] / scale_factor_[2];
  }

  // Cleanup
  osqp_cleanup(osqp_work);
  FreeData(data);
  c_free(settings);
  return true;
}

void OSQPProblem::FreeData(OSQPData* data) {
  delete[] data->q;
  delete[] data->l;
  delete[] data->u;

  delete[] data->P->i;
  delete[] data->P->p;
  delete[] data->P->x;

  delete[] data->A->i;
  delete[] data->A->p;
  delete[] data->A->x;
}

void OSQPProblem::set_end_state_ref(
    const std::array<double, 3>& weight_end_state,
    const std::array<double, 3>& end_state_ref) {
  weight_end_state_ = weight_end_state;
  end_state_ref_ = end_state_ref;
  has_end_state_ref_ = true;
}

void OSQPProblem::set_x_ref(const double weight_x_ref,
                            std::vector<double> x_ref) {
  if (x_ref.size() != num_of_knots_) {
    std::cout << "x_ref size is wrong!" << std::endl;
    return;
  }
  weight_x_ref_ = weight_x_ref;
  x_ref_ = x_ref;
  has_x_ref_ = true;
}

void OSQPProblem::set_dx_bounds(double low_bound, double up_bound) {
  for (auto& x : dx_bounds_) {
    x.first = low_bound;
    x.second = up_bound;
  }
}

void OSQPProblem::set_ddx_bounds(double low_bound, double up_bound) {
  for (auto& x : ddx_bounds_) {
    x.first = low_bound;
    x.second = up_bound;
  }
}

void OSQPProblem::set_dddx_bounds(double low, double up) {
  dddx_bound_.first = low;
  dddx_bound_.second = up;
}
