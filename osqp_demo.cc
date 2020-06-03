#include "osqp_problem.h"
#include <math.h>

using PathBoundary = std::vector<std::pair<double, double>>;

class PathBoundaryInfo {
 public:
  inline void set_start_s(double start_s) { start_s_ = start_s; }
  inline void set_delta_s(double delta_s) { delta_s_ = delta_s; }
  inline void set_boundary(const PathBoundary& boundary) { boundary_ = boundary; }

  inline double delta_s() { return delta_s_; }
  inline const PathBoundary& boundary() { return boundary_; }

 private:
  double start_s_;
  double delta_s_;
  PathBoundary boundary_;
};

void build_path_boundary(PathBoundaryInfo& bound_info, double planning_length,
                         double delta_s, double start_s) {
  bound_info.set_start_s(start_s);
  bound_info.set_delta_s(delta_s);

  int num = floor(planning_length / delta_s);
  PathBoundary bound;
  for (int i = 0; i < num; i++) {
    bound.emplace_back(std::make_pair(-2.0, 3.0));
  }

   int start = floor(num / 3.0);
   int end = floor(num / 2.0);

   for (int i = start; i < end; i++) {
     bound[i].first = -1.0;
   }

  bound_info.set_boundary(bound);
}

int main(int argc, char** argv) {
  std::cout<<"OSQP RUNNING..."<<std::endl;
  PathBoundaryInfo bound_info;
  build_path_boundary(bound_info, 50.0, 0.5, 0.0);

  std::array<double, 3> init_state = {1.0, 0.0, 0.0};
  OSQPProblem prob(bound_info.boundary().size(), bound_info.delta_s(),
                   init_state);

  std::array<double, 3> end_state = {-1.0, 0.0, 0.0};
  prob.set_end_state_ref({1000.0, 0.0, 0.0}, end_state);

  prob.set_weight_x(4.0);
  prob.set_weight_dx(20.0);
  prob.set_weight_ddx(1000.0);
  prob.set_weight_dddx(50000.0);

  prob.set_scale_factor({1.0, 10.0, 100.0});

  prob.set_x_bounds(bound_info.boundary());
  prob.set_dx_bounds(-2.0, 2.0);
  prob.set_ddx_bounds(-2.0, 2.0);
  prob.set_dddx_bounds(-4.0, 4.0);

  if(prob.Optimize(1000)){
    std::cout<<"Optimize successful!!"<<std::endl;
  }else{
    std::cout<<"Optimize failed!!"<<std::endl;
  }
  std::cout<<"OSQP END!!!"<<std::endl;

  return 1;
};
