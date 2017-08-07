#include <cmath>
#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"


using CppAD::AD;

const size_t N  = 5;
const double dt = 0.3;


// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

// Scale factors for contributors to the cost function
const double FACT_CTE          = 2.0;
const double FACT_EPSI         = 1000.0;     // Penalize angle error more than cross track error. This improved Driving at higher speeds.
const double FACT_V            = 0.1;
const double FACT_DELTA        = 1.0;
const double FACT_A            = 0.0;       // Penalize acceleration less. This improved driving at higher speeds by allowing car to apply the breaks.
const double FACT_DELTA_CHANGE = 100000.0;     // Penalize steering angle changes more. This enforces smoother curves.
const double FACT_A_CHANGE     = 0.0;       // Penalize acceleration changes less. This improved driving at higher speeds by allowing car to apply the breaks.penalize acceleration less. Allow breaking.

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs_;
  double ref_v_;
  FG_eval(Eigen::VectorXd coeffs, const double ref_v) {
    coeffs_ = coeffs;
    ref_v_  =  ref_v;
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // The part of the cost based on the reference state.
    for (size_t t = 0; t < N; t++) {
      fg[0] += FACT_CTE  * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += FACT_EPSI * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += FACT_V    * CppAD::pow(vars[v_start + t] - ref_v_, 2);
    }

    // Minimize the use of actuators.
    for (size_t t = 0; t < N - 1; t++) {
      fg[0] += FACT_DELTA * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += FACT_A     * CppAD::pow(vars[a_start + t], 2); // Acceleration & Breaking
    }

    // Minimize the value gap between sequential actuations.
    for (size_t t = 0; t < N - 2; t++) {
      fg[0] += FACT_DELTA_CHANGE * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += FACT_A_CHANGE     * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    //
    // Setup Constraints
    //
    // NOTE: In this section you'll setup the model constraints.

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (size_t t = 1; t < N; t++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      AD<double> xp1 = x0 + v0 * CppAD::cos(psi0) * dt;
      AD<double> yp1 = coeffs_[0] + coeffs_[1]*xp1 + coeffs_[2]*xp1*xp1 + coeffs_[3]*xp1*xp1*xp1;// 3rd degree polynomial evaluated at predicted value of x1
      AD<double> epsp1 = CppAD::atan(coeffs_[1] + 2.0*coeffs_[2]*xp1 + 3.0*coeffs_[3]*xp1*xp1); // angle calculated from 3rd degree polynomial derivate evaluated at predicted value of x1

      fg[1 + x_start + t]   = x1    - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t]   = y1    - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1  - (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_start + t]   = v1    - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1  - (yp1 - (y0 + v0 * CppAD::sin(psi0) * dt));
      fg[1 + epsi_start + t]= epsi1 - ((psi0 + v0 * delta0 / Lf * dt) - epsp1);

    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC(const double steer_angle_limit, const double reference_velocity) {
  steer_angle_limit_  = steer_angle_limit;
  reference_velocity_ = reference_velocity;
}

MPC::~MPC() {}

bool MPC::Solve(Eigen::VectorXd x0, Eigen::VectorXd coeffs) {
  bool ok = true;

  const double x    = x0[0];
  const double y    = x0[1];
  const double psi  = x0[2];
  const double v    = x0[3];
  const double cte  = x0[4];
  const double epsi = x0[5];

  // number of independent variables
  // N timesteps == N - 1 actuations
  size_t n_vars = N * 6 + (N - 1) * 2;
  // Number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }
  // Set the initial variable values
  vars[x_start]    = x;
  vars[y_start]    = y;
  vars[psi_start]  = psi;
  vars[v_start]    = v;
  vars[cte_start]  = cte;
  vars[epsi_start] = epsi;

  // Lower and upper limits for variables
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // All non-actuators upper and lowerlimits
  for (size_t i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] =  1.0e19;
  }

  // The upper and lower limits of steering angle delta.
  for (size_t i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -steer_angle_limit_;
    vars_upperbound[i] =  steer_angle_limit_;
  }

  // The acceleration/decceleration upper and lower limits.
  for (size_t i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] =  1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start]    = x;
  constraints_lowerbound[y_start]    = y;
  constraints_lowerbound[psi_start]  = psi;
  constraints_lowerbound[v_start]    = v;
  constraints_lowerbound[cte_start]  = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start]    = x;
  constraints_upperbound[y_start]    = y;
  constraints_upperbound[psi_start]  = psi;
  constraints_upperbound[v_start]    = v;
  constraints_upperbound[cte_start]  = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs, reference_velocity_);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  options += "Integer print_level  0\n"; // more print information
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution_);

  // Check some of the solution values
  ok &= solution_.status == CppAD::ipopt::solve_result<Dvector>::success;
  /*
  if (ok)
    std::cout << "Solver Succesfull " << std::endl;
  else
    std::cout << "Solver Failed " << std::endl;

  // Cost
  auto cost = solution_.obj_value;
  std::cout << "Cost " << cost << std::endl;
  */

  return ok;
}

double MPC::get_steer_angle(){
  return solution_.x[delta_start];
}

double MPC::get_throttle(){
  return solution_.x[a_start];
}

vector<double> MPC::get_x_vals(){
  return get_vals(x_start);
}

vector<double> MPC::get_y_vals(){
  return get_vals(y_start);
}

vector<double> MPC::get_vals(int start){
  vector<double> vals(N);
  for (size_t i=0; i<N; ++i){
    vals[i] = solution_.x[start+i];
  }
  return vals;
}
