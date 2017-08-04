#ifndef MPC_H
#define MPC_H

#include <vector>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;
using namespace std;

typedef CPPAD_TESTVECTOR(double) Dvector;

class MPC {
 public:
  MPC(const double steer_angle_limit, const double reference_velocity);

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  bool Solve(Eigen::VectorXd x0, Eigen::VectorXd coeffs);

  // Provide access to the mpc solution
  double get_steer_angle();
  double get_throttle();
  vector<double> get_x_vals();
  vector<double> get_y_vals();

 private:
  double steer_angle_limit_;
  double reference_velocity_;
  // place to store solution of solver
  CppAD::ipopt::solve_result<Dvector> solution_;
  // function to easily extract values from the solution.x vector
  vector<double> get_vals(int start);
};

#endif /* MPC_H */
