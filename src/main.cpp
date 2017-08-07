#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// For converting back and forth between mph and m/s
double mph2mps(double x) { return x * 0.44704; }
double mps2mph(double x) { return x / 0.44704; }

const double STEER_ANGLE_LIMIT  = deg2rad(25);
const double REFERENCE_VELOCITY = mph2mps(40); // Target with this reference speed

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc(STEER_ANGLE_LIMIT, REFERENCE_VELOCITY);

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"]; // global x positions of the waypoints in m
          vector<double> ptsy = j[1]["ptsy"]; // global y positions of the waypoints in m
          double px = j[1]["x"];              // global x positions of the vehicle in m
          double py = j[1]["y"];              // global y positions of the vehicle in m
          double psi = j[1]["psi"];           // vehicle orientation in radians
          double v = j[1]["speed"];           // current velocity in mph
          double delta = j[1]["steering_angle"]; // current steering angle in radians
          double a     = j[1]["throttle"];       // current acceleration in m/s2

          const double latency= 0.1; // 100 msec
          const double Lf     = 2.67; // Valid only for our current car!

          // Within MPC, work in consistent SI units. Convert velocity to m/s
          v = mph2mps(v);

          // Calculate position & orientation of car at end of latency period
          // We need to be accurate at high speeds, so integrate it over the latency period
          // With constant acceleration and steering angle.

          double px_lat = px;  // global car x-location  after latency period
          double py_lat = py;  // global car y-location  after latency period
          double psi_lat= psi;  // global car orientation after latency period
          double v_lat  = v;
          px_lat += v_lat*cos(psi_lat)*latency;
          py_lat += v_lat*sin(psi_lat)*latency;
          psi_lat-= v_lat*delta*latency/Lf;
          v_lat  += a*latency;

          // Convert waypoint coordinates into local car coordinate system after latency period
          Eigen::VectorXd ptsx_c(ptsx.size());
          Eigen::VectorXd ptsy_c(ptsx.size());
          for (size_t i=0; i<ptsx.size(); ++i){
            const double dx = ptsx[i] - px_lat;
            const double dy = ptsy[i] - py_lat;
            ptsx_c[i] =  dx*cos(psi_lat) + dy*sin(psi_lat);
            ptsy_c[i] = -dx*sin(psi_lat) + dy*cos(psi_lat);
          }

          // Fit a 3rd order polynomial through the waypoints in local car coordinate system
          auto coeffs = polyfit(ptsx_c, ptsy_c, 3);

          // Initialize state of the car in local car coodinate system after latency period
          double x_c    = 0.0;
          double y_c    = 0.0;
          double psi_c  = 0.0;
          double v_c    = v_lat;
          double cte_c  = coeffs[0];
          double epsi_c = -atan(coeffs[1]);

          Eigen::VectorXd state_c(6);
          state_c << x_c, y_c, psi_c, v_c, cte_c, epsi_c;

          // Solve it
          mpc.Solve(state_c, coeffs);

          // Send result back to simulator
          json msgJson;

          // Actuators
          msgJson["steering_angle"] = -mpc.get_steer_angle()/STEER_ANGLE_LIMIT; // Reverse into Unity angle convention & scale to expected range of [-1,1]
          msgJson["throttle"] = mpc.get_throttle();

          // To display the MPC predicted trajectory by a green line
          // Note: Points are in reference to the vehicle's coordinate system
          msgJson["mpc_x"] = mpc.get_x_vals();
          msgJson["mpc_y"] = mpc.get_y_vals();

          // To display the waypoints by a yellow line
          // Note: Points are in reference to the vehicle's coordinate system
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          for (int i=0; i<ptsx_c.size(); ++i){
            next_x_vals.push_back(ptsx_c[i]);
            next_y_vals.push_back(ptsy_c[i]);
          }
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
