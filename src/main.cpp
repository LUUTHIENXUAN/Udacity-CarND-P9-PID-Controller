#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, char *argv[])
{
  uWS::Hub h;

  PID pid; // pid for steering angle
  // TODO: Initialize the pid variable.
  std::cout << "Initializing PID for steering angle..." << std::endl;

  double init_Kp = 0.15; // atof(argv[1])
  double init_Ki = 0.0;  // atof(argv[2])
  double init_Kd = 0.75; // atof(argv[3])
  pid.Init(init_Kp, init_Ki, init_Kd);
  // Twiddle
  int current_time_step = 0; //current_time_step

  PID pids; // pid for speed
  std::cout << "Initializing PID for speed..." << std::endl;
  pids.Init(0.1, 0, 0);

  const double reference_speed = 200.0;

  h.onMessage([&pid, &current_time_step, &pids, &reference_speed]
              (uWS::WebSocket<uWS::SERVER> ws, char *data,
                                           size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte   = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value, throttle_value;

         /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          if (steer_value >  1) steer_value =  1;
          if (steer_value < -1) steer_value = -1;

          current_time_step += 1;
          double error = pid.run(cte, current_time_step);

          if (current_time_step == pid.max_time_step &&
              !pid.twiddle_completed){

            current_time_step = 0; //reset to 0
            double final_err  = error;
            pid.twiddle(0.01, final_err);

          }
          //assume that preference_cte and preference_angle as 0;
          double total_cost = 0;
          total_cost       += speed - reference_speed; //dealing with stopping
          total_cost       += 0.001 *(steer_value - angle)
                                    *(steer_value - angle)
                                    *speed;
          //total_cost       += 0.5 * abs(cte)*abs(cte);
          total_cost       += 0.001 * error*speed;
          total_cost       += 0.05  * abs(angle)*abs(angle)*speed;
          total_cost       += 0.001 * abs(angle)*abs(cte)*speed;
          pids.UpdateError(total_cost);
          throttle_value = pids.TotalError();

          if (throttle_value >  1) throttle_value =  1;
          if (throttle_value < -1) throttle_value = -1;


          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"]       = throttle_value ;//0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          //std::string reset_msg = "42[\"reset\",{}]";
          //ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);

        }

      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)){
    std::cout << "Listening to port " << port << std::endl;
  }
  else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();

}
