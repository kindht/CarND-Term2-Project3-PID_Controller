#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

#include <numeric>

// for convenience
using nlohmann::json;
using std::string;
using std::cout;
using std::endl;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */
  bool use_twiddle = false;
  bool init_twiddle= true;
  
  cout << "use_twiddle: " << use_twiddle << endl;
  
  double p[] = {0.210001,  0.000271, 3.0}; // Run Final
   //double p[] = {0.217218 , 0.001, 3.0}; // Run OK
  //double p[] = {0.21, 0.0004, 3.081};    // Best Run 
  //double p[] = {0.2, 0.0003, 3.0};       // Run OK mentor suggested   

  // Run twiddle with following(Run OK w/ some hard turn), result {0.217218 , 0.001, 3} , {0.210001,  0.000271, 3.0}
  //double p[] = {0.2, 0.001, 3.0};   
  
  double dp[] = {0.01, 0.001, 0.1};  // result {0.217218 , 0.001, 3} , {0.210001,  0.000271, 3.0}

  pid.Init(p[0], p[1], p[2]);

  h.onMessage([&pid, &p, &dp, &use_twiddle, &init_twiddle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          //double speed = std::stod(j[1]["speed"].get<string>());
          //double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          json msgJson;

          double sum_dp = dp[0] + dp[1] + dp[2];
                                    
          static double err;
          static double best_err;
          static double best_p[3];

          static bool plus;
          static bool minus;

          static int idx; // index of p[] /dp[]
          static int i;   // counter of for loop
          static int total_run;  // total run of loops
    
          const int N = 500;   // num of iterations in one loop  
          const double tol = 0.00001;  // dp converge 
         
          if (use_twiddle) {

            if (init_twiddle) {
              //cout << "init_twiddle" << endl;
              err = 0;
              best_err = std::numeric_limits<double>::infinity();

              plus = true;
              minus = true;
          
              idx = 0;// adjust Kp, Ki, Kd respectively (Kp=0, Ki=1, Kd=2)
              cout << "init_twiddle idx=" << idx << endl;
              total_run = 0;

              init_twiddle = false;         
            } // End if init_twiddle
            total_run += 1;
            cout << "total_run=" << total_run << endl;

             // use current p[] to run N iterations
            for (i = 0 ; i < N ; ++i) {        

                //cout << "i=" << i << endl;                       
                //cout << "cte=" << cte << endl;               
                //cout << "prev err=" << err << endl;
               
                err += pow(cte, 2);
                //cout << "sum err=" << err << endl;

                pid.UpdateError(cte);
                steer_value = pid.TotalError();

                msgJson["steering_angle"] = steer_value;
                msgJson["throttle"] = 0.3;
                auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                //cout << msg << endl;
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              }// End for i              
              err = err / N ;
              cout << "average err=" << err << endl;               
              cout << "sum_dp="   << sum_dp << endl;
              cout << "best_err=" << best_err << endl;
              if ( sum_dp < tol) { 
                  // Stop twiddle
                  use_twiddle = false;

                  cout << "best p[0] p[1] p[2]   = " << best_p[0] <<"  " << best_p[1] <<" " << best_p[2] << endl;
                  pid.Init(best_p[0],best_p[1],best_p[2]);

                  string msg = "42[\"reset\",{}]"; // must use string to declare
                  cout << msg << endl;
                  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              }
              else { // update PID parameters p[] , dp[]
                  Update:
                  if (plus) {
                    cout << "plus" << endl;
                    p[idx] += dp[idx];
                    plus = false;                  
                  }  // check if plus success err < best_err
                  else if (err < best_err && minus == true) {
                    best_err = err;
                    cout <<"plus ok p:"<< p[0] <<","<< p[1]<<","<<p[2] << endl;
                    memcpy(best_p, p, sizeof(p));
                    cout << "best p[0] p[1] p[2]   = " << best_p[0] <<"  " << best_p[1] <<" " << best_p[2] << endl;

                    dp[idx] *= 1.1;
                    plus = true;
                    idx = (idx + 1 ) % 3;   // next p
                    cout << "plusOK idx=" << idx << endl;
                    goto Update;
                  } else if (minus) {
                    cout << "minus" << endl;
                    p[idx] -= 2*dp[idx];
                    //if (p[idx] < 0) { p[idx] = 0; }  // attention this case
                    minus = false;
                  } else { // check if minus success err < best_err
                    if (err < best_err) {
                      best_err = err;
                      memcpy(best_p, p, sizeof(p));

                      dp[idx] *= 1.1;
                      plus  = true;
                      minus = true;
                      idx = (idx + 1 ) % 3;  // next p
                      cout << "minuOK idx=" << idx << endl;
                      goto Update;
                    }                    
                    else {
                      p[idx] += dp[idx];
                      dp[idx] *= 0.9;
                      plus  = true;
                      minus = true;
                      idx = (idx + 1 ) % 3;  // next p
                      cout << "*0.9 idx=" << idx << endl;
                      goto Update;
                    }
                  } // End minus success err < best_err               
              }// End else update parameters

              // Reset
              err = 0;
              i = 0;

              pid.Init(p[0],p[1],p[2]);
              cout << "Reset idx=" << idx << endl;
              cout << "Reset: p[0] p[1] p[2]   = " << p[0] <<"  " << p[1] <<" " << p[2] << endl;

              string msg = "42[\"reset\",{}]"; // must use string to declare
              cout << msg << endl;
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
  
          } // End if use_twiddle

          else { // Not use_twiddle
              pid.UpdateError(cte);
              steer_value = pid.TotalError();
          
              // DEBUG
              //cout << "CTE: " << cte << " Steering Value: " << steer_value << endl;

              msgJson["steering_angle"] = steer_value;
              msgJson["throttle"] = 0.3;
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
              //cout << msg << endl;
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }  // End if "telemetry" 
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // End websocket message
  }); // End h.onMessage

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