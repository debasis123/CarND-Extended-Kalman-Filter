/*
* @Original Author: Udacity
* @Last Modified by:   debasis123
*/

#include <uWS/uWS.h>
#include "json.hpp"
#include "FusionEKF.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>
#include <string>

using namespace std;
using namespace uWS;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// for convenience
using json = nlohmann::json;

//////////////////////
// Helper functions //
//////////////////////

/**
 * Check if the SocketIO event has JSON data.
 * If there is data, the JSON object in string format will be returned,
 * else the empty string will be returned.
 */
static std::string hasIncomingDataFromSimulator(const std::string& s) {
  const auto found_null = s.find("null");
  const auto b1         = s.find_first_of("[");
  const auto b2         = s.find_first_of("]");

  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2-b1+1);
  }
  return "";
}

/**
 * Gets the laser data from the simulator
 */
static MeasurementPackage getLaserData(istringstream& iss) {
  MeasurementPackage meas;
  meas.sensor_type_ = MeasurementPackage::SensorType::LASER;
  meas.raw_measurements_ = VectorXd(2);
  double px;    iss >> px;
  double py;    iss >> py;
  meas.raw_measurements_ << px, py;
  long long timestamp;  iss >> timestamp;
  meas.timestamp_ = timestamp;

  return meas;
}

/**
 * Gets radar data from the simulator
 */
static MeasurementPackage getRadarData(istringstream& iss) {
  MeasurementPackage meas;
  meas.sensor_type_ = MeasurementPackage::SensorType::RADAR;
  meas.raw_measurements_ = VectorXd(3);
  double rho;        iss >> rho;
  double theta;      iss >> theta;
  double rho_dot;    iss >> rho_dot;
  meas.raw_measurements_ << rho, theta, rho_dot;
  long long timestamp;  iss >> timestamp;
  meas.timestamp_ = timestamp;

  return meas;
}

/**
 * Gets the ground truth values (to be required to compute RMSE)
 */
static VectorXd getGroundTruthValues(istringstream& iss) {
  VectorXd gt_values(4);
  double x_gt;      iss >> x_gt;      gt_values(0) = x_gt;
  double y_gt;      iss >> y_gt;      gt_values(1) = y_gt;
  double vx_gt;     iss >> vx_gt;     gt_values(2) = vx_gt;
  double vy_gt;     iss >> vy_gt;     gt_values(3) = vy_gt;

  return gt_values;
}

/**
 * Computes estimated values from EKF
 */
static VectorXd getEstimatedValues(const FusionEKF& fusionEKF) {
  VectorXd estimate(4);
  double p_x = fusionEKF.ekf_.x_(0);       estimate(0) = p_x;
  double p_y = fusionEKF.ekf_.x_(1);       estimate(1) = p_y;
  double v1  = fusionEKF.ekf_.x_(2);       estimate(2) = v1;
  double v2  = fusionEKF.ekf_.x_(3);       estimate(3) = v2;

  return estimate;
}

/**
 * Computes data to be fed back to the simulator from C++
 */
static json getFeedbackForSimulator(const VectorXd& estimate, const VectorXd& RMSE) {
  // kalman filter estimated position x and y
  json msgJson;
  msgJson["estimate_x"] = estimate(0);
  msgJson["estimate_y"] = estimate(1);
  msgJson["rmse_x"]     = RMSE(0);
  msgJson["rmse_y"]     = RMSE(1);
  msgJson["rmse_vx"]    = RMSE(2);
  msgJson["rmse_vy"]    = RMSE(3);

  return msgJson;
}


/////////////////////////////////////
// Main entry point of the project //
/////////////////////////////////////

/**
 * Performs the following:
 * 1. reads in the sensor data measurements line by line from the Simulator and stores in a measurement object
 * 2. calls Kalman filter on the data to get estimated positions
 * 3. calls a function to calculate RMSE based on estimated positions and truth values
 * 4. send estimate values and RMSE values back to simulator to print on screen
 */
int main()
{
  uWS::Hub h;

  // Create a Fusion Extended Kalman Filter instance
  FusionEKF fusionEKF;

  // used to compute the RMSE later
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truths;

  ///////////////////////////////////////////////////////
  // BELOW WE DEFINE A SERIES OF FUNCTION OBJECTS on h //
  ///////////////////////////////////////////////////////

  h.onMessage(
    [&fusionEKF, &estimations, &ground_truths] (uWS::WebSocket<uWS::SERVER> ws,
                                                       char *data,
                                                       size_t length,
                                                       uWS::OpCode opCode) {
      ////////////////////////////////////////////////////////
      // Check if there is incoming data from the simulator //
      // Read the simulation data and parse, if any         //
      ////////////////////////////////////////////////////////

      // "42" at the start of the message means there's a websocket message event.
      // The 4 signifies a websocket message
      // The 2 signifies a websocket event

      if (length > 2 && data[0] == '4' && data[1] == '2') {
        auto s = hasIncomingDataFromSimulator(std::string(data));

        if (!s.empty()) {
          auto j = json::parse(s);
          const std::string event = j[0].get<std::string>();
          if (event == "telemetry") {
            // j[1] is the data JSON object
            // INPUT: values provided by the simulator to the c++ program
            // the measurement that the simulator observed (either lidar or radar)
            string sensor_measurment = j[1]["sensor_measurement"];
            MeasurementPackage meas;
            istringstream iss(sensor_measurment);

        	  // read first element from the current line to determine whether it's a Lidar or Radar data
        	  string sensor_type;
        	  iss >> sensor_type;
        	  if (sensor_type.compare("L") == 0) {
              meas = getLaserData(iss);
            }
            else if (sensor_type.compare("R") == 0) {
              meas = getRadarData(iss);
            }

            ///////////////////////////////////////////////////
            // Call the EKF-based fusion on measurement data //
            ///////////////////////////////////////////////////

            fusionEKF.ProcessMeasurement(meas);


            //////////////////////////////////////////////////
            // Compute the RMSE values and Estimated values //
            //////////////////////////////////////////////////

            // ground truth values
            auto gt_values = getGroundTruthValues(iss);
            ground_truths.push_back(gt_values);

        	  // Push the current estimated x,y positon from the Kalman filter's state vector
        	  auto estimate = getEstimatedValues(fusionEKF);
        	  estimations.push_back(estimate);

            // compute RMSE for comparison
        	  auto RMSE = Tools::CalculateRMSE(estimations, ground_truths);


            ////////////////////////////////////////////////////
            // feeback RMSE values and estimated data from KF //
            // back to simulator to print on screen           //
            ////////////////////////////////////////////////////

            auto msgJson = getFeedbackForSimulator(estimate, RMSE);
            std::string msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
            // std::cout << msg << std::endl;
            // ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            ws.send(msg.data(), uWS::OpCode::TEXT);
          }
        }
        // o/w, the SocketIO event has no JSON data.
        else {
          std::string msg = "42[\"manual\",{}]";
          // ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          ws.send(msg.data(), uWS::OpCode::TEXT);
        }
      }
    }
  ); // end function object

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest(
    [](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t length, size_t remainingBytes) {
      const std::string s = "<h1>Hello world!</h1>";
      if (req.getUrl().valueLength == 1) {
        res->end(s.data(), s.length());
      } else {
        // i guess this should be done more gracefully?
        res->end(nullptr, 0);
      }
    }
  );

  h.onConnection(
    [](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
      std::cout << "Connected!!!" << std::endl;
    }
  );

  h.onDisconnection(
    [](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
      ws.close();
      std::cout << "Disconnected" << std::endl;
    }
  );

  const uint16_t port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
} // end main
