#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "particle_filter.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;


string hasData(string s) {
/* ******
  Checks if the SocketIO event has JSON data.
  If there is data the JSON object in string format will be returned,
  else the empty string "" will be returned.
******* */
  
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  //Create instance to connect to simulator
  uWS::Hub h;

  // Set up parameters here
  double delta_t = 0.1;  // Time elapsed between measurements [sec]
  double sensor_range = 50;  // Sensor range [m]

  // GPS and Movement uncertainty [x [m], y [m], theta [rad]]
  double sigma_pos [3] = {0.3, 0.3, 0.01};
  // Landmark measurement uncertainty [x [m], y [m]]
  double sigma_landmark [2] = {0.3, 0.3};

  // Read map data
  Map map;
  if (!read_map_data("../data/map_data.txt", map)) {
    std::cout << "Error: Could not open map file" << std::endl;
    return -1;
  }

  // Create particle filter
  ParticleFilter pf;

  h.onMessage([&pf,&map,&delta_t,&sensor_range,&sigma_pos,&sigma_landmark]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
               uWS::OpCode opCode) {
    // Read Client (simulator) input Info to process request 
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
                
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data));

      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // ---- UPDATE MOTION STATE OF PARTICLES ----
          // Initialization
          if (!pf.initialized()) { 
            // Sense noisy position data from the simulator (Create particles around Vehicle's GPS positioning)
            double sense_x     = std::stod(j[1]["sense_x"].get<string>());
            double sense_y     = std::stod(j[1]["sense_y"].get<string>());
            double sense_theta = std::stod(j[1]["sense_theta"].get<string>());

            pf.init(sense_x, sense_y, sense_theta, sigma_pos);
            std::cout<< "[INFO] Initialization Complete...\n";
          }
          
          //Prediction 
          else {
            // Predict the vehicle's next state from previous (noiseless control) data.
            double previous_velocity = std::stod(j[1]["previous_velocity"].get<string>());
            double previous_yawrate  = std::stod(j[1]["previous_yawrate"].get<string>());
            
            std::cout<< "Before Prediction\n";
            pf.prediction(delta_t, sigma_pos, previous_velocity, previous_yawrate);
            std::cout<< "Prediction\n";
          }

          
          // ---- SENSOR MEASUREMENT OBSERVATIONS OF LANDMARKS (from robot not particles) ----
          // receive noisy observation data from the simulator sense_observations in JSON format 
          //   [{obs_x,obs_y},{obs_x,obs_y},...{obs_x,obs_y}] 
          string sense_observations_x = j[1]["sense_observations_x"]; 
          string sense_observations_y = j[1]["sense_observations_y"];

          // All x positions of landmarks from vehicle perspective in float vector
          vector<float> x_sense;
          std::istringstream iss_x(sense_observations_x);
          std::copy(std::istream_iterator<float>(iss_x),
          std::istream_iterator<float>(),
          std::back_inserter(x_sense));

           // All y positions of landmarks from vehicle perspective in float vector
          vector<float> y_sense;
          std::istringstream iss_y(sense_observations_y);
          std::copy(std::istream_iterator<float>(iss_y),
          std::istream_iterator<float>(),
          std::back_inserter(y_sense));

          // Create vector of observed landmarks
          vector<LandmarkObs> noisy_observations;
          for (size_t i = 0; i < x_sense.size(); ++i) {
            LandmarkObs obs;
            obs.x = x_sense[i];
            obs.y = y_sense[i];
            noisy_observations.push_back(obs);
          }
          std::cout<< "Observation\n";

          //---- ASSOCIATE OBSERVATIONS TO PARTICLES AND UPDATE PARTICLES ---
          // Associate particles' transformed observations to landmarks, calculate weights, and resample
          pf.updateWeights(sensor_range, sigma_landmark, noisy_observations, map);
          std::cout<< "UpdateWeights\n";
          pf.resample();
          std::cout<< "Resample\n";

          
          //---- EVALUATE PARTICLE FILTER ----
          //Define variables for evaluation
          vector<Particle> particles = pf.particles;
          int num_particles = particles.size();
          double highest_weight = -1.0;
          Particle best_particle;
          double weight_sum = 0.0;
          
          // Calculate and output the average weighted error and best particle at each iteration
          for (int i = 0; i < num_particles; ++i) {
            if (particles[i].weight > highest_weight) {
              highest_weight = particles[i].weight;
              best_particle = particles[i];
            }
            weight_sum += particles[i].weight;
          }
          std::cout << "highest w " << highest_weight << std::endl;
          std::cout << "average w " << weight_sum/num_particles << std::endl;

          //Create json file to send back to simulator (the client)
          json msgJson;
          msgJson["best_particle_x"]     = best_particle.x;
          msgJson["best_particle_y"]     = best_particle.y;
          msgJson["best_particle_theta"] = best_particle.theta;
          //msgJson["best_particle_associations"] = pf.getAssociations(best_particle);
          //msgJson["best_particle_sense_x"] = pf.getSenseCoord(best_particle, "X");
          //msgJson["best_particle_sense_y"] = pf.getSenseCoord(best_particle, "Y");
          auto msg = "42[\"best_particle\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          std::cout<< "Evaluation\n";
        
        }  // end "telemetry" if
      
        else {
          string msg = "42[\"manual\",{}]";
          std::cout<< "Outside Telemetry\n";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }      
      }// end of string is not empty
      else{
        std::cout<< "String is empty\n";
      }
    }  // end websocket message if
    else{
    std::cout<< "No 42 WebSocket Message\n";
    }
  }); // end h.onMessage

  
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