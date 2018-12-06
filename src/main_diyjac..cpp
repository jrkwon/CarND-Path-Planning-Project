#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

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
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }
  return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);
  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];
  double heading = atan2( (map_y-y),(map_x-x) );
  double angle = abs(theta-heading);

  if(angle > pi()/4)
  {
    closestWaypoint++;
    if (closestWaypoint >= maps_x.size())
      closestWaypoint = 0;
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}


// Transform from global Cartesian x,y to local car coordinates x,y
// where x is pointing to the positive x axis and y is deviation from the car's path
vector<double> getLocalXY(double car_x, double car_y, double theta, double wx, double wy) {
  vector<double> results;

  // convert to local coordinates
  float deltax = (wx - car_x);
  float deltay = (wy - car_y);
  results.push_back(deltax*cos(theta) + deltay*sin(theta));
  results.push_back(-deltax*sin(theta) + deltay*cos(theta));
  return results;
}

vector<double> getWorldXY(double car_x, double car_y, double theta, double lx, double ly) {
  vector<double> results;

  // convert back to global coordinates
  results.push_back(lx*cos(theta) - ly*sin(theta) + car_x);
  results.push_back(lx*sin(theta) + ly*cos(theta) + car_y);
  return results;
}

// segment of 25 x,y coordinates for waypoints (6 in the back, 1 closest and 18 in the front) for a given lane
vector<vector<double>> getLocalizedWayPointSegement(double car_x, double car_y, double car_yaw, double d, vector<double> maps_x, vector<double> maps_y, vector<double> maps_dx, vector<double> maps_dy) {
  vector<double> wpx;
  vector<double> wpy;
  vector<vector<double>> results;
  double theta = deg2rad(car_yaw);

  int closestWaypoint = ClosestWaypoint(car_x, car_y, maps_x, maps_y);
  int previous = closestWaypoint - 6;
  if (previous < 0) {
    previous += maps_x.size();
  }
  // cout << "waypoints: ";
  for (int i = 0; i < 25; i++) {
    int next = (previous+i)%maps_x.size();
    vector<double> localxy = getLocalXY(car_x, car_y, theta, (maps_x[next]+d*maps_dx[next]), (maps_y[next]+d*maps_dy[next]));
    // cout << next << ":" << localxy[0] << ":" << localxy[1] << ",";
    wpx.push_back(localxy[0]);
    wpy.push_back(localxy[1]);
  }
  // cout << endl;
  results.push_back(wpx);
  results.push_back(wpy);

  return results;
}

// convert a set of world x,y vector coordinates to local x y vectors
vector<vector<double>> getLocalizedPoints(double car_x, double car_y, double car_yaw, vector<double> wx, vector<double> wy) {
  vector<double> lx;
  vector<double> ly;
  vector<vector<double>> results;
  double theta = deg2rad(car_yaw);

  for (int i = 0; i < wx.size(); i++) {
    vector<double> localxy = getLocalXY(car_x, car_y, theta, wx[i], wy[i]);
    lx.push_back(localxy[0]);
    ly.push_back(localxy[1]);
  }
  results.push_back(lx);
  results.push_back(ly);

  return results;
}

// convert a set of local x,y vector coordinates to world x y vectors
vector<vector<double>> getWorldPoints(double car_x, double car_y, double car_yaw, vector<double> lx, vector<double> ly) {
  vector<double> wx;
  vector<double> wy;
  vector<vector<double>> results;
  double theta = deg2rad(car_yaw);

  for (int i = 0; i < lx.size(); i++) {
    vector<double> worldxy = getWorldXY(car_x, car_y, theta, lx[i], ly[i]);
    wx.push_back(worldxy[0]);
    wy.push_back(worldxy[1]);
  }
  results.push_back(wx);
  results.push_back(wy);

  return results;
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // for our spline fit
  vector<double> vx;
  vector<double> vy;
  vector<double> vd;
  vector<double> xyd;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double nextd = 6.;
  // max speed ~ 49.75MPH
  const double inc_max = 0.445;
  double dist_inc = inc_max;
  int timestep = 0;
  int stucktimer = 0;
  bool lanechange = false;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    double s;
    double d_x;
    double d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // set up logging
  string log_file = "../data/logger.csv";
  ofstream out_log(log_file.c_str(), ofstream::out);
  out_log << "t,x,y,vd,xyd,nd,d,st" << endl;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_dx,&map_waypoints_dy,&vx,&vy,&vd,&xyd,&nextd,&inc_max,&dist_inc,&out_log,&timestep,&stucktimer,&lanechange]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
          double angle = deg2rad(car_yaw);

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> lx;
          vector<double> ly;
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          int path_size = previous_path_x.size();
          int num_points = 50;

          // for speed and steering/lane change control
          vector<double> t;
          vector<double> t2;
          vector<double> inc;
          vector<double> nd;

          tk::spline smooth_lanes;
          tk::spline smooth_speed;
          tk::spline smooth_local;

          // get our frenet coordinates
          vector<double> frenet = getFrenet(car_x, car_y, deg2rad(car_yaw), map_waypoints_x, map_waypoints_y);

          // set up lane tracking using spline
          vector<vector<double>> localwxy = getLocalizedWayPointSegement(car_x, car_y, car_yaw, nextd, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy);
          // wrong way!
          if (localwxy[0][0] > 0.) {
            car_yaw += 180;
            cout << "wrong direction detected! car x,y,yaw: " << car_x << "," << car_y << "," << car_yaw-180 << " new yaw: " << car_yaw << endl;
            localwxy = getLocalizedWayPointSegement(car_x, car_y, car_yaw, nextd, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy);
          }
          smooth_lanes.set_points(localwxy[0], localwxy[1]);

          // at beginning - no paths
          if (path_size == 0)
          {
            t.push_back(double(-1));
            t.push_back(double(15));
            t.push_back(double(25));
            t.push_back(double(num_points));
            t.push_back(double(num_points*2));
            inc.push_back(dist_inc*0.01);
            inc.push_back(dist_inc*0.10);
            inc.push_back(dist_inc*0.15);
            inc.push_back(dist_inc*0.25);
            inc.push_back(dist_inc*0.35);
            inc.push_back(dist_inc);
            smooth_speed.set_points(t,inc);

            double nextlwpx = 0.;
            double nextlwpy;
            for (int i = 0; i<num_points; i++)
            {
              nextlwpx += smooth_speed(double(i));
              nextlwpy = smooth_lanes(nextlwpx);
              lx.push_back(nextlwpx);
              ly.push_back(nextlwpy);
              if (i > 0)
                vd.push_back(distance(lx[i-1], ly[i-1], lx[i], ly[i]));
              else
                vd.push_back(smooth_speed(double(0)));
            }

            // calculate the smoother path
            double localxx = 0.;
            double localxy = 0.;
            for(int i = 0; i < num_points; i++)
            {
              ly[i] = smooth_lanes(lx[i]);
              double dist = distance(localxx, localxy, lx[i], ly[i]);
              double speed = smooth_speed(double(i));
              if (dist > speed || dist < speed*0.8)
              {
                 double heading = atan2(ly[i]-localxy,lx[i]-localxx);
                 lx[i] = localxx + smooth_speed(double(i))*cos(heading);
                 ly[i] = smooth_lanes(lx[i]);
                 dist = distance(localxx, localxy, lx[i], ly[i]);
              }
              localxx = lx[i];
              localxy = ly[i];
            }

            vector<vector<double>> worldxy = getWorldPoints(car_x, car_y, car_yaw, lx, ly);
            for (int i=path_size; i<worldxy[0].size(); i++) {
              vx.push_back(worldxy[0][i]);
              vy.push_back(worldxy[1][i]);
              next_x_vals.push_back(worldxy[0][i]);
              next_y_vals.push_back(worldxy[1][i]);
            }
            out_log << timestep << "," << car_x << "," << car_y << ","  << "0,0," << nextd << "," << frenet[1] << "," << stucktimer << std::endl;

          // we are already moving...
          } else {
            vector<vector<double>> previous_localxy = getLocalizedPoints(car_x, car_y, car_yaw, previous_path_x, previous_path_y);
            lx = previous_localxy[0];
            ly = previous_localxy[1];

            for (int i = 0; i < (num_points-path_size); i++)
            {
              vector<double> frenet = getFrenet(vx[i], vy[i], deg2rad(car_yaw), map_waypoints_x, map_waypoints_y);
              out_log << timestep << "," << vx[i] << "," << vy[i] << "," << vd[i] << "," << xyd[i] << "," << nextd << "," << frenet[1] << "," << stucktimer << std::endl;
            }
            vx.erase(vx.begin(),vx.begin()+(num_points-path_size));
            vy.erase(vy.begin(),vy.begin()+(num_points-path_size));
            vd.erase(vd.begin(),vd.begin()+(num_points-path_size));
            xyd.erase(xyd.begin(),xyd.begin()+(num_points-path_size));

            // if we are changing lanes
            if (lanechange && abs(smooth_lanes(0.)) < 0.01) {
              lanechange = false;
            }

            // make a smoother waypoint polyline
            vector<vector<double>> newwxy = getLocalizedWayPointSegement(car_x, car_y, car_yaw, nextd, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy);
            if (newwxy[0][0] > 0.) {
              car_yaw += 180;
              cout << "wrong direction detected! car x,y,yaw: " << car_x << "," << car_y << "," << car_yaw-180 << " new yaw: " << car_yaw << endl;
              newwxy = getLocalizedWayPointSegement(car_x, car_y, car_yaw, nextd, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy);
            }
            tk::spline newlane;
            newlane.set_points(newwxy[0], newwxy[1]);
            vector<double> localwx;
            vector<double> localwy;
            for (int i; i<path_size; i++) {
              localwx.push_back(lx[i]);
              localwy.push_back(ly[i]);
            }
            double nextx = lx[path_size-1]+40;
            for (int i; i<path_size; i++) {
              localwx.push_back(nextx);
              localwy.push_back(newlane(nextx));
              nextx += dist_inc;
            }
            smooth_lanes.set_points(localwx, localwy);

            for(int i = 0; i < path_size; i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            t.push_back(0.);
            t.push_back(double(250));
            if (vd[0] < inc_max) {
              inc.push_back(vd[0]);
            } else {
              inc.push_back(vd[path_size-1]);
            }
            inc.push_back(dist_inc);
            smooth_speed.set_points(t,inc);

            std::cout << "x,y: " << car_x << "," << car_y << " Time=[" << timestep <<":" << stucktimer << "] localy=[" << smooth_lanes(0.) << "], frenet_d=[" << frenet[1] << "]" << std::endl;

            // filler
            for(int i = path_size; i<num_points; i++) {
              lx.push_back(lx[i-1]+smooth_speed(double(i)));
              ly.push_back(smooth_lanes(lx[i]));
              vx.push_back(0.0);
              vy.push_back(0.0);
              next_x_vals.push_back(0.0);
              next_y_vals.push_back(0.0);
            }

            // calculate the smoother path
            double localxx = lx[0];
            double localxy = ly[0];
            for(int i = 0; i < num_points; i++)
            {
              ly[i] = smooth_lanes(lx[i]);
              double dist = distance(localxx, localxy, lx[i], ly[i]);
              if (dist > smooth_speed(double(i)))
              {
                 double heading = atan2(ly[i]-localxy,lx[i]-localxx);
                 lx[i] = localxx + smooth_speed(double(i))*cos(heading);
                 ly[i] = smooth_lanes(lx[i]);
                 dist = distance(localxx, localxy, lx[i], ly[i]);
              }
              if (i >= path_size)
                vd.push_back(dist);
              localxx = lx[i];
              localxy = ly[i];
            }

            vector<vector<double>> worldxy = getWorldPoints(car_x, car_y, car_yaw, lx, ly);
            for (int i=path_size; i<worldxy[0].size(); i++) {
              vx[i] = worldxy[0][i];
              vy[i] = worldxy[1][i];
              next_x_vals[i] = worldxy[0][i];
              next_y_vals[i] = worldxy[1][i];
            }

          }

          // planner.
          vector<double> lane1;
          vector<double> lane2;
          vector<double> lane3;
          vector<vector<double>> lanes;
          int ourlane = round(round(nextd-2)/4);
          int bestlane = ourlane;
          lanes.push_back(lane1);
          lanes.push_back(lane2);
          lanes.push_back(lane3);
          bool badsensor = false;
          int backvehicle_shift = 5;

          // we are slow: need to see more of the back
          //if (dist_inc < 0.35) {
          //  backvehicle_shift = 10;
          //}
          // we are stuck: need to see more of the back
          //if (stucktimer > 1000) {
          //  backvehicle_shift = 15;
          //}
          for (int k = 0; k<sensor_fusion.size(); k++) {
            vector<double> vid = sensor_fusion[k];
            double vidx = vid[1]+vid[3]*0.02;
            double vidy = vid[2]+vid[4]*0.02;
            vector<double> vidlocal = getLocalXY(car_x, car_y, deg2rad(car_yaw), vidx, vidy);
            double viddist = distance(car_x, car_y, vid[1], vid[2]);
            double vids = vidlocal[0] + backvehicle_shift;
            double vidd = vid[6];
            sensor_fusion[k].push_back(vids);
            sensor_fusion[k].push_back(distance(0,0,vid[3],vid[4])*0.02);
            sensor_fusion[k].push_back(round(round(vidd-2)/4));

            // fast vehicle - adjust
            if (smooth_speed(0.) < dist_inc) {
              if (smooth_speed(0.) < vid[8]) {
                vids += (vid[8]-smooth_speed(0.))*120.;
              }
            } else {
              if (dist_inc < vid[8]) {
                vids += (vid[8]-dist_inc)*240.;
              }
            }

            string lanestr = "error";
            if (vids > 0.) {
              if (vidd < 12. && vidd > 0.) {
                if (vidd <= 3.7) {
                  lanestr = "0";
                  cout << "[" << vid[0] << ":(" << vid[5] << ":" << vid[6] << "):(" << vidlocal[0] << ":" << vidlocal[1] << "):" << vids << ":" << viddist << ":" << lanestr << "],";
                  lanes[0].push_back(vids);
                }
                if (vidd > 3.7 && vidd <= 4.3) {
                  lanestr = "0,1";
                  cout << "[" << vid[0] << ":(" << vid[5] << ":" << vid[6] << "):(" << vidlocal[0] << ":" << vidlocal[1] << "):" << vids << ":" << viddist << ":" << lanestr << "],";
                  lanes[0].push_back(vids);
                  lanes[1].push_back(vids);
                }
                if (vidd > 4.3 && vidd <= 7.7) {
                  lanestr = "1";
                  cout << "[" << vid[0] << ":(" << vid[5] << ":" << vid[6] << "):(" << vidlocal[0] << ":" << vidlocal[1] << "):" << vids << ":" << viddist << ":" << lanestr << "],";
                  lanes[1].push_back(vids);
                }
                if (vidd > 7.7 && vidd <= 8.3) {
                  lanestr = "1,2";
                  cout << "[" << vid[0] << ":(" << vid[5] << ":" << vid[6] << "):(" << vidlocal[0] << ":" << vidlocal[1] << "):" << vids << ":" << viddist << ":" << lanestr << "],";
                  lanes[1].push_back(vids);
                  lanes[2].push_back(vids);
                }
                if (vidd > 8.3) {
                  lanestr = "2";
                  cout << "[" << vid[0] << ":(" << vid[5] << ":" << vid[6] << "):(" << vidlocal[0] << ":" << vidlocal[1] << "):" << vids << ":" << viddist << ":" << lanestr << "],";
                  lanes[2].push_back(vids);
                }
              } else {
                badsensor = true;
                cout << "<" << vid[0] << ":(" << vid[5] << ":" << vid[6] << "):(" << vidlocal[0] << ":" << vidlocal[1] << "):" << vids << ":" << viddist << ":" << lanestr << ">,";
              }
            } else {
              cout << "<<" << vid[0] << ":(" << vid[5] << ":" << vid[6] << "):(" << vidlocal[0] << ":" << vidlocal[1] << "):" << vids << ":" << viddist << ":" << lanestr << ">>,";
            }
          }
          cout << endl;

          // sort to find the nearest vehicle in each lane first
          for (int lane = 0; lane<3; lane++) {
            if (lanes[lane].size() > 0) {
              sort(lanes[lane].begin(),lanes[lane].end());
            }
          }

          // stuck! - create a virual car lower than the adjacent car in our lane.
          if (stucktimer > 1000) {
            int newlane = -1;
            for (int lane=0; lane<lanes.size(); lane++) {
              if (newlane < 0 && ourlane != lane && abs(ourlane-lane)==1) {
                newlane = lane;
                for (int i=0; i<sensor_fusion.size(); i++) {
                  vector<double> oldvid = sensor_fusion[i];
                  vector<double> vid;
                  if (oldvid[7] == lanes[newlane][0] && (lanes[newlane][0] > 15 || lanes[newlane][0] < 5)) {
                    lanes[ourlane][0] = lanes[newlane][0]-1.;
                    for (int j=0; j<oldvid.size(); j++) {
                      vid.push_back(oldvid[j]);
                    }
                    vid[7] = lanes[ourlane][0];
                    vid[9] = double(ourlane);
                    sensor_fusion.push_back(vid);
                    cout << "create virual car from lane: " << newlane << " at: " << vid[7] << endl;
                  } else {
                    stucktimer = 0;
                    cout << "Too close to create virual car from lane: " << newlane << " at: " << oldvid[7] << endl;
                  }
                }
              }
            }
          }

          // look at each lane
          for (int lane = 0; lane<3; lane++) {
            // if the lane has vehicles
            if (lanes[lane].size() > 0) {
              // if the current best lane has a nearer vehicle than this lane
              if (lanes[bestlane].size() > 0 && (lanes[bestlane][0] < lanes[lane][0])) {
                // only switch if ourlane has a vehicle less than 80 meters away and is the next lane over.
                if (lanes[ourlane].size() > 0 && lanes[ourlane][0] < 80. && abs(ourlane-lane)==1) {
                  if (abs(ourlane-lane) == 1) {
                    // and it is more than 20 meters away.
                    if (lanes[lane][0] > 20) {
                      bestlane = lane;
                    } else {
                      if (lanes[lane][0] > 5 && stucktimer > 1000) {
                        bestlane = lane;
                      }
                    }
                  //} else {
                  //  if (lanes[1].size() > 1 && lanes[1][0] > 40) {
                  //    // this better be worth it!
                  //    if (lanes[lane][0] > 80) {
                  //      bestlane = lane;
                  //    }
                  //    if (dist_inc < (inc_max*0.9)) {
                  //      dist_inc = vd[path_size-1];
                  //      // dist_inc = vd[0];
                  //      cout << "Changing lane: holding speed spacing to: " << dist_inc << endl;
                  //    } else {
                  //      dist_inc = inc_max*0.9;
                  //      cout << "Changing 2 lanes: decelerating speed spacing to: " << dist_inc << endl;
                  //    }
                  //  }
                  }
                  /*
                  if (dist_inc < inc_max) {
                    // dist_inc = vd[path_size-1];
                    dist_inc = vd[0];
                    cout << "Changing lane: holding speed spacing to: " << dist_inc << endl;
                  }
                  */
                }
              }
            // if the lane is cleared of vehicles
            } else {
              // only switch if ourlane has a vehicle less than 80 meters away and is next lane over.
              if (lanes[ourlane].size() > 0 && lanes[ourlane][0] < 80. && lanes[bestlane].size() > 0 && abs(ourlane-lane)==1) {
                // only change if it is in the next lane
                if (abs(ourlane-lane) == 1) {
                  bestlane = lane;

                // if it is not the next lane, then make sure lane 1 is clear enough for us to get around
                //} else {
                //  if (lanes[1].size() > 1 && lanes[1][0] > 10) {
                //    bestlane = lane;
                //  }
                }
                if (dist_inc < inc_max) {
                  dist_inc = vd[path_size-1];
                }
              }
            }
          }
          int lane0size = lanes[0].size();
          int lane1size = lanes[1].size();
          int lane2size = lanes[2].size();
          float lane0closest = 0;
          float lane1closest = 0;
          float lane2closest = 0;
          if (lane0size > 0) lane0closest = lanes[0][0];
          if (lane1size > 0) lane1closest = lanes[1][0];
          if (lane2size > 0) lane2closest = lanes[2][0];

          cout << "lane0:" << lane0size << ":" << lane0closest << " lane1:" << lane1size << ":" << lane1closest << " lane2:" << lane2size << ":" << lane2closest << " ourlane:" << ourlane << " bestlane:" << bestlane << " inc=[" << dist_inc << ":" << smooth_speed(0) << "]" << endl;
          if (timestep > 50 && ourlane != bestlane) {
            if ( not lanechange and not badsensor ) {
              cout << "ourlane:" << ourlane << " bestlane:" << bestlane << endl;
              nextd = bestlane*4+2;
              if (nextd > 7) {
                nextd -= 0.3;
              }
              lanechange = true;
              stucktimer = 0;
            } else {
              cout << "nextd: " << nextd << " change lane disabled! current position: " << vx[0] << "," << vy[0] << endl;
            }
          }

          // no good way out - the other vehicle is too near - slow down
          if (lanes[ourlane].size() > 0 && lanes[ourlane][0] < 50. && lanes[ourlane][0] > 5.) {
            // no stuck timer if we are in the middle lane.
            if (ourlane != 1) {
              stucktimer++;
            }
            cout << "need to slowdown and match: " << lanes[ourlane][0] << " in lane: " << ourlane << endl;
            for (int i=0; i<sensor_fusion.size(); i++) {
              vector<double> vid = sensor_fusion[i];
              cout << i << " comparing: " << vid[7] <<":" << vid[9] << " with " << lanes[ourlane][0] << "(" << vid[3] << ":" << vid[4] << ":" << vid[8] << ")" << endl;
              if (vid[7] == lanes[ourlane][0] && vid[9] == ourlane && (lanes[ourlane][0] < 40 || (lanes[ourlane][0] < 50 && vid[8] < 0.3))) {
                // follow vehicle
                if (vid[8] > 0.25) {
                  if (dist_inc >= vid[8]) {
                    // vehicle is slower than us... or closer than 50 meters away gradural decelerate to its speed
                    dist_inc = vid[8]*0.95;
                    cout << "decelerating: setting speed spacing to: " << dist_inc << endl;
                  } else {
                    if (vid[7] > 15) {
                      // vehicle is faster than us and is more than 15 meters away
                      // gradurally accelerate to its speed
                      dist_inc = dist_inc + (vid[8] - dist_inc)/2.;
                      cout << "accelerating: setting speed spacing to: " << dist_inc << endl;
                    }
                  }
                } else {
                  cout << "disabled vehicle!" << endl;
                  dist_inc = 0.1;
                  cout << "setting speed spacing to: " << dist_inc << endl;
                }
              }
            }
          } else {
            // slowly increase speed to avoid max acceleration...
            if (dist_inc < inc_max && not lanechange) {
              double interval = inc_max - smooth_speed(0.);
              if (interval > 0.3) {
                double inc1 = (smooth_speed(0.)*9. + inc_max)/10.;
                double inc2 = (dist_inc*9. + inc_max)/10.;
                if (inc1 > dist_inc && inc1 < inc2) {
                  dist_inc = inc1;
                } else {
                  dist_inc = inc2;
                }
              } else if (interval > 0.2) {
                double inc1 = (smooth_speed(0.)*7. + inc_max)/8.;
                double inc2 = (dist_inc*7. + inc_max)/8.;
                if (inc1 > dist_inc && inc1 < inc2) {
                  dist_inc = inc1;
                } else {
                  dist_inc = inc2;
                }
              } else {
                double inc1 = (smooth_speed(0.)*5. + inc_max)/6.;
                double inc2 = (dist_inc*5. + inc_max)/6.;
                if (inc1 > dist_inc && inc1 < inc2) {
                  dist_inc = inc1;
                } else {
                  dist_inc = inc2;
                }
              }
              cout << "No vehicle within range! interval: " << interval << " setting speed spacing to: " << dist_inc << endl;
            }
          }

          vector<double> localx(next_x_vals.size());
          vector<double> localy(next_x_vals.size());
          for (int i=0; i < next_x_vals.size(); i++)
          {
            float next_x = (next_x_vals[i] - car_x);
            float next_y = (next_y_vals[i] - car_y);
            localx[i] = next_x*cos(angle) + next_y*sin(angle);
            localy[i] = -next_x*sin(angle) + next_y*cos(angle);
          }

          // fit a polynomial
          smooth_local.set_points(localx, localy);

          // calculate the smoother path
          double localxx = 0.;
          double localxy = 0.;
          for(int i = 0; i < num_points; i++)
          {
            localy[i] = smooth_local(localx[i]);
            double dist = distance(localxx, localxy, localx[i], localy[i]);
            if (dist > smooth_speed(double(i)))
            {
               double heading = atan2(localy[i]-localxy,localx[i]-localxx);
               localx[i] = localxx + smooth_speed(double(i))*cos(heading);
               localy[i] = smooth_local(localx[i]);
               dist = distance(localxx, localxy, localx[i], localy[i]);
            }
            localxx = localx[i];
            localxy = localy[i];
          }

          // convert back to global coordinates
          for (int i=0; i<num_points; i++)
          {
            next_x_vals[i] = localx[i]*cos(angle) - localy[i]*sin(angle) + car_x;
            next_y_vals[i] = localx[i]*sin(angle) + localy[i]*cos(angle) + car_y;
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          for (int i=path_size; i<num_points; i++)
          {
            if (i > 0)
              xyd.push_back(distance(next_x_vals[i-1], next_y_vals[i-1], next_x_vals[i], next_y_vals[i]));
            else
              xyd.push_back(dist_inc);
          }

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          //this_thread::sleep_for(chrono::milliseconds(500));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          timestep++;        
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