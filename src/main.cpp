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

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);







  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
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

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

int lanecheck(double d)
{
  if (d < 4) return 0;
  else if (d < 8) return 1;
  else return 2;
}

enum CarState {KEEP_LANE, Prepfor_CHANGE_LANE, CHANGE_LANE};
double min_distance;  // speed adaptive minimum allowable distance from front car

//*************************Check for clearance with neighboring vehicle*************************//
bool clear_to_pass(double car_s, double car_speed, double other_s, double other_speed)
{
  double spacing = abs(car_s - other_s);
  if (spacing > 6700) spacing = 6945.55 - spacing;  //correction for when near path looping back near s=0

  bool frontObstruction = (other_s > car_s) & (spacing < 30);
  bool frontSlow = (other_s > car_s) & (spacing < 30) & (other_speed < car_speed);
  bool rearObstruction = (other_s < car_s) & (spacing < 30);
  bool rearFast = (other_s < car_s) & (other_speed > (car_speed - 10));

  bool passOK = !(frontObstruction || frontSlow || rearObstruction || rearFast);     //returns 0 when other car obstructing
  cout<<"PassOK?: "<< passOK <<" spacing= "<<spacing<<endl;
  return passOK;
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  int lane = 1;          //start in middle lane
  int target_lane = 1;
  CarState lanestate = KEEP_LANE;
  double ref_vel = 0.3;   //initial non-zero velocity mph to prevent divide by zero errors

  h.onMessage([&lane, &target_lane, &lanestate, &ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s, 
    &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
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
        
        if (event == "telemetry") 
        {
          // j[1] is the data JSON object
          
        	  // Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];         //This in degrees!!
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

            int prev_size = previous_path_y.size();  //number of (un-processed) points remaining in previous_path
            
            if(prev_size > 0) 
              {
              car_s = end_path_s;                   //ego car's s position will be the end of previously sent s value
              }

            bool too_close = false;
            bool OKto_pass = true;
            bool pass_right = ((lane == 1) || (lane == 0));
            bool speedup = true;

            //cout<<"Number of other cars tracked: " << sensor_fusion.size() << endl;


            //***********************************Tracking Other Vehicles***********************************//

            for(int i=0; i< sensor_fusion.size(); i++)  //loop through sensor_fusion list of all detected obstacles
              {
              
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double front_car_speed = sqrt (vx*vx + vy*vy);
                double front_car_s = sensor_fusion[i][5];

                front_car_s += ((double)prev_size*0.02*front_car_speed);    //future s location of detected car
                float d = sensor_fusion[i][6];
                int front_car_lane = lanecheck(d);

                if (front_car_lane == lane)    //if lane of detected vehicle is same as of ego car
                  {

                    min_distance = 1.2 * car_speed * 1610/3600; 
                    if ((front_car_s > car_s) && ((front_car_s - car_s) < min_distance)) //& if front car is <min_distance
                       {
                         too_close = true; //also raise flag to change lane
                         //cout<< "Car detected ahead in same lane: "<<front_car_lane <<" & speed "<< front_car_speed<<endl;
                         if ((front_car_speed < car_speed) && (ref_vel >= 1))
                             ref_vel -= 0.3;
                        }
                  }      

                else if (lanestate == Prepfor_CHANGE_LANE)
                    {
                      if((front_car_lane == target_lane) && !clear_to_pass(car_s, car_speed, front_car_s, front_car_speed))
                         OKto_pass = false;
                              
                      else if ((  ((lane == 1) && (front_car_lane == 2)) || ((lane==0) && (front_car_lane ==1))  ) && 
                                            !clear_to_pass(car_s, car_speed, front_car_s, front_car_speed))
                        pass_right = false;
                      
                    }

              } 
              

            /*if(too_close && (ref_vel >= 1.0))

              ref_vel -= 0.3; //slow down until not too close*/                                
             
            if ((ref_vel <= 49) && !(too_close))
            {
              speedup = true;
              ref_vel += 0.3;
            }

               

              //***********************************Behavior Planning**************************//

            
              switch (lanestate)
              {
                case Prepfor_CHANGE_LANE:
                    if (OKto_pass)
                    {
                      lanestate = CHANGE_LANE;
                      speedup = true;
                    }
                    else if (pass_right)
                    {
                      target_lane = lanecheck(car_d) +1;
                      lanestate = CHANGE_LANE;
                      speedup = true;
                    }
                    else speedup = !too_close;

                break;

                case CHANGE_LANE:
                    lane = target_lane;
                    lanestate = KEEP_LANE;
                    speedup = true;
                break;

                case KEEP_LANE:
                default:
                    if (too_close)
                    {                     
                      if (lane == 0) // || ((lane ==1) && (pass_right))) 
                        target_lane = 1;
                      else target_lane = lane - 1; 
                      lanestate = Prepfor_CHANGE_LANE;
                    }
                    speedup = !too_close;
                break;

              }
              
            //****************************Generate   Trajectory***********************************//

            vector<double> ptsx;
            vector<double> ptsy;
              
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            if(prev_size <2)                //When starting off, and no waypoints have been sent previously
              {
                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw); 

                ptsx.push_back(prev_car_x);
                ptsx.push_back(car_x);
                ptsy.push_back(prev_car_y);
                ptsy.push_back(car_y);
              
              }
            else                            //Otherwise use the last 2 points from previous waypoints sent to car
              {
                ref_x = previous_path_x[prev_size - 1];
                ref_y = previous_path_y[prev_size - 1];
              
                double ref_x_prev = previous_path_x[prev_size - 2];
                double ref_y_prev = previous_path_y[prev_size - 2];
                ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
                
                ptsx.push_back(ref_x_prev);
                ptsx.push_back(ref_x);
                ptsy.push_back(ref_y_prev);
                ptsy.push_back(ref_y);    
              }

            int d_value = 2+(4*lane);
            vector<double> next_wp0 = getXY(car_s + 30,d_value, map_waypoints_s, map_waypoints_x,map_waypoints_y); 
            vector<double> next_wp1 = getXY(car_s + 60,d_value, map_waypoints_s, map_waypoints_x,map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s + 100,d_value, map_waypoints_s, map_waypoints_x,map_waypoints_y);

             ptsx.push_back(next_wp0[0]);
             ptsx.push_back(next_wp1[0]);
             ptsx.push_back(next_wp2[0]);
             ptsy.push_back(next_wp0[1]);
             ptsy.push_back(next_wp1[1]);
             ptsy.push_back(next_wp2[1]);

            //Transform coordinates into perspective of ego car:

            for(int i=0; i<ptsx.size(); i++)
              {
                  double shift_x =ptsx[i]-ref_x;
                  double shift_y =ptsy[i]-ref_y;

                  ptsx[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
                  ptsy[i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));  
              }

            //Fit Spline to the 5 point vectors above: ptsx and ptsy
            tk::spline spline;
            spline.set_points(ptsx, ptsy);

          	vector<double> next_x_vals;                    //Declare vector that will be the waypoints sent to car
          	vector<double> next_y_vals;

            for (int i=0; i< previous_path_x.size(); i++)  //put back all unused previous waypoints into new set
              {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);                  
              }
                         
            double target_x = 30.0;
            double target_y = spline(target_x);
            double target_dist = distance(target_x, target_y, 0, 0);
              
            double x_add_on = 0.0;
            double N = (target_dist/(0.02*ref_vel/2.24));  //convert mph into meters traveled per 0.02s & to # of points
            double step  = target_x/N;

            for (int i=0; i< 75-previous_path_x.size(); i++) //Calculate additional waypoints to total 75
              {             
                double x_point =x_add_on + step;
                double y_point = spline(x_point);
                x_add_on  = x_point;
                double  x_ref = x_point;
                double  y_ref = y_point;

                //Convert back from ego car's reference to map coordinates:
                x_point = ref_x + x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
                y_point = ref_y + x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);

              }

            


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	
            /*double dist_inc = 0.4;
            for(int i = 0; i < 50; i++)
              {
               double next_s = car_s + (i+1)*dist_inc;//(1 + exp(-(i)));
               double next_d = 4;

               vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
               next_x_vals.push_back(xy[0]);
               next_y_vals.push_back(xy[1]);

               next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
               next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
              }*/

            //END

            json msgJson;
            msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
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
