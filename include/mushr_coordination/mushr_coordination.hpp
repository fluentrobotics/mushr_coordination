#ifndef MUSHR_COORDINATION_H
#define MUSHR_COORDINATION_H

#include <iostream>
#include <algorithm>
#include <string>
#include <utility>
#include <yaml-cpp/yaml.h>
#include <math.h>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

//#include "libMultiRobotPlanning/ecbs_ta.hpp"
#include "libMultiRobotPlanning/next_best_assignment.hpp"
#include "mushr_coordination/GoalPoseArray.h"
#include "mushr_coordination/ecbs_mushr.hpp"


#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/Marker.h"
#include "clcbs_ros/GoalPoseArray.h"


// TODO: need namespace for the header
using libMultiRobotPlanning::ECBSTA;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;
using libMultiRobotPlanning::NextBestAssignment;

// util
bool goal_compare(Waypoints & obj, int x, int y) {
  return obj.points.back().x == x && obj.points.back().y == y;
}

namespace node_mushr_coor{
class MushrCoordination {
  public:
    MushrCoordination(ros::NodeHandle &nh)
      : m_maxTaskAssignments(1e9),
        m_planning(false),
        m_ini_obs(false),
        m_ini_goal(false),
        m_sim(true) {
      nh.getParam("/mushr_coordination/w", m_w);
      nh.getParam("/mushr_coordination/num_waypoint", m_num_waypoint);
      nh.getParam("/mushr_coordination/num_agent", m_num_agent);
      m_car_pose = std::vector<geometry_msgs::Pose>(m_num_agent);
      for (size_t i = 0; i < m_num_agent; ++i) {
        std::string name, color;
        nh.getParam("/mushr_coordination/car" + std::to_string(i + 1) + "/name", name);
        nh.getParam("/mushr_coordination/car" + std::to_string(i + 1) + "/color", color);
        std::cout << name << " " << color << std::endl;
        m_car_name.push_back(name);
        m_car_color.push_back(color);
        m_sub_car_pose.push_back(nh.subscribe<geometry_msgs::PoseStamped>(
              name + (m_sim ? "/init_pose" : "/mocap_pose"),
              10,
              boost::bind(&MushrCoordination::CarPoseCallback, this, _1, i)
            ));
      }
      m_pub_goals = nh.advertise<clcbs_ros::GoalPoseArray>(
          "/clcbs_ros/goals", 
          10
        );
      m_pub_border = nh.advertise<visualization_msgs::Marker>(
          "/mushr_coordination/border",
          10
        );
      m_sub_obs_pose = nh.subscribe("/mushr_coordination/obstacles", 
              10, 
              &MushrCoordination::ObsPoseCallback, 
              this);
      m_sub_goal = nh.subscribe("/mushr_coordination/goals", 
              10, 
              &MushrCoordination::GoalCallback, 
              this);
      ros::Duration(1).sleep();
    }

    void CarPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, size_t i) {
      std::cout << "get car " << i << std::endl;
      m_assigned.insert(i);
      m_car_pose[i] = msg->pose;
      if (isReady()) {
          solve();
      }
    }

    void ObsPoseCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
      std::cout << "get obs " << std::endl;
      m_ini_obs = true;  
      for (auto &pose : msg->poses) {
        m_obs_pose.emplace_back(pose.position.x, pose.position.y);
      }
      if (isReady()) {
        solve();
      }
    }

    void GoalCallback(const mushr_coordination::GoalPoseArray::ConstPtr& msg) {
      std::cout << "get goal" << std::endl;
      m_ini_goal = true;
      m_scale = msg->scale;
      m_minx = msg->minx;
      m_miny = msg->miny;
      m_maxx = msg->maxx;
      m_maxy = msg->maxy;
      for (auto &goal : msg->goals) {
        m_goal_pose.emplace_back();
        for (auto &pose : goal.poses) {
          m_goal_pose.back().emplace_back(pose);
        }
      }
      if (isReady()) {
          solve();
      }
    }

  private:
    void solve() {
      m_planning = true;
      std::cout << "start planning" << std::endl;
      std::unordered_set<Location> obstacles;
      std::vector<State> startStates;
      std::vector<State> oldStartStates;
      std::vector<Waypoints> goals;

      int extra = 0;
      // init goals
      for (auto& goal: m_goal_pose) {
        std::vector<Location> ls;
        for(auto& waypoint: goal) {
          ls.emplace_back(scalex(waypoint.position.x), scaley(waypoint.position.y));
        }
        goals.emplace_back(ls);
      }
      for (auto & g: goals) {
        std::cout << g << std::endl;
      }
      // init obstacles
      for(auto& pos: m_obs_pose) {
        obstacles.insert(Location(scalex(pos.first), scaley(pos.second)));
      }
      // init start states
      for(auto& pos: m_car_pose) {
        startStates.emplace_back(0, scalex(pos.position.x), scaley(pos.position.y), 0); 
      }

      int mkid = 0; //visualize

      int dimx = scalex(m_maxx) + 1;
      int dimy = scaley(m_maxy) + 1;
      bool success = true;
      std::vector<int> startTime;
      std::vector<PlanResult<State, Action, int> > solution;
      std::cout << "x: " << dimx << ", y: " << dimy << ", agent: " << m_num_agent << ", task: "<< m_goal_pose.size() << ", waypoint: " << m_num_waypoint << std::endl;

      startTime.push_back(0);
      while (goals.size() > 0 && success) {
        extra = m_num_agent - goals.size() > 0 ? m_num_agent - goals.size() : 0;
        for (int i = 0; i < extra; i++) {
          std::vector<Location> ls;
          ls.emplace_back(-i - 1, -i - 1);
          goals.emplace_back(ls);
        }
        Environment mapf(dimx, dimy, m_num_waypoint, obstacles, startStates, goals,
                      m_maxTaskAssignments);
        ECBSTA<State, Action, int, Conflict, Constraints, Waypoints,
              Environment>
            cbs(mapf, m_w);
        
        std::vector<PlanResult<State, Action, int> > sub_solution;
        success &= cbs.search(startStates, sub_solution);
        oldStartStates = startStates;
        startStates.clear();
        std::vector<Waypoints>::iterator it;
        int ct = 0;
        int sub_makespan = 0;
        for (const auto& s : sub_solution) {
          State last = s.states.back().first;
          it = std::find_if(goals.begin(), goals.end(), 
            std::bind(goal_compare, std::placeholders::_1, last.x, last.y));
          if (it != goals.end()) {
            ct++;
            goals.erase(it);
          }
          startStates.emplace_back(State(0, last.x, last.y, 0));
          sub_makespan = std::max<int64_t>(sub_makespan, s.cost);
        }
        startTime.push_back(sub_makespan);
        solution.insert(solution.end(), sub_solution.begin(), sub_solution.end());
        if (success && goals.size() - extra > 0) {
          std::cout << "finish " << ct + extra << " tasks, " << goals.size() - extra << " task remaining" << std::endl;
        } else if (success) {
          std::cout << "planner success" << std::endl;
          break;
        } else {
          std::cout << "planner failed" << std::endl;
          break;
        }
        std::cout << "---------------------" << std::endl;
      }
      create_border(1, 1, 1, 0.1);

      if (success) {
        clcbs_ros::GoalPoseArray goalmsg;
        goalmsg.header.frame_id = "/map";
        goalmsg.header.stamp = ros::Time::now();
        goalmsg.num_agent = m_num_agent;
        goalmsg.num_waypoint = m_num_waypoint * (startTime.size() - 1);
        goalmsg.scale = m_scale;
        goalmsg.minx = m_minx;
        goalmsg.miny = m_miny;
        goalmsg.maxx = m_maxx;
        goalmsg.maxy = m_maxy;
        // goalmsg.align_dist = 0.5;
        std::vector<bool> used(m_goal_pose.size());
        for (size_t a = 0; a < m_num_agent; ++a) {
          goalmsg.goals.emplace_back();
          for(int t = 0; t < startTime.size() - 1; t++) {
            int j = t * m_num_agent + a;
            bool found = false;
            for (size_t i = 0; i < m_goal_pose.size() && !found; i++) {
              if (!used[i] && fabs(scalex(m_goal_pose[i][1].position.x) - solution[j].states.back().first.x) < 0.0001 &&
                  fabs(scaley(m_goal_pose[i][1].position.y) - solution[j].states.back().first.y) < 0.0001) {
                goalmsg.goals[a].poses.emplace_back(m_goal_pose[i][0]);
                goalmsg.goals[a].poses.emplace_back(m_goal_pose[i][1]);
                found = true;
                used[i] = true;
              }
            } 
            if (!found) {
              if (goalmsg.goals[a].poses.empty()) {
                goalmsg.goals[a].poses.emplace_back(m_car_pose[a]);
                goalmsg.goals[a].poses.emplace_back(m_car_pose[a]);
              } else {                
                goalmsg.goals[a].poses.emplace_back(goalmsg.goals[a].poses.back());
                goalmsg.goals[a].poses.emplace_back(goalmsg.goals[a].poses.back());
              }
            }
          }
        }
        m_pub_goals.publish(goalmsg);
        std::cout << "publish goals to clcbs" << std::endl;
      }
      
      m_goal_pose.clear();
      m_obs_pose.clear();
      m_car_pose = std::vector<geometry_msgs::Pose>(m_num_agent);
      m_ini_obs = false;
      m_ini_goal = false;
      m_assigned.clear();
      m_planning = false;
    }

    void create_border(double r, double b, double g, double thickness) {
      visualization_msgs::Marker marker;

      geometry_msgs::Point p;
      p.x = m_minx; p.y = m_miny; p.z = 0.0;
      marker.points.push_back(p);
      p.x = m_minx; p.y = m_maxy; p.z = 0.0;
      marker.points.push_back(p);
      p.x = m_maxx; p.y = m_maxy; p.z = 0.0;
      marker.points.push_back(p);
      p.x = m_maxx; p.y = m_miny; p.z = 0.0;
      marker.points.push_back(p);
      p.x = m_minx; p.y = m_miny; p.z = 0.0;
      marker.points.push_back(p);
      p.x = m_minx; p.y = m_maxy; p.z = 0.0;
      marker.points.push_back(p);
      
      marker.color.r = r;
      marker.color.g = b;
      marker.color.b = g;
      marker.color.a = 1.0;

      marker.scale.x = thickness;
      marker.scale.y = thickness;
      marker.scale.z = thickness;

      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time();
      marker.id = 0;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.action = visualization_msgs::Marker::ADD;

      m_pub_border.publish(marker);
    }

    void create_marker(visualization_msgs::Marker* marker, int* mkid, double x, double y, double r, double g, double b, double size, int last) {
      marker->pose.position.x = x;
      marker->pose.position.y = y;
      marker->pose.position.z = 0;

      marker->pose.orientation.x = 0;
      marker->pose.orientation.y = 0;
      marker->pose.orientation.z = 0;
      marker->pose.orientation.w = 1;

      marker->color.r = r;
      marker->color.g = g;
      marker->color.b = b;
      marker->color.a = 1.0;

      marker->scale.x = size;
      marker->scale.y = size;
      marker->scale.z = size;

      marker->header.frame_id = "map";
      marker->header.stamp = ros::Time();
      marker->id = (*mkid)++;

      marker->type = last ? visualization_msgs::Marker::SPHERE : visualization_msgs::Marker::CUBE;
      marker->action = visualization_msgs::Marker::ADD;
    }

    double r_color(std::string hex) {
      int hexValue = std::stoi(hex, 0, 16);
      return ((hexValue >> 16) & 0xFF) / 255.0;
    }

    double g_color(std::string hex) {
      int hexValue = std::stoi(hex, 0, 16);
      return  ((hexValue >> 8) & 0xFF) / 255.0;
    }

    double b_color(std::string hex) {
      int hexValue = std::stoi(hex, 0, 16);
      return ((hexValue) & 0xFF) / 255.0; 
    }

    bool isReady() {
      return m_assigned.size() == m_num_agent && m_ini_obs && m_ini_goal && !m_planning;
    }

    int scalex(double x) {
      return floor((x - m_minx) * m_scale);
    }

    int scaley(double y) {
      return floor((y - m_miny) * m_scale);
    }

    double r_scalex(int x) {
      return x / m_scale + m_minx;
    }

    double r_scaley(int y) {
      return y / m_scale + m_miny;
    }

    //bool goal_compare(Waypoints & obj, int x, int y) {
    //  return obj.points.back().x == x && obj.points.back().y == y;
    //}

    std::vector<ros::Subscriber> m_sub_car_pose;
    ros::Subscriber m_sub_obs_pose;
    ros::Subscriber m_sub_goal;
    ros::Publisher m_pub_border;
    ros::Publisher m_pub_goals;
    std::vector<std::string> m_car_name;
    std::vector<std::string> m_car_color;
    std::vector<geometry_msgs::Pose> m_car_pose;
    std::vector<std::pair<double, double>> m_obs_pose;
    std::vector<std::vector<geometry_msgs::Pose>> m_goal_pose;
    std::set<size_t> m_assigned;
    bool m_planning;
    bool m_ini_obs;
    bool m_ini_goal;
    int m_maxTaskAssignments;
    int m_num_agent;
    int m_num_waypoint;
    bool m_sim;
    double m_w;
    double m_scale;
    double m_minx, m_miny, m_maxx, m_maxy;
};
}

#endif // MUSHR_COORDINATION_H
