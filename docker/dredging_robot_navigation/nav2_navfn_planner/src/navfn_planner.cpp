// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2018 Simbe Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Navigation Strategy based on:
// Brock, O. and Oussama K. (1999). High-Speed Navigation Using
// the Global Dynamic Window Approach. IEEE.
// https://cs.stanford.edu/group/manips/publications/pdfs/Brock_1999_ICRA.pdf

#include "nav2_navfn_planner/navfn_planner.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include "builtin_interfaces/msg/duration.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "nav2_navfn_planner/navfn.hpp"
#include "nav2_util/costmap.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <chrono>
#include <thread>

using namespace std::chrono_literals;

namespace nav2_navfn_planner
{

NavfnPlanner::NavfnPlanner()
    : nav2_util::LifecycleNode("navfn_planner", "", true)
{
  RCLCPP_INFO(get_logger(), "Creating");
  RCLCPP_INFO(get_logger(), "..............Hello, this is new code........");

  // Declare this node's parameters
  declare_parameter("tolerance", rclcpp::ParameterValue(0.0));
  declare_parameter("use_astar", rclcpp::ParameterValue(false));

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
}

NavfnPlanner::~NavfnPlanner()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

nav2_util::CallbackReturn
NavfnPlanner::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // Initialize parameters
  get_parameter("tolerance", tolerance_);
  get_parameter("use_astar", use_astar_);

  getCostmap(costmap_);
  RCLCPP_DEBUG(get_logger(), "Costmap size: %d,%d",
               costmap_.metadata.size_x, costmap_.metadata.size_y);

  // Create a planner based on the new costmap size
  if (isPlannerOutOfDate())
  {
    current_costmap_size_[0] = costmap_.metadata.size_x;
    current_costmap_size_[1] = costmap_.metadata.size_y;
    planner_ = std::make_unique<NavFn>(costmap_.metadata.size_x, costmap_.metadata.size_y);
  }

  // Initialize pubs & subs
  plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", 1);

  auto node = shared_from_this();

  // Create the action server that we implement with our navigateToPose method
  action_server_ = std::make_unique<ActionServer>(rclcpp_node_, "ComputePathToPose",
                                                  std::bind(&NavfnPlanner::computePathToPose, this));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
NavfnPlanner::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  plan_publisher_->on_activate();
  action_server_->activate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
NavfnPlanner::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_->deactivate();
  plan_publisher_->on_deactivate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
NavfnPlanner::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  action_server_.reset();
  plan_publisher_.reset();
  planner_.reset();
  tf_listener_.reset();
  tf_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
NavfnPlanner::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_FATAL(get_logger(), "Lifecycle node entered error state");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
NavfnPlanner::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

/*
void filler(nav2_msgs::msg::Path & initial, nav2_msgs::msg::Path & plan)
{
  plan.poses.clear();

  plan.header = initial.header;
  plan.poses.resize(initial.poses.size());
  for (unsigned int i = 0; i < initial.poses.size(); i++) 
  {

  }
}
*/

double kCalc(const geometry_msgs::msg::Pose &start, const geometry_msgs::msg::Pose &goal)
{
  return ((goal.position.y - start.position.y) / (goal.position.x - start.position.x));
}

double yCalc(const geometry_msgs::msg::Pose &start, const geometry_msgs::msg::Pose &goal)
{
  return (goal.position.y - start.position.y);
}

double xCalc(const geometry_msgs::msg::Pose &start, const geometry_msgs::msg::Pose &goal)
{
  return (goal.position.x - start.position.x);
}

/*******************************************************/
/*******************************************************/
/**************Coverage planning algorithm**************/
/*******************Part 1, Classes*********************/
/*******************************************************/
/*******************************************************/

struct GridMap
{
    int width, height;
    double resolution, center_x, center_y, left_lower_x, left_lower_y;
    std::vector<double> data;
    GridMap(int w, int h, double r, double c_x, double c_y) : width{w}, height{h}, resolution{r}, center_x{c_x}, center_y{c_y}
    {
        left_lower_x = center_x - (width / 2.0) * resolution;
        left_lower_y = center_y - (height / 2.0) * resolution;
        for (int i = 0; i < (width * height); i++)
        {
            data.push_back(0.0);
        }
    }
    ~GridMap() = default;
    void set_value_from_xy_index(int index_x, int index_y, double val)
    {
        int grid_index{index_y * width + index_x};
        if (0 <= grid_index && grid_index < (width * height))
        {
            data.at(grid_index) = val;
        }
    }
    void set_value_from_polygon(std::vector<double> vx, std::vector<double> vy, double val, bool inside = true)
    {
        for (int index_x = 0; index_x < width; index_x++)
        {
            for (int index_y = 0; index_y < height; index_y++)
            {
                double x_pos = left_lower_x + index_x * resolution + resolution / 2.0;
                double y_pos = left_lower_y + index_y * resolution + resolution / 2.0;
                bool flag = check_inside_polygon(x_pos, y_pos, vx, vy);
                if (flag == inside)
                {
                    set_value_from_xy_index(index_x, index_y, val);
                }
            }
        }
    }
    bool check_occupied_from_xy_index(int xind, int yind, double occupied_val = 1.0)
    {
        double val{};
        int grid_index{yind * width + xind};
        if (0 <= grid_index && grid_index <= (height * width))
        {
            val = data.at(grid_index);
        }
        else
        {
            std::cerr << "grid_index out of bound!" << std::endl;
        }
        if (val >= occupied_val)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    void expand_grid()
    {
        std::vector<std::pair<int, int>> v;
        for (int index_x = 0; index_x < width; index_x++)
        {
            for (int index_y = 0; index_y < height; index_y++)
            {
                if (check_occupied_from_xy_index(index_x, index_y))
                {
                    v.push_back(std::pair<int, int>(index_x, index_y));
                }
            }
        }
        for (auto &&i : v)
        {
            set_value_from_xy_index(i.first + 1, i.second, 1.0);
            set_value_from_xy_index(i.first, i.second + 1, 1.0);
            set_value_from_xy_index(i.first + 1, i.second + 1, 1.0);
            set_value_from_xy_index(i.first - 1, i.second, 1.0);
            set_value_from_xy_index(i.first, i.second - 1, 1.0);
            set_value_from_xy_index(i.first - 1, i.second - 1, 1.0);
        }
    }
    bool check_inside_polygon(double x_pos, double y_pos, std::vector<double> vx, std::vector<double> vy)
    {
        size_t npoint{vx.size() - 1};
        bool inside = false;
        int i2;
        double min_x{}, max_x{};
        for (size_t i1 = 0; i1 < npoint; i1++)
        {
            i2 = (i1 + 1) % (npoint + 1);
            if (vx.at(i1) >= vx.at(i2))
            {
                min_x = vx.at(i2);
                max_x = vx.at(i1);
            }
            else
            {
                min_x = vx.at(i1);
                max_x = vx.at(i2);
            }
            if (!(min_x < x_pos && x_pos < max_x))
            {
                continue;
            }
            if ((vy.at(i1) + (vy.at(i2) - vy.at(i1)) / (vx.at(i2) - vx.at(i1)) * (x_pos - vx.at(i1)) - y_pos) > 0.0)
            {
                inside = !inside;
            }
        }
        return inside;
    }
};
struct SweepSearcher
{
    int moving_direction;
    int sweeping_direction;
    std::vector<int> x_indices_goal_y;
    int goal_y;
    std::vector<std::pair<int, int>> turning_window;
    SweepSearcher(int mdir, int sdir, std::vector<int> v, int i) : moving_direction{mdir}, sweeping_direction{sdir}, x_indices_goal_y{v}, goal_y{i}
    {
        turning_window.push_back(std::pair<int, int>(0, 0));
        turning_window.push_back(std::pair<int, int>(0, 0));
        turning_window.push_back(std::pair<int, int>(0, 0));
        turning_window.push_back(std::pair<int, int>(0, 0));
        update_turning_window();
    }
    void update_turning_window()
    {
        turning_window.at(0) = std::pair<int, int>(moving_direction, 0);
        turning_window.at(1) = std::pair<int, int>(moving_direction, sweeping_direction);
        turning_window.at(2) = std::pair<int, int>(0, sweeping_direction);
        turning_window.at(3) = std::pair<int, int>(-moving_direction, sweeping_direction);
    }
};

/*******************************************************/
/*******************************************************/
/**************Coverage planning algorithm**************/
/******************Part 1, Algorithm********************/
/*******************************************************/
/*******************************************************/

std::vector<geometry_msgs::msg::Pose> generate_plan(geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2, geometry_msgs::msg::Pose p3, geometry_msgs::msg::Pose p4, double resolution = 0.5)
{
    std::vector<double> initial_x;
    std::vector<double> initial_y;
    initial_x.push_back(p1.position.x);
    initial_y.push_back(p1.position.y);
    initial_x.push_back(p2.position.x);
    initial_y.push_back(p2.position.y);
    initial_x.push_back(p3.position.x);
    initial_y.push_back(p3.position.y);
    initial_x.push_back(p4.position.x);
    initial_y.push_back(p4.position.y);
    initial_x.push_back(p1.position.x);
    initial_y.push_back(p1.position.y);

    // find_sweep_direction_and_start_posi:
    double longest{}, dist_to_next_x{}, dist_to_next_y{}, start_pos_x{}, start_pos_y{};
    for (size_t i = 0; i < initial_x.size() - 1; i++)
    {
        double delta_x = initial_x.at(i + 1) - initial_x.at(i);
        double delta_y = initial_y.at(i + 1) - initial_y.at(i);
        double line = std::hypot(delta_x, delta_y);
        if (line > longest)
        {
            longest = line;
            dist_to_next_x = delta_x;
            dist_to_next_y = delta_y;
            start_pos_x = initial_x.at(i);
            start_pos_y = initial_y.at(i);
        }
    }

    // convert_grid_coordinate:
    double thetha{std::atan2(dist_to_next_y, dist_to_next_x)};
    double cos{std::cos(-thetha)};
    double sin{std::sin(-thetha)};
    std::vector<double> rx;
    std::vector<double> ry;
    for (size_t i = 0; i < initial_x.size(); i++)
    {
        double dist_to_all_from_x = initial_x.at(i) - start_pos_x;
        double dist_to_all_from_y = initial_y.at(i) - start_pos_y;
        rx.push_back(dist_to_all_from_x * cos - dist_to_all_from_y * sin);
        ry.push_back(dist_to_all_from_x * sin + dist_to_all_from_y * cos);
    }

    // setup_grid_map:
    int width{static_cast<int>(std::ceil((*(std::max_element(rx.begin(), rx.end())) - *(std::min_element(rx.begin(), rx.end()))) / resolution) + 10)};
    int height{static_cast<int>(std::ceil((*(std::max_element(ry.begin(), ry.end())) - *(std::min_element(ry.begin(), ry.end()))) / resolution) + 10)};
    double center_x{}, center_y{};
    for (size_t i = 0; i < rx.size(); i++)
    {
        center_x += rx.at(i);
        center_y += ry.at(i);
    }
    center_x /= rx.size();
    center_y /= rx.size();
    GridMap grid_map(width, height, resolution, center_x, center_y);
    grid_map.set_value_from_polygon(rx, ry, 1.0, false);
    grid_map.expand_grid();
    // SweepDirection.DOWN (-1)
    std::vector<int> x_indices_goal_y{};
    int goal_y{};
    for (int index_y = 0; index_y < height; index_y++)
    {
        for (int index_x = 0; index_x < width; index_x++)
        {
            if (!(grid_map.check_occupied_from_xy_index(index_x, index_y)))
            {
                goal_y = index_y;
                x_indices_goal_y.push_back(index_x);
            }
        }
        if (goal_y)
        {
            break;
        }
    }

    // create a SweepSearcher class
    SweepSearcher sweep_searcher(-1, -1, x_indices_goal_y, goal_y);

    // sweep_path_search:
    // the algorithm
    // search_start_grid:
    x_indices_goal_y.clear();
    goal_y = 0;
    for (int index_y = height - 1; index_y >= 0; index_y--)
    {
        for (int index_x = width - 1; index_x >= 0; index_x--)
        {
            if (!(grid_map.check_occupied_from_xy_index(index_x, index_y)))
            {
                goal_y = index_y;
                x_indices_goal_y.push_back(index_x);
            }
        }
        if (goal_y)
        {
            break;
        }
    }
    // MovingDirection.LEFT (-1)
    int cxind{*std::max_element(x_indices_goal_y.begin(), x_indices_goal_y.end())};
    grid_map.set_value_from_xy_index(cxind, goal_y, 0.5);
    double x{grid_map.left_lower_x + cxind * resolution + resolution / 2.0};
    double y{grid_map.left_lower_y + goal_y * resolution + resolution / 2.0};
    std::vector<double> px;
    std::vector<double> py;
    px.push_back(x);
    py.push_back(y);
    while (true)
    {
        // move_target_grid:
        int nxind = sweep_searcher.moving_direction + cxind;
        int nyind = goal_y;
        bool move_target_grid_return = false;
        if (!(grid_map.check_occupied_from_xy_index(nxind, nyind, 0.5)))
        {
            cxind = nxind;
            goal_y = nyind;
        }
        else
        {
            bool find_safe_turning_grid_return = false;
            int ncxind, ncyind;
            for (auto &&i : sweep_searcher.turning_window)
            {
                ncxind = i.first + cxind;
                ncyind = i.second + goal_y;
                if (!(grid_map.check_occupied_from_xy_index(ncxind, ncyind, 0.5)))
                {
                    find_safe_turning_grid_return = true;
                    break;
                }
            }
            if (!find_safe_turning_grid_return)
            {
                ncxind = -sweep_searcher.moving_direction + cxind;
                ncyind = goal_y;
                if (grid_map.check_occupied_from_xy_index(ncxind, ncyind))
                {
                    move_target_grid_return = true;
                }
            }
            else
            {
                while (!(grid_map.check_occupied_from_xy_index(ncxind + sweep_searcher.moving_direction, ncyind, 0.5)))
                {
                    ncxind += sweep_searcher.moving_direction;
                }
                sweep_searcher.moving_direction *= -1;
                sweep_searcher.update_turning_window();
            }
            if (!move_target_grid_return)
            {
                cxind = ncxind;
                goal_y = ncyind;
            }
        }
        bool is_search_done = true;
        for (auto &&i : sweep_searcher.x_indices_goal_y)
        {
            if (!(grid_map.check_occupied_from_xy_index(i, sweep_searcher.goal_y, 0.5)))
            {
                is_search_done = false;
                break;
            }
        }
        if (is_search_done || move_target_grid_return)
        {
            break;
        }
        x = grid_map.left_lower_x + cxind * resolution + resolution / 2.0;
        y = grid_map.left_lower_y + goal_y * resolution + resolution / 2.0;
        if (px.size() > 1)
        {
            if ((*(py.end() - 2) - *(py.end() - 1)) * (*(px.end() - 2) - x) == (*(py.end() - 2) - y) * (*(px.end() - 2) - *(px.end() - 1)))
            {
                px.pop_back();
                py.pop_back();
            }
        }
        px.push_back(x);
        py.push_back(y);
        grid_map.set_value_from_xy_index(cxind, goal_y, 0.5);
    }

    // convert_global_coordinate:
    thetha = std::atan2(dist_to_next_y, dist_to_next_x);
    cos = std::cos(thetha);
    sin = std::sin(thetha);
    rx.clear();
    ry.clear();
    for (size_t i = 0; i < px.size(); i++)
    {
        rx.push_back((px.at(i) * cos - py.at(i) * sin) + start_pos_x);
        ry.push_back((px.at(i) * sin + py.at(i) * cos) + start_pos_y);
    }
    geometry_msgs::msg::Pose tmp;
    std::vector<geometry_msgs::msg::Pose> res;
    for (size_t i = 0; i < rx.size(); i++)
    {
        tmp.position.x = rx.at(i);
        tmp.position.y = ry.at(i);
        res.push_back(tmp);
    }
    return res;
}

int xs = 1;
void NavfnPlanner::computePathToPose()
{
  // Initialize the ComputePathToPose goal and result
  auto goal = action_server_->get_current_goal();
  auto result = std::make_shared<nav2_msgs::action::ComputePathToPose::Result>();

  try
  {
    if (action_server_ == nullptr)
    {
      RCLCPP_DEBUG(get_logger(), "Action server unavailable. Stopping.");
      return;
    }

    if (!action_server_->is_server_active())
    {
      RCLCPP_DEBUG(get_logger(), "Action server is inactive. Stopping.");
      return;
    }

    if (action_server_->is_cancel_requested())
    {
      RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling planning action.");
      action_server_->terminate_goals();
      return;
    }

    // Get the current costmap
    getCostmap(costmap_);
    RCLCPP_DEBUG(get_logger(), "Costmap size: %d,%d",
                 costmap_.metadata.size_x, costmap_.metadata.size_y);

    geometry_msgs::msg::PoseStamped start;
    if (!nav2_util::getCurrentPose(start, *tf_))
    {
      return;
    }

    /*******************************************************/
    /*******************************************************/
    /**************Coverage planning algorithm**************/
    /**************Part 3, Calling the function*************/
    /*******************************************************/
    /*******************************************************/

    geometry_msgs::msg::Pose startingPose;
    startingPose.position.x = -5.0;
    startingPose.position.y = -5.0;
    startingPose = start.pose;
    geometry_msgs::msg::Pose testGoal1;
    testGoal1.position.x = -9.6;
    testGoal1.position.y = -9.6;
    geometry_msgs::msg::Pose testGoal2;
    testGoal2.position.x = -9.6;
    testGoal2.position.y = 9.6;
    geometry_msgs::msg::Pose testGoal3;
    testGoal3.position.x = 9.6;
    testGoal3.position.y = 9.6;
    geometry_msgs::msg::Pose testGoal4;
    testGoal4.position.x = 9.6;
    testGoal4.position.y = -9.6;
    std::vector<geometry_msgs::msg::Pose> testGoals = generate_plan(testGoal1, testGoal2, testGoal3, testGoal4, 0.5 );

    int goalValue = 0;

    // Update planner based on the new costmap size
    if (isPlannerOutOfDate())
    {
      current_costmap_size_[0] = costmap_.metadata.size_x;
      current_costmap_size_[1] = costmap_.metadata.size_y;
      planner_->setNavArr(costmap_.metadata.size_x, costmap_.metadata.size_y);
    }

    if (action_server_->is_preempt_requested())
    {
      RCLCPP_INFO(get_logger(), "Preempting the goal pose.");
      goal = action_server_->accept_pending_goal();
    }

    RCLCPP_DEBUG(get_logger(), "Attempting to a find path from (%.2f, %.2f) to "
                               "(%.2f, %.2f).",
                 start.pose.position.x, start.pose.position.y,
                 goal->pose.pose.position.x, goal->pose.pose.position.y);

    // Make the plan for the provided goal pose
    bool foundPath;
    double tolerance = 0.25;
    bool allPointsChecked = false;
    geometry_msgs::msg::Pose checkingPose;
    checkingPose.position.x = startingPose.position.x;
    checkingPose.position.y = startingPose.position.y;
    
    for (size_t t = xs; t < testGoals.size(); t++)
    {
      while (!allPointsChecked)
      {
        double xRatio = 0.0;
        double yRatio = 0.0;
        double kRatio = 0.0;
        double degrees = 0.0;
        RCLCPP_INFO(get_logger(), "t: (%.i)", t);

        if (
            testGoals.at(t).position.x <= checkingPose.position.x + tolerance &&
            testGoals.at(t).position.x >= checkingPose.position.x - tolerance &&
            testGoals.at(t).position.y <= checkingPose.position.y + tolerance &&
            testGoals.at(t).position.y >= checkingPose.position.y - tolerance)
        {
          RCLCPP_INFO(get_logger(), "on goal");
          xs++;
          break;
        }
        else
        {
          RCLCPP_INFO(get_logger(), "startingPose: (%.2f, %.2f)", start.pose.position.x, start.pose.position.y);
          RCLCPP_INFO(get_logger(), "CheckingPose: (%.2f, %.2f)", checkingPose.position.x, checkingPose.position.y);
          RCLCPP_INFO(get_logger(), "testGoal: (%.2f, %.2f)", testGoals.at(t).position.x, testGoals.at(t).position.y);

          if (
              start.pose.position.x <= checkingPose.position.x + tolerance &&
              start.pose.position.x >= checkingPose.position.x - tolerance &&
              start.pose.position.y <= checkingPose.position.y + tolerance &&
              start.pose.position.y >= checkingPose.position.y - tolerance)
          {
            goalValue = t;
            RCLCPP_INFO(get_logger(), "in bound");
            RCLCPP_INFO(get_logger(), "goalValue: (%.i)", t);
            allPointsChecked = true;
          }
          else
          {
            RCLCPP_INFO(get_logger(), "not in bound");
          }

          //RCLCPP_INFO(get_logger(), "TestGoal: (%.2f, %.2f)", testGoals.at(t).position.x, testGoals.at(t).position.y);
          //RCLCPP_INFO(get_logger(), "CheckingPose: (%.2f, %.2f)", checkingPose.position.x, checkingPose.position.y);

          xRatio = xCalc(checkingPose, testGoals.at(t));
          RCLCPP_INFO(get_logger(), "x-ratio: (%.2f)", xRatio);

          yRatio = yCalc(checkingPose, testGoals.at(t));
          RCLCPP_INFO(get_logger(), "y-ratio: (%.2f)", yRatio);

          kRatio = kCalc(checkingPose, testGoals.at(t));
          RCLCPP_INFO(get_logger(), "k-ratio: (%.2f)", kRatio);

          degrees = atan(kRatio) * 180 / 3.14159265;
          RCLCPP_INFO(get_logger(), "degrees: (%.2f)", degrees);

          xRatio = cos(atan(kRatio));
          xRatio = abs(xRatio);
          RCLCPP_INFO(get_logger(), "x-ratio new: (%.2f)", xRatio);

          yRatio = sin(atan(kRatio));
          yRatio = abs(yRatio);
          RCLCPP_INFO(get_logger(), "y-ratio new: (%.2f)", yRatio);

          double testerX = xCalc(testGoals.at(t - 1), testGoals.at(t));
          double testerY = yCalc(testGoals.at(t - 1), testGoals.at(t));

          if (testerX < 0)
          {
            xRatio = xRatio * -1;
          }

          if (testerY < 0)
          {
            yRatio = yRatio * -1;
          }

          RCLCPP_INFO(get_logger(), "..............", yRatio);

          checkingPose.position.x = checkingPose.position.x + 0.5 * xRatio;
          checkingPose.position.y = checkingPose.position.y + 0.5 * yRatio;
        }
      }

      if (allPointsChecked)
      {
        break;
      }
    }
    

    foundPath = false;
    for (int i = goalValue; i < (int)testGoals.size(); i++)
    {
      if (i == goalValue)
      {
        foundPath = makePlan(start.pose, testGoals.at(i), tolerance_, result->path);
      }
      else
      {
        foundPath = makePlan(testGoals.at(i - 1), testGoals.at(i), tolerance_, result->path);
      }
    }

    //bool foundPath = makePlan(start.pose, goal->pose.pose, tolerance_, result->path);

    if (!foundPath)
    {
      RCLCPP_WARN(get_logger(), "Planning algorithm failed to generate a valid"
                                " path to (%.2f, %.2f)",
                  goal->pose.pose.position.x, goal->pose.pose.position.y);
      // TODO(orduno): define behavior if a preemption is available
      action_server_->terminate_goals();
      return;
    }

    RCLCPP_DEBUG(get_logger(), "Found valid path of size %u", result->path.poses.size());

    // Publish the plan for visualization purposes
    RCLCPP_DEBUG(get_logger(), "Publishing the valid path");
    publishPlan(result->path);

    // TODO(orduno): Enable potential visualization

    RCLCPP_DEBUG(get_logger(),
                 "Successfully computed a path to (%.2f, %.2f) with tolerance %.2f",
                 goal->pose.pose.position.x, goal->pose.pose.position.y, tolerance_);
    action_server_->succeeded_current(result);
    return;
  }
  catch (std::exception &ex)
  {
    RCLCPP_WARN(get_logger(), "Plan calculation to (%.2f, %.2f) failed: \"%s\"",
                goal->pose.pose.position.x, goal->pose.pose.position.y, ex.what());

    // TODO(orduno): provide information about fail error to parent task,
    //               for example: couldn't get costmap update
    action_server_->terminate_goals();
    return;
  }
  catch (...)
  {
    RCLCPP_WARN(get_logger(), "Plan calculation failed");

    // TODO(orduno): provide information about the failure to the parent task,
    //               for example: couldn't get costmap update
    action_server_->terminate_goals();
    return;
  }
}

bool NavfnPlanner::isPlannerOutOfDate()
{
  if (!planner_.get() || current_costmap_size_[0] != costmap_.metadata.size_x ||
      current_costmap_size_[1] != costmap_.metadata.size_y)
  {
    return true;
  }
  return false;
}

bool NavfnPlanner::makePlan(
    const geometry_msgs::msg::Pose &start,
    const geometry_msgs::msg::Pose &goal, double tolerance,
    nav2_msgs::msg::Path &plan)
{
  // clear the plan, just in case
  //plan.poses.clear();

  // TODO(orduno): add checks for start and goal reference frame -- should be in global frame

  double wx = start.position.x;
  double wy = start.position.y;

  RCLCPP_DEBUG(get_logger(), "Making plan from (%.2f,%.2f) to (%.2f,%.2f)",
               start.position.x, start.position.y, goal.position.x, goal.position.y);

  unsigned int mx, my;
  if (!worldToMap(wx, wy, mx, my))
  {
    RCLCPP_WARN(
        get_logger(),
        "Cannot create a plan: the robot's start position is off the global"
        " costmap. Planning will always fail, are you sure"
        " the robot has been properly localized?");
    return false;
  }

  // clear the starting cell within the costmap because we know it can't be an obstacle
  clearRobotCell(mx, my);

  // make sure to resize the underlying array that Navfn uses
  planner_->setNavArr(costmap_.metadata.size_x, costmap_.metadata.size_y);

  planner_->setCostmap(&costmap_.data[0], true, allow_unknown_);

  int map_start[2];
  map_start[0] = mx;
  map_start[1] = my;

  wx = goal.position.x;
  wy = goal.position.y;

  if (!worldToMap(wx, wy, mx, my))
  {
    RCLCPP_WARN(get_logger(),
                "The goal sent to the planner is off the global costmap."
                " Planning will always fail to this goal.");
    return false;
  }

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  // TODO(orduno): Explain why we are providing 'map_goal' to setStart().
  //               Same for setGoal, seems reversed. Computing backwards?

  planner_->setStart(map_goal);
  planner_->setGoal(map_start);
  planner_->calcNavFnAstar();

  /*
  if (use_astar_) {
    
  } else {
    planner_->calcNavFnDijkstra(true);
  }
  */

  double resolution = costmap_.metadata.resolution;
  geometry_msgs::msg::Pose p, best_pose;
  p = goal;

  bool found_legal = false;
  double best_sdist = std::numeric_limits<double>::max();

  p.position.y = goal.position.y - tolerance;

  while (p.position.y <= goal.position.y + tolerance)
  {
    p.position.x = goal.position.x - tolerance;
    while (p.position.x <= goal.position.x + tolerance)
    {
      double potential = getPointPotential(p.position);
      double sdist = squared_distance(p, goal);
      if (potential < POT_HIGH && sdist < best_sdist)
      {
        best_sdist = sdist;
        best_pose = p;
        found_legal = true;
      }
      p.position.x += resolution;
    }
    p.position.y += resolution;
  }

  if (found_legal)
  {
    // extract the plan
    if (getPlanFromPotential(best_pose, plan))
    {
      smoothApproachToGoal(best_pose, plan);
    }
    else
    {
      RCLCPP_ERROR(
          get_logger(),
          "Failed to create a plan from potential when a legal"
          " potential was found. This shouldn't happen.");
    }
  }

  return !plan.poses.empty();
}

void NavfnPlanner::smoothApproachToGoal(
    const geometry_msgs::msg::Pose &goal,
    nav2_msgs::msg::Path &plan)
{
  // Replace the last pose of the computed path if it's actually further away
  // to the second to last pose than the goal pose.

  auto second_to_last_pose = plan.poses.end()[-2];
  auto last_pose = plan.poses.back();
  if (
      squared_distance(last_pose, second_to_last_pose) >
      squared_distance(goal, second_to_last_pose))
  {
    plan.poses.back() = goal;
  }
  else
  {
    geometry_msgs::msg::Pose goal_copy = goal;
    plan.poses.push_back(goal_copy);
  }
}

bool NavfnPlanner::computePotential(const geometry_msgs::msg::Point &world_point)
{
  // make sure to resize the underlying array that Navfn uses
  planner_->setNavArr(costmap_.metadata.size_x, costmap_.metadata.size_y);

  std::vector<unsigned char> costmapData = std::vector<unsigned char>(
      costmap_.data.begin(), costmap_.data.end());

  planner_->setCostmap(&costmapData[0], true, allow_unknown_);

  unsigned int mx, my;
  if (!worldToMap(world_point.x, world_point.y, mx, my))
  {
    return false;
  }

  int map_start[2];
  map_start[0] = 0;
  map_start[1] = 0;

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  planner_->setStart(map_start);
  planner_->setGoal(map_goal);

  if (use_astar_)
  {
    return planner_->calcNavFnAstar();
  }

  return planner_->calcNavFnDijkstra();
}

bool NavfnPlanner::getPlanFromPotential(
    const geometry_msgs::msg::Pose &goal,
    nav2_msgs::msg::Path &plan)
{
  // clear the plan, just in case
  //plan.poses.clear();

  // Goal should be in global frame
  double wx = goal.position.x;
  double wy = goal.position.y;

  // the potential has already been computed, so we won't update our copy of the costmap
  unsigned int mx, my;
  if (!worldToMap(wx, wy, mx, my))
  {
    RCLCPP_WARN(
        get_logger(),
        "The goal sent to the navfn planner is off the global costmap."
        " Planning will always fail to this goal.");
    return false;
  }

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  planner_->setStart(map_goal);

  planner_->calcPath(costmap_.metadata.size_x * 4);

  // extract the plan
  float *x = planner_->getPathX();
  float *y = planner_->getPathY();
  int len = planner_->getPathLen();

  plan.header.stamp = this->now();
  plan.header.frame_id = global_frame_;

  for (int i = len - 1; i >= 0; --i)
  {
    // convert the plan to world coordinates
    double world_x, world_y;
    mapToWorld(x[i], y[i], world_x, world_y);

    geometry_msgs::msg::Pose pose;
    pose.position.x = world_x;
    pose.position.y = world_y;
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    plan.poses.push_back(pose);
  }

  return !plan.poses.empty();
}

double
NavfnPlanner::getPointPotential(const geometry_msgs::msg::Point &world_point)
{
  unsigned int mx, my;
  if (!worldToMap(world_point.x, world_point.y, mx, my))
  {
    return std::numeric_limits<double>::max();
  }

  unsigned int index = my * planner_->nx + mx;
  return planner_->potarr[index];
}

bool NavfnPlanner::validPointPotential(const geometry_msgs::msg::Point &world_point)
{
  return validPointPotential(world_point, tolerance_);
}

bool NavfnPlanner::validPointPotential(
    const geometry_msgs::msg::Point &world_point, double tolerance)
{
  double resolution = costmap_.metadata.resolution;

  geometry_msgs::msg::Point p = world_point;
  p.y = world_point.y - tolerance;

  while (p.y <= world_point.y + tolerance)
  {
    p.x = world_point.x - tolerance;
    while (p.x <= world_point.x + tolerance)
    {
      double potential = getPointPotential(p);
      if (potential < POT_HIGH)
      {
        return true;
      }
      p.x += resolution;
    }
    p.y += resolution;
  }

  return false;
}

bool NavfnPlanner::worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my)
{
  if (wx < costmap_.metadata.origin.position.x || wy < costmap_.metadata.origin.position.y)
  {
    RCLCPP_ERROR(get_logger(), "wordToMap failed: wx,wy: %f,%f, size_x,size_y: %d,%d", wx, wy,
                 costmap_.metadata.size_x, costmap_.metadata.size_y);
    return false;
  }

  mx = static_cast<int>(
      std::round((wx - costmap_.metadata.origin.position.x) / costmap_.metadata.resolution));
  my = static_cast<int>(
      std::round((wy - costmap_.metadata.origin.position.y) / costmap_.metadata.resolution));

  if (mx < costmap_.metadata.size_x && my < costmap_.metadata.size_y)
  {
    return true;
  }

  RCLCPP_ERROR(get_logger(), "wordToMap failed: mx,my: %d,%d, size_x,size_y: %d,%d", mx, my,
               costmap_.metadata.size_x, costmap_.metadata.size_y);

  return false;
}

void NavfnPlanner::mapToWorld(double mx, double my, double &wx, double &wy)
{
  wx = costmap_.metadata.origin.position.x + mx * costmap_.metadata.resolution;
  wy = costmap_.metadata.origin.position.y + my * costmap_.metadata.resolution;
}

void NavfnPlanner::clearRobotCell(unsigned int mx, unsigned int my)
{
  // TODO(orduno): check usage of this function, might instead be a request to
  //               world_model / map server
  unsigned int index = my * costmap_.metadata.size_x + mx;
  costmap_.data[index] = nav2_util::Costmap::free_space;
}

void NavfnPlanner::getCostmap(
    nav2_msgs::msg::Costmap &costmap,
    const std::string /*layer*/)
{
  // TODO(orduno): explicitly provide specifications for costmap using the costmap on the request,
  //               including master (aggregate) layer

  auto request = std::make_shared<nav2_util::CostmapServiceClient::CostmapServiceRequest>();
  request->specs.resolution = 1.0;

  auto result = costmap_client_.invoke(request, 5s);
  costmap = result.get()->map;
}

void NavfnPlanner::printCostmap(const nav2_msgs::msg::Costmap &costmap)
{
  std::cout << "Costmap" << std::endl;
  std::cout << "  size:       " << costmap.metadata.size_x << "," << costmap.metadata.size_x << std::endl;
  std::cout << "  origin:     " << costmap.metadata.origin.position.x << "," << costmap.metadata.origin.position.y << std::endl;
  std::cout << "  resolution: " << costmap.metadata.resolution << std::endl;
  std::cout << "  data:       "
            << "(" << costmap.data.size() << " cells)" << std::endl
            << "    ";

  const char separator = ' ';
  const int valueWidth = 4;

  unsigned int index = 0;
  for (unsigned int h = 0; h < costmap.metadata.size_y; ++h)
  {
    for (unsigned int w = 0; w < costmap.metadata.size_x; ++w)
    {
      std::cout << std::left << std::setw(valueWidth) << std::setfill(separator) << static_cast<unsigned int>(costmap.data[index]);
      index++;
    }
    std::cout << std::endl
              << "    ";
  }
  std::cout << std::endl;
}

void NavfnPlanner::publishPlan(const nav2_msgs::msg::Path &path)
{
  // Publish as a nav1 path msg
  nav_msgs::msg::Path rviz_path;

  rviz_path.header = path.header;
  rviz_path.poses.resize(path.poses.size());

  // Assuming path is already provided in world coordinates
  for (unsigned int i = 0; i < path.poses.size(); i++)
  {
    rviz_path.poses[i].header = path.header;
    rviz_path.poses[i].pose = path.poses[i];
  }

  plan_publisher_->publish(rviz_path);
}

} // namespace nav2_navfn_planner
