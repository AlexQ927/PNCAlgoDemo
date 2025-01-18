#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>
#include <queue>
#include <stdexcept>
#include <memory>
#include <limits>
#include <unordered_map>
#include "astar_planner/msg/astar_path.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace pnc
{

using nav_msgs::msg::OccupancyGrid;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Pose;
using astar_planner::msg::AstarPath;

struct AstarNode {
  int x;
  int y;
  double cost;
  // AstarNode * p_node;
  int parent;
  AstarNode(): x(0), y(0), cost(0.0), parent(-1) {}
  AstarNode(int in_x, int in_y, double in_cost, int in_parent)
  : x(in_x), y(in_y), cost(in_cost), parent(in_parent) {}
};

struct MotionCost {
  int8_t i;
  int8_t j;
  double cost;
  MotionCost() : i(0), j(0), cost(0.0) {}
  MotionCost(int8_t in_i, int8_t in_j, double in_cost)
  : i(in_i), j(in_j), cost(in_cost) {}
};

struct IndexXY {
  int x;
  int y;
  IndexXY() : x(0), y(0) {}
  IndexXY(int in_x, int in_y) : x(in_x), y(in_y) {}
};

class AstarSearchNode : public rclcpp::Node 
{
public:
  AstarSearchNode()
  : Node("astar_search_node") {
    cost_map = std::make_unique<std::vector<std::vector<int8_t>>>();
    motion = std::make_unique<std::vector<MotionCost>>();
    open_map = std::make_unique<std::unordered_map<int, AstarNode>>();
    close_map = std::make_unique<std::unordered_map<int, AstarNode>>();
    this->grid_sub = this->create_subscription<OccupancyGrid>(
      "grid_map", 10, std::bind(&AstarSearchNode::grid_callback, this, _1));
    this->start_sub = this->create_subscription<PoseStamped>(
      "start_pose", 10, std::bind(&AstarSearchNode::start_callback, this, _1));
    this->end_sub = this->create_subscription<PoseStamped>(
      "end_pose", 10, std::bind(&AstarSearchNode::end_callback, this, _1));
    this->astar_pub = this->create_publisher<AstarPath>("astar_path", 10);
    this->timer = this->create_wall_timer(
      500ms, std::bind(&AstarSearchNode::timer_callback, this));
  }
  std::vector<Pose> get_final_path() { return final_path; }
private:
  void grid_callback(const OccupancyGrid& grid)
  {
    this->grid_buf.push(grid);
    this->process_bufs();
  }
  void start_callback(const PoseStamped& start)
  {
    this->start_buf.push(start);
    this->process_bufs();
  }
  void end_callback(const PoseStamped& end)
  {
    this->end_buf.push(end);
    this->process_bufs();
  }
  void timer_callback();
  void init_data();
  IndexXY cal_pose_index(const Pose& p, const Pose& ori);
  int cal_grid_index(const AstarNode& node);
  Pose cal_node_pose(const AstarNode& node);
  double calc_heuristic(const AstarNode& n1, const AstarNode& n2);
  bool is_valid(const AstarNode& node);
  bool find_final_path(const AstarNode& end_node);
  bool process_bufs();
  bool process_synced_data(const OccupancyGrid& grid, const PoseStamped& start, 
                           const PoseStamped& end);
  bool astar_search(const IndexXY& start, const IndexXY& end);

private:
  bool is_arrived = false;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr grid_sub{};
  rclcpp::Subscription<PoseStamped>::SharedPtr start_sub{};
  rclcpp::Subscription<PoseStamped>::SharedPtr end_sub{};
  rclcpp::Publisher<AstarPath>::SharedPtr astar_pub{};
  rclcpp::TimerBase::SharedPtr timer{};
  std::queue<OccupancyGrid> grid_buf{};
  std::queue<PoseStamped> start_buf{};
  std::queue<PoseStamped> end_buf{};
  int height{};
  int width{};
  double resolution{};
  Pose origin_pose{};
  Pose start_pose{};
  Pose end_pose{};
  std::vector<int8_t> grid_map{};
  std::unique_ptr<std::vector<std::vector<int8_t>>> cost_map{};
  std::unique_ptr<std::vector<MotionCost>> motion{};
  std::unique_ptr<std::unordered_map<int, AstarNode>> open_map{};
  std::unique_ptr<std::unordered_map<int, AstarNode>> close_map{};
  std::vector<IndexXY> final_path_idx{};
  std::vector<Pose> final_path{};
  AstarPath message{};
};// class astar_search_node
} // namespace pnc

