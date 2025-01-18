#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>
#include <queue>
#include <stdexcept>
#include <memory>

using std::placeholders::_1;

namespace pnc
{

using nav_msgs::msg::OccupancyGrid;
using geometry_msgs::msg::PoseStamped;

struct AstarNode {
  int x;
  int y;
  double cost;
  // AstarNode * p_node;
  int parent;
  AstarNode(int in_x, int in_y, double in_cost, int in_parent)
  : x(in_x), y(in_y), cost(in_cost), parent(in_parent){}
};

struct MotionCost {
  int8_t i;
  int8_t j;
  double cost;
  MotionCost(int8_t in_i, int8_t in_j, double in_cost)
  : i(in_i), j(in_j), cost(in_cost){}
};

struct IndexXY {
  int x;
  int y;
  IndexXY(int in_x, int in_y) : x(in_x), y(in_y) {}
};

class AstarSearchNode : public rclcpp::Node 
{
public:
  AstarSearchNode()
  : Node("astar_search_node") {
    this->grid_sub = this->create_subscription<OccupancyGrid>(
      "grid_map", 10, std::bind(&AstarSearchNode::grid_callback, this, _1));
    this->start_sub = this->create_subscription<PoseStamped>(
      "start_pose", 10, std::bind(&AstarSearchNode::start_callback, this, _1));
    this->end_sub = this->create_subscription<PoseStamped>(
      "end_pose", 10, std::bind(&AstarSearchNode::end_callback, this, _1));
  }
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
  int cal_pose_index(double x1, double x0);
  int cal_grid_index(const AstarNode& node);
  bool is_valid(const AstarNode& node);

  void process_bufs();
  void process_synced_data(const OccupancyGrid& grid, const PoseStamped& start, 
                           const PoseStamped& end);
  bool astar_search();
  rclcpp::Subscription<OccupancyGrid>::SharedPtr grid_sub{};
  rclcpp::Subscription<PoseStamped>::SharedPtr start_sub{};
  rclcpp::Subscription<PoseStamped>::SharedPtr end_sub{};
  std::queue<OccupancyGrid> grid_buf{};
  std::queue<PoseStamped> start_buf{};
  std::queue<PoseStamped> end_buf{};
  uint32_t height{};
  uint32_t width{};
  double resolution{};
  geometry_msgs::msg::Pose origin_pose;
  int i_x0{};
  int i_y0{};
  geometry_msgs::msg::Pose start_pose;
  geometry_msgs::msg::Pose end_pose;
  std::unique_ptr<std::vector<int8_t>> grid_map{};
  std::unique_ptr<std::vector<std::vector<int8_t>>> cost_map{};
  std::unique_ptr<std::vector<MotionCost>> motion{};
  std::unique_ptr<std::vector<IndexXY>> final_path{};
};// class astar_search_node
} // namespace pnc

