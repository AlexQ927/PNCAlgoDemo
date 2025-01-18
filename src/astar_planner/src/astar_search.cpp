#include "astar_search.hpp"

using namespace pnc;

IndexXY AstarSearchNode::cal_pose_index(const Pose &p, const Pose &ori)
{
  IndexXY idx{};
  idx.x = std::round((p.position.x - ori.position.x) / resolution);
  idx.y = std::round((p.position.y - ori.position.y) / resolution);
  return idx;
}

int AstarSearchNode::cal_grid_index(const AstarNode &node)
{
  return node.y * width + node.x;
}

double AstarSearchNode::calc_heuristic(const AstarNode &n1, const AstarNode &n2)
{
  double weight = 1.0;
  double d = weight * std::hypot(n1.x - n2.x, n1.y - n2.y);
  return 0.0;
}

bool AstarSearchNode::is_valid(const AstarNode &node)
{
  if (node.x < 0 || node.x >= width) return false;
  else if (node.y < 0 || node.y >= height) return false;
  else if ((cost_map->at(node.y).at(node.x)) != 0) return false;
  return true;
}

bool AstarSearchNode::find_final_path(const AstarNode &end_node)
{
  final_path.push_back(IndexXY(end_node.x, end_node.y));
  auto parent = end_node.parent;
  while (parent != -1) {
    auto node = close_map->at(parent);
    final_path.push_back(IndexXY(node.x, node.y));
    parent = node.parent;
  }
  return true;
}

bool AstarSearchNode::process_bufs()
{
  bool res = true;
  while (!grid_buf.empty() && !start_buf.empty() && !end_buf.empty()) {
    auto grid_msg = grid_buf.front();
    grid_buf.pop();
    auto start_msg = start_buf.front();
    start_buf.pop();
    auto end_msg = end_buf.front();
    end_buf.pop();
    if (res = process_synced_data(grid_msg, start_msg, end_msg) != true) break;
  }
  return res;
}

bool AstarSearchNode::process_synced_data(const OccupancyGrid &grid,
  const PoseStamped &start, const PoseStamped &end)
{
  height = grid.info.height;
  width = grid.info.width;
  resolution = grid.info.resolution;
  origin_pose = grid.info.origin;
  grid_map = std::make_unique<std::vector<int8_t>>(grid.data);
  for (auto i = 0; i < height; i++) {
    std::vector<int8_t> row;
    for (auto j = 0; j < width; j++) {
      row.push_back(grid_map->at(i * width + j));
    }
    cost_map->push_back(row);
  }
  for (auto i = -1; i <= 1; i++) {
    for (auto j = -1; j <= 1; j++) {
      if (i == 0 && j == 0) continue;
      double cost  = std::sqrt(i * i + j * j);
      auto motion_cost = MotionCost(i, j, cost);
      motion->push_back(motion_cost);
    }
  }
  start_pose = start.pose;
  auto start_idx = cal_pose_index(start_pose, origin_pose);
  end_pose = end.pose;
  auto end_idx = cal_pose_index(end_pose, origin_pose);
  return astar_search(start_idx, end_idx);
}

bool AstarSearchNode::astar_search(const IndexXY& start, const IndexXY& end)
{
  auto start_node = AstarNode(start.x, start.y, 0.0, -1);
  auto end_node = AstarNode(end.x, end.y, 0.0, -1);
  open_map->at(cal_grid_index(start_node)) = start_node;
  while (true) {
    if (open_map->empty()) {
      RCLCPP_ERROR(get_logger(), "Open map is empty..");
      break;
    }
    double min_cost = std::numeric_limits<double>::max();
    int curr_id;
    for(const auto& ele : *open_map) {
      auto hc = calc_heuristic(ele.second, end_node);
      if (hc < min_cost) {
        min_cost = hc;
        curr_id = ele.first;
      }
    }
    auto curr_node = open_map->at(curr_id);
    if (curr_node.x == end_node.x && curr_node.y == end_node.y) {
      RCLCPP_INFO(get_logger(), "Arrived!");
      end_node.parent = curr_node.parent;
      end_node.cost = curr_node.cost;
      break;
    }
    open_map->erase(curr_id);
    close_map->at(curr_id) = curr_node;
    for (const auto& m : *motion) {
      auto node = AstarNode(curr_node.x + m.i, curr_node.y + m.j, 
        curr_node.cost + m.cost, curr_id);
      auto node_id = cal_grid_index(node);

      if (!is_valid(node)) continue;
      else if (close_map->find(node_id) != close_map->end()) continue;

      if (open_map->find(node_id) == open_map->end() || 
          open_map->at(node_id).cost > node.cost) open_map->at(node_id) = node;
    }
  }
  return find_final_path(end_node);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr astar_search_node = std::make_shared<AstarSearchNode>();
  rclcpp::spin(astar_search_node);
  rclcpp::shutdown();
  return 0;
}