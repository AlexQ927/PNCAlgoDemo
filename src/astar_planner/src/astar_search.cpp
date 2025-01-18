#include "astar_search.hpp"

void pnc::AstarSearchNode::process_bufs()
{
  while (!grid_buf.empty() && !start_buf.empty() && !end_buf.empty()) {
    auto grid_msg = grid_buf.front();
    grid_buf.pop();
    auto start_msg = start_buf.front();
    start_buf.pop();
    auto end_msg = end_buf.front();
    end_buf.pop();
    process_synced_data(grid_msg, start_msg, end_msg);
  }
}

void pnc::AstarSearchNode::process_synced_data(const OccupancyGrid &grid,
  const PoseStamped &start, const PoseStamped &end)
{
  height = grid.info.height;
  width = grid.info.width;
  resolution = grid.info.resolution;
  origin_pose = grid.info.origin;
  i_x0 = std::round(origin_pose.position.x / resolution);
  i_y0 = std::round(origin_pose.position.y / resolution);
  grid_map = std::make_unique<std::vector<int8_t>>(grid.data);
  for (int i = 0; i < height; i++) {
    std::vector<int8_t> row;
    for (int j = 0; j < width; j++) {
      row.push_back(grid_map->at(i * width + j));
    }
    cost_map->push_back(row);
  }
  for (int i = -1; i <= 1; i++) {
    for (int j = -1; j <= 1; j++) {
      if (i == 0 && j == 0) continue;
      double cost  = std::sqrt(i * i + j * j);
      auto motion_cost = MotionCost(i, j, cost);
      motion->push_back(motion_cost);
    }
  }
  start_pose = start.pose;
  end_pose = end.pose;
}
