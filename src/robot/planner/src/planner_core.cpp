#include <algorithm>

#include "planner_core.hpp"

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) : path_(std::make_shared<nav_msgs::msg::Path>()), map_(std::make_shared<nav_msgs::msg::OccupancyGrid>()), logger_(logger) {}

void PlannerCore::initPlanner(double smoothingFactor, int iterations, double goalTolerance,
                              double goalSearchRadius, int clearanceCostThreshold,
                              int hardObstacleThreshold) {
  smoothingFactor_ = smoothingFactor;
  iterations_ = iterations;
  goalTolerance_ = goalTolerance;
  goalSearchRadius_ = goalSearchRadius;
  clearanceCostThreshold_ = clearanceCostThreshold;
  hardObstacleThreshold_ = hardObstacleThreshold;
}

int PlannerCore::cellCost(const CellIndex &idx) const {
  if (!map_ || idx.x < 0 || idx.x >= static_cast<int>(map_->info.width) ||
      idx.y < 0 || idx.y >= static_cast<int>(map_->info.height)) {
    return 127;
  }

  int mapIndex = idx.y * map_->info.width + idx.x;
  int8_t val = map_->data[mapIndex];
  if (val < 0) {
    val = 100;
  }
  return static_cast<int>(val);
}

bool PlannerCore::isTraversable(const CellIndex &idx) const {
  return cellCost(idx) < hardObstacleThreshold_;
}

bool PlannerCore::findSafeGoalCell(const CellIndex &goalIdx, CellIndex &safeGoalIdx) const {
  safeGoalIdx = goalIdx;
  if (isTraversable(goalIdx)) {
    return true;
  }

  if (!map_ || map_->info.resolution <= 0.0) {
    return false;
  }

  int searchRadiusCells = static_cast<int>(std::ceil(goalSearchRadius_ / map_->info.resolution));
  int toleranceRadiusCells = static_cast<int>(std::ceil(goalTolerance_ / map_->info.resolution));
  searchRadiusCells = std::max({searchRadiusCells, toleranceRadiusCells, 1});

  bool found = false;
  double bestScore = std::numeric_limits<double>::infinity();

  for (int dy = -searchRadiusCells; dy <= searchRadiusCells; ++dy) {
    for (int dx = -searchRadiusCells; dx <= searchRadiusCells; ++dx) {
      CellIndex candidate(goalIdx.x + dx, goalIdx.y + dy);
      if (candidate.x < 0 || candidate.x >= static_cast<int>(map_->info.width) ||
          candidate.y < 0 || candidate.y >= static_cast<int>(map_->info.height)) {
        continue;
      }

      double distanceCells = std::hypot(static_cast<double>(dx), static_cast<double>(dy));
      if (distanceCells > searchRadiusCells) {
        continue;
      }

      if (!isTraversable(candidate)) {
        continue;
      }

      double candidateCost = static_cast<double>(cellCost(candidate));
      double score = distanceCells + (candidateCost / 100.0);
      if (score < bestScore) {
        bestScore = score;
        safeGoalIdx = candidate;
        found = true;
      }
    }
  }

  return found;
}

bool PlannerCore::planPath(double startWorldX, double startWorldY, double goalX, double goalY,
                            nav_msgs::msg::OccupancyGrid::SharedPtr map) {
  // Store the map in the member variable
  map_ = map;

  // Convert current goal to map indices
  CellIndex goalIdx;
  if (!poseToMap(goalX, goalY, goalIdx)) {
    RCLCPP_WARN(logger_, "Goal is out of costmap bounds. Aborting.");
    return false;
  }

  CellIndex planningGoalIdx;
  if (!findSafeGoalCell(goalIdx, planningGoalIdx)) {
    RCLCPP_WARN(logger_, "Goal is inside a blocked region and no safe nearby target was found.");
    return false;
  }

  if (planningGoalIdx != goalIdx) {
    RCLCPP_WARN(logger_,
                "Goal cell (%d, %d) is too close to obstacles; planning to nearby safe cell (%d, %d).",
                goalIdx.x, goalIdx.y, planningGoalIdx.x, planningGoalIdx.y);
  }

  CellIndex startIdx;
  if (!poseToMap(startWorldX, startWorldY, startIdx)) {
    RCLCPP_WARN(logger_, "Start is out of costmap bounds. Aborting.");
    return false;
  }

  RCLCPP_INFO(logger_,
              "Planning from odom start (%.2f, %.2f) => cell (%d, %d) to goal (%d, %d).",
              startWorldX, startWorldY, startIdx.x, startIdx.y, planningGoalIdx.x, planningGoalIdx.y);

  // Run A*
  std::vector<CellIndex> pathCells;
  bool success = doAStar(startIdx, planningGoalIdx, pathCells);

  if (!success) {
    RCLCPP_WARN(logger_, "A* failed to find a path.");
    return false;
  }

  // Convert path cells to nav_msgs::Path
  if (!path_->poses.empty()) {
    path_->poses.clear();
  }

  for (auto &cell : pathCells) {
    geometry_msgs::msg::PoseStamped poseStamped;
    poseStamped.header = map_->header;

    double wx, wy;
    mapToPose(cell, wx, wy);

    poseStamped.pose.position.x = wx;
    poseStamped.pose.position.y = wy;
    poseStamped.pose.orientation.w = 1.0;

    path_->poses.push_back(poseStamped);
  }
  return true;
}

bool PlannerCore::doAStar(const CellIndex &startIdx, const CellIndex &goalIdx,
                          std::vector<CellIndex> &outPath) {
  const int width = map_->info.width;
  const int height = map_->info.height;

  // Preallocate maps with average expected size
  std::unordered_map<CellIndex, double, CellIndexHash> gScore;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> cameFrom;
  std::unordered_map<CellIndex, double, CellIndexHash> fScore;

  gScore.reserve(width * height / 10);  // Estimate ~10% of cells visited
  cameFrom.reserve(width * height / 10);
  fScore.reserve(width * height / 10);

  // Lambda functions to set and get scores
  auto setScore = [&](auto &storage, const CellIndex &idx, double val) {
    storage[idx] = val;
  };

  auto getScore = [&](auto &storage, const CellIndex &idx) {
    auto it = storage.find(idx);
    if (it == storage.end()) {
      return std::numeric_limits<double>::infinity();
    }
    return it->second;
  };

  // Initialize start node
  setScore(gScore, startIdx, 0.0);
  double hStart = euclideanHeuristic(startIdx, goalIdx);
  setScore(fScore, startIdx, hStart);

  // Open set (min-heap by f_score)
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> openSet;
  openSet.push(AStarNode(startIdx, hStart));

  while (!openSet.empty()) {
    AStarNode current = openSet.top();
    openSet.pop();
    CellIndex cidx = current.index;

    // Goal check
    if (cidx == goalIdx) {
      reconstructPath(cameFrom, cidx, outPath);
      return true;
    }

    double currentG = getScore(gScore, cidx);

    // Check neighbors (8-way)
    auto neighbors = getNeighbors8(cidx);
    for (auto &nb : neighbors) {
      if (nb.x < 0 || nb.x >= width || nb.y < 0 || nb.y >= height) {
        continue;
      }

      int costVal = cellCost(nb);
      if (costVal >= hardObstacleThreshold_) {
        continue;
      }

      // Step cost: 1.0 orth, sqrt(2) diag
      double stepCost = stepDistance(cidx, nb);
      // Stronger penalty so the planner prefers the center of free space.
      double normalizedCost = static_cast<double>(costVal) /
                              static_cast<double>(clearanceCostThreshold_);
      double penalty = stepCost * normalizedCost * normalizedCost * 8.0;
      if (costVal > clearanceCostThreshold_) {
        penalty += 5.0 * stepCost;
      }

      double tentativeG = currentG + stepCost + penalty;
      double oldG = getScore(gScore, nb);
      if (tentativeG < oldG) {
        setScore(gScore, nb, tentativeG);
        double h = euclideanHeuristic(nb, goalIdx);
        double f = tentativeG + h;
        setScore(fScore, nb, f);

        cameFrom[nb] = cidx;
        openSet.push(AStarNode(nb, f));
      }
    }
  }
  return false;  // No path found
}

void PlannerCore::reconstructPath(
    const std::unordered_map<CellIndex, CellIndex, CellIndexHash> &cameFrom,
    const CellIndex &current, std::vector<CellIndex> &outPath) {
  outPath.clear();
  CellIndex c = current;
  outPath.push_back(c);

  auto it = cameFrom.find(c);
  while (it != cameFrom.end()) {
    c = it->second;
    outPath.push_back(c);
    it = cameFrom.find(c);
  }
  std::reverse(outPath.begin(), outPath.end());
}

std::vector<CellIndex> PlannerCore::getNeighbors8(const CellIndex &c)
{
  std::vector<CellIndex> result;
  result.reserve(8);
  for (int dx = -1; dx <= 1; dx++) {
    for (int dy = -1; dy <= 1; dy++) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      result.push_back(CellIndex(c.x + dx, c.y + dy));
    }
  }
  return result;
}

double PlannerCore::euclideanHeuristic(const CellIndex &a, const CellIndex &b) {
  double dx = static_cast<double>(a.x - b.x);
  double dy = static_cast<double>(a.y - b.y);
  return std::sqrt(dx * dx + dy * dy);
}

double PlannerCore::stepDistance(const CellIndex &a, const CellIndex &b) {
  int dx = std::abs(a.x - b.x);
  int dy = std::abs(a.y - b.y);
  // Diagonal move: dx + dy == 2
  return (dx + dy == 2) ? std::sqrt(2.0) : 1.0;
}

bool PlannerCore::poseToMap(double wx, double wy, CellIndex &outIdx) {
  double originX = map_->info.origin.position.x;
  double originY = map_->info.origin.position.y;
  double res = map_->info.resolution;

  double mx = (wx - originX) / res;
  double my = (wy - originY) / res;

  int ix = static_cast<int>(std::floor(mx));
  int iy = static_cast<int>(std::floor(my));

  if (ix < 0 || ix >= static_cast<int>(map_->info.width) ||
      iy < 0 || iy >= static_cast<int>(map_->info.height)) {
    return false;
  }

  outIdx.x = ix;
  outIdx.y = iy;
  return true;
}

void PlannerCore::mapToPose(const CellIndex &idx, double &wx, double &wy) {
  double originX = map_->info.origin.position.x;
  double originY = map_->info.origin.position.y;
  double res = map_->info.resolution;

  wx = originX + (idx.x + 0.5) * res;
  wy = originY + (idx.y + 0.5) * res;
}

nav_msgs::msg::Path::SharedPtr PlannerCore::getPath() const {
  return path_;
}

} 