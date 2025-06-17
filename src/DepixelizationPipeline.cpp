#include "DepixelizationPipeline.h"

#include <cassert>
#include <map>
#include <tuple>

#include <glm/glm.hpp>

#include "Bezier.h"
#include "Svg.h"

bool DepixelizationPipeline::loadImage(char* path) {
  m_image.release();
  m_image = cv::imread(path, cv::IMREAD_UNCHANGED);

  if (m_image.empty()) {
    return false;
  }

  // convert BGR(A) to RGB(A)
  if (m_image.channels() == 4) {
    cv::cvtColor(m_image, m_image, cv::COLOR_BGRA2RGBA);
  } else if (m_image.channels() == 3) {
    cv::cvtColor(m_image, m_image, cv::COLOR_BGR2RGB);
  }

  return true;
}

std::pair<int, int> DepixelizationPipeline::indexToCoordinate(int index) const {
  int x = index % m_image.cols;
  int y = index / m_image.cols;

  if (x >= m_image.cols || y >= m_image.rows) return {-10, -10};

  return {x, y};
}

int DepixelizationPipeline::coordinateToIndex(int x, int y) const { return y * m_image.cols + x; }

bool DepixelizationPipeline::hasSimilarityEdge(int idx1, int idx2) const {
  return std::find(m_similarity[idx1].begin(), m_similarity[idx1].end(), idx2) !=
         m_similarity[idx1].end();
}

void DepixelizationPipeline::addSimilarityEdge(int idx1, int idx2) {
  m_similarity[idx1].push_back(idx2);
  m_similarity[idx2].push_back(idx1);
}

void DepixelizationPipeline::removeSimilarityEdge(int idx1, int idx2) {
  m_similarity[idx1].erase(
      std::remove(m_similarity[idx1].begin(), m_similarity[idx1].end(), idx2),
      m_similarity[idx1].end()
  );

  m_similarity[idx2].erase(
      std::remove(m_similarity[idx2].begin(), m_similarity[idx2].end(), idx1),
      m_similarity[idx2].end()
  );
}

void DepixelizationPipeline::computeSimilarityGraph(float similarityThreshold) {
  int height = m_image.rows;
  int width = m_image.cols;

  m_similarity.clear();
  m_similarity.resize(height * width);

  // compute similarity graph
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      cv::Vec4f color = m_image.at<cv::Vec4b>(y, x);

      // check neighboring pixel colors and compute color distances
      for (int dx = 1; dx >= -1; --dx) {
        for (int dy = 1; dy >= -1; --dy) {
          if (dx == 0 && dy == 0) continue;  // skip the center pixel

          int neigh_x = x + dx;
          int neigh_y = y + dy;

          if (neigh_x < 0 || neigh_x >= width || neigh_y < 0 || neigh_y >= height) continue;

          cv::Vec4f neighbor_color = m_image.at<cv::Vec4b>(neigh_y, neigh_x);
          if (cv::norm(color - neighbor_color) <= similarityThreshold) {
            int cur_idx = coordinateToIndex(x, y);
            int neigh_idx = coordinateToIndex(neigh_x, neigh_y);
            bool can_add_edge = true;

            // diagonal-antidiagonal intersection resolution
            if (dy == -1 && dx == -1) {
              std::pair<int, int> antidiagonal = {
                  coordinateToIndex(x, y - 1), coordinateToIndex(x - 1, y)
              };

              if (hasSimilarityEdge(antidiagonal.first, antidiagonal.second)) {
                cv::Vec4f diagonal_color = (color + neighbor_color) / 2.0f;
                cv::Vec4f antidiagonal_color = ((cv::Vec4f)m_image.at<cv::Vec4b>(y - 1, x) +
                                                (cv::Vec4f)m_image.at<cv::Vec4b>(y, x - 1)) /
                                               2.0f;

                if (cv::norm(diagonal_color - antidiagonal_color) <= similarityThreshold) {
                  removeSimilarityEdge(antidiagonal.first, antidiagonal.second);
                  removeSimilarityEdge(neigh_idx, cur_idx);
                  can_add_edge = false;
                } else {
                  int min_cardinality_diag =
                      std::min(m_similarity[cur_idx].size(), m_similarity[neigh_idx].size());

                  int min_cardinality_antidiag = std::min(
                      m_similarity[antidiagonal.first].size(),
                      m_similarity[antidiagonal.second].size()
                  );

                  AmbiguousCrossing amb{{cur_idx, neigh_idx}, antidiagonal, false};

                  if (min_cardinality_diag <= min_cardinality_antidiag) {
                    can_add_edge = true;
                    removeSimilarityEdge(antidiagonal.first, antidiagonal.second);
                    amb.diagonal_kept = true;
                  } else {
                    can_add_edge = false;
                    removeSimilarityEdge(cur_idx, neigh_idx);
                    amb.diagonal_kept = false;
                  }

                  m_ambiguousCrossings.push_back(amb);
                  m_ambiguousCrossingsPositions[{x, y}] = m_ambiguousCrossings.size() - 1;
                }
              }
            }

            if (can_add_edge && !hasSimilarityEdge(cur_idx, neigh_idx)) {
              addSimilarityEdge(cur_idx, neigh_idx);
            }
          }
        }
      }
    }
  }
}

void DepixelizationPipeline::getSimilarityGraphBuffers(
    std::vector<float>& vertices, std::vector<unsigned int>& indices,
    std::vector<float>& ambiguousCrossings, std::vector<unsigned int>& ambiguousCrossingsIndices
) const {
  int height = m_image.rows;
  int width = m_image.cols;

  vertices.clear();
  indices.clear();

  // generate vertices for each pixel center
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int idx = coordinateToIndex(x, y);
      vertices.push_back((x + 0.5f) / static_cast<float>(width) * 2.0f - 1.0f);   // x-coordinate
      vertices.push_back(1.0f - (y + 0.5f) / static_cast<float>(height) * 2.0f);  // y-coordinate

      // add indices for edges in only one direction to avoid duplicates
      for (int neighbor : m_similarity[idx]) {
        if (idx > neighbor) continue;  // avoid duplicate edges
        indices.push_back(idx);
        indices.push_back(neighbor);
      }
    }
  }

  // generate ambiguous crossings data
  ambiguousCrossings.clear();
  ambiguousCrossingsIndices.clear();

  for (const auto& [pos, crossingIdx] : m_ambiguousCrossingsPositions) {
    std::vector<std::pair<float, float>> coords;
    coords.push_back({pos.first - 0.25f, pos.second - 0.25f});
    coords.push_back({pos.first + 0.25f, pos.second + 0.25f});
    coords.push_back({pos.first + 0.25f, pos.second - 0.25f});
    coords.push_back({pos.first - 0.25f, pos.second + 0.25f});

    // add kept edge first, then the other one
    if (!m_ambiguousCrossings[crossingIdx].diagonal_kept) {
      std::swap(coords[0], coords[2]);
      std::swap(coords[1], coords[3]);
    }

    for (const auto& coord : coords) {
      ambiguousCrossings.push_back((coord.first) / static_cast<float>(width) * 2.0f - 1.0f);
      ambiguousCrossings.push_back(1.0f - (coord.second) / static_cast<float>(height) * 2.0f);
    }

    unsigned int vertexIdx = ambiguousCrossings.size() / 2 - 4;

    // store indices
    ambiguousCrossingsIndices.push_back(vertexIdx);
    ambiguousCrossingsIndices.push_back(vertexIdx + 1);

    ambiguousCrossingsIndices.push_back(vertexIdx + 2);
    ambiguousCrossingsIndices.push_back(vertexIdx + 3);
  }
}

void DepixelizationPipeline::findPixelClusters() {
  m_clusters.clear();

  int num_pixels = m_image.rows * m_image.cols;

  std::vector<bool> visited(num_pixels, false);

  std::function<cv::Vec4f(int, std::set<int>&, cv::Vec4f)> findPixelClusterDFS =
      [&](int idx, std::set<int>& cluster, cv::Vec4f sum) {
        if (visited[idx]) {
          return sum;  // already visited
        }

        // mark as visited and add to cluster
        visited[idx] = true;
        cluster.insert(idx);

        // add this pixel color to sum
        auto [x, y] = indexToCoordinate(idx);
        sum += m_image.at<cv::Vec4b>(y, x);

        // recursively visit neighbors
        for (int neigh : m_similarity[idx]) {
          if (!visited[neigh]) {
            sum = findPixelClusterDFS(neigh, cluster, sum);
          }
        }

        return sum;
      };

  for (int i = 0; i < num_pixels; ++i) {
    if (!visited[i]) {
      std::set<int> cluster;
      cv::Vec4f color_sum = findPixelClusterDFS(i, cluster, cv::Vec4f(0.0f));

      m_clusters.push_back({cluster, color_sum / static_cast<float>(cluster.size())});
    }
  }
}

void DepixelizationPipeline::findExternalBoundary(int clusterIdx) {
  PixelCluster& cluster = m_clusters[clusterIdx];
  cluster.externalBoundary.clear();

  // start from extreme left node
  int extreme_node = -1;
  for (int node : cluster.boundaryNodes) {
    if (extreme_node == -1 || m_pathGraph[node].pos.x < m_pathGraph[extreme_node].pos.x) {
      extreme_node = node;
    }
  }

  int prev = -1;
  int curr = extreme_node;
  do {
    cluster.externalBoundary.push_back(curr);
    int next = -1;

    for (int neigh : m_pathGraph[curr].neighbors) {
      // only pick neighbors that are in this cluster's boundary,
      // and skip going straight back to prev
      if (!cluster.boundaryNodes.count(neigh)) continue;
      if (neigh == prev) continue;

      next = neigh;
      break;
    }

    assert(next != -1 && "Error:No next node found in boundary traversal");

    prev = curr;
    curr = next;
  } while (curr != extreme_node);
}

void DepixelizationPipeline::colorClusters() {
  m_result = cv::Mat::zeros(m_image.size(), m_image.type());

  m_pixelToClusterMap.clear();
  m_pixelToClusterMap.resize(m_result.rows * m_result.cols, -1);

  for (int i = 0; i < m_clusters.size(); ++i) {
    const auto& cluster = m_clusters[i];

    for (int pixel : cluster.pixels) {
      auto [x, y] = indexToCoordinate(pixel);
      m_result.at<cv::Vec4b>(y, x) = cluster.avgColor;
      m_pixelToClusterMap[pixel] = i;
    }
  }
}

int DepixelizationPipeline::createPathNode(FractionalCoord fracPos, PathGraphNode::Type type) {
  m_pathGraph.push_back(PathGraphNode(fracPos, type));
  return static_cast<int>(m_pathGraph.size()) - 1;
}

void DepixelizationPipeline::addPathNodeNeighbor(int nodeIdx, int neighborIdx) {
  m_pathGraph[nodeIdx].neighbors.insert(neighborIdx);
  m_pathGraph[neighborIdx].neighbors.insert(nodeIdx);
}

void DepixelizationPipeline::removePathNodeNeighbor(int nodeIdx, int neighborIdx) {
  m_pathGraph[nodeIdx].neighbors.erase(neighborIdx);
  m_pathGraph[neighborIdx].neighbors.erase(nodeIdx);
}

int DepixelizationPipeline::getOrCreatePathNode(
    std::map<FractionalCoord, int>& path_node_map, FractionalCoord pos, PathGraphNode::Type type,
    int clusterIdx
) {
  if (path_node_map.find(pos) == path_node_map.end())
    path_node_map[pos] = createPathNode(pos, type);

  m_clusters[clusterIdx].boundaryNodes.insert(path_node_map[pos]);
  m_pathGraph[path_node_map[pos]].clusters.insert(clusterIdx);

  return path_node_map[pos];
}

void DepixelizationPipeline::createBoundaryNodes(
    std::map<FractionalCoord, int>& path_node_map, int x, int y, bool vertical, int clusterIdx
) {
  // corner nodes
  int corner1 =
      getOrCreatePathNode(path_node_map, {{x, 0}, {y, 0}}, PathGraphNode::CORNER, clusterIdx);
  int corner2 = getOrCreatePathNode(
      path_node_map, {{x + (vertical ? 0 : 1), 0}, {y + (vertical ? 1 : 0), 0}},
      PathGraphNode::CORNER, clusterIdx
  );

  // edge nodes
  int edges[EDGE_NODES_PER_PIXEL];
  for (int i = 1; i <= EDGE_NODES_PER_PIXEL; ++i) {
    FractionalCoord pos = {{x, vertical ? 0 : i}, {y, vertical ? i : 0}};
    edges[i - 1] = getOrCreatePathNode(path_node_map, pos, PathGraphNode::EDGE, clusterIdx);
  }

  // add neighbors
  addPathNodeNeighbor(corner1, edges[0]);
  for (int i = 0; i < EDGE_NODES_PER_PIXEL - 1; ++i) {
    addPathNodeNeighbor(edges[i], edges[i + 1]);
  }
  addPathNodeNeighbor(edges[EDGE_NODES_PER_PIXEL - 1], corner2);
}

void DepixelizationPipeline::computePathGeneration() {
  m_pathGraph.clear();

  findPixelClusters();
  colorClusters();

  // generate path graph nodes for each cluster
  std::map<FractionalCoord, int> path_node_map;
  for (int i = 0; i < m_clusters.size(); ++i) {
    PixelCluster& cluster = m_clusters[i];
    cv::Vec4b avg_color = cluster.avgColor;

    for (int pixel : cluster.pixels) {
      auto [x, y] = indexToCoordinate(pixel);

      // check in what boundaries the pixel is
      bool is_left_boundary = (x == 0 || m_result.at<cv::Vec4b>(y, x - 1) != avg_color);
      bool is_right_boundary =
          (x == m_image.cols - 1 || m_result.at<cv::Vec4b>(y, x + 1) != avg_color);
      bool is_top_boundary = (y == 0 || m_result.at<cv::Vec4b>(y - 1, x) != avg_color);
      bool is_bottom_boundary =
          (y == m_image.rows - 1 || m_result.at<cv::Vec4b>(y + 1, x) != avg_color);

      // for each boundary, create the appropriate nodes
      if (is_left_boundary) createBoundaryNodes(path_node_map, x, y, true, i);
      if (is_right_boundary) createBoundaryNodes(path_node_map, x + 1, y, true, i);
      if (is_top_boundary) createBoundaryNodes(path_node_map, x, y, false, i);
      if (is_bottom_boundary) createBoundaryNodes(path_node_map, x, y + 1, false, i);
    }
  }

  // diagonal links treatment
  std::set<std::tuple<FractionalCoord, int, int, std::set<int>>> nodes_to_create;
  for (int i = 0; i < m_pathGraph.size(); ++i) {
    auto& node = m_pathGraph[i];
    if (node.type != PathGraphNode::CORNER) continue;

    // every corner is at a integer coordinate
    int x = node.originalPos.first.first;
    int y = node.originalPos.second.first;

    if (x < 1 || y < 1 || x > m_image.cols - 1 || y > m_image.rows - 1)
      continue;  // skip if out of bounds

    int cur_idx_similarity = coordinateToIndex(x, y);
    int idx_top_left = coordinateToIndex(x - 1, y - 1);
    int idx_top_right = coordinateToIndex(x, y - 1);
    int idx_left = coordinateToIndex(x - 1, y);

    // create a diagonal link
    if (hasSimilarityEdge(cur_idx_similarity, idx_top_left)) {
      // L shape link -- skip
      if (hasSimilarityEdge(cur_idx_similarity, idx_left) ||
          hasSimilarityEdge(cur_idx_similarity, idx_top_right)) {
        continue;
      }

      // skip if neighbors do not exist (close colors -- non transitive)
      if (!path_node_map.count({{x, 1}, {y, 0}}) ||
          !path_node_map.count({{x, 0}, {y - 1, EDGE_NODES_PER_PIXEL}})) {
        continue;
      }

      if (node.neighbors.size() == 2) continue;  // skip if only has two neighbors (no change)

      // remove top and right neighbors from the original node and link to duplicate
      int right_neighbor = path_node_map.at({{x, 1}, {y, 0}});
      int top_neighbor = path_node_map.at({{x, 0}, {y - 1, EDGE_NODES_PER_PIXEL}});

      removePathNodeNeighbor(i, right_neighbor);
      removePathNodeNeighbor(i, top_neighbor);

      // update clusters (if the opposite clusters are different)
      std::set<int> new_clusters = node.clusters;

      // remove the original node from the top right cluster
      if (m_pixelToClusterMap[idx_top_right] != m_pixelToClusterMap[idx_left]) {
        new_clusters.erase(m_pixelToClusterMap[idx_left]);

        m_clusters[m_pixelToClusterMap[idx_top_right]].boundaryNodes.erase(i);
        node.clusters.erase(m_pixelToClusterMap[idx_top_right]);
      }

      nodes_to_create.insert(std::make_tuple(
          FractionalCoord{{x, 0}, {y, 0}}, right_neighbor, top_neighbor, new_clusters
      ));
    } else if (hasSimilarityEdge(idx_left, idx_top_right)) {
      // L shape link
      if (hasSimilarityEdge(idx_left, cur_idx_similarity) ||
          hasSimilarityEdge(idx_left, idx_top_left)) {
        continue;
      }

      if (!path_node_map.count({{x - 1, EDGE_NODES_PER_PIXEL}, {y, 0}}) ||
          !path_node_map.count({{x, 0}, {y - 1, EDGE_NODES_PER_PIXEL}})) {
        continue;
      }

      if (node.neighbors.size() == 2) continue;

      // remove left and top neighbors from the original node and link to duplicate
      int left_neighbor = path_node_map.at({{x - 1, EDGE_NODES_PER_PIXEL}, {y, 0}});
      int top_neighbor = path_node_map.at({{x, 0}, {y - 1, EDGE_NODES_PER_PIXEL}});

      removePathNodeNeighbor(i, left_neighbor);
      removePathNodeNeighbor(i, top_neighbor);

      // update clusters (if the opposite clusters are different)
      std::set<int> new_clusters = node.clusters;

      // remove the original node from the top left cluster
      if (m_pixelToClusterMap[idx_top_left] != m_pixelToClusterMap[cur_idx_similarity]) {
        new_clusters.erase(m_pixelToClusterMap[cur_idx_similarity]);

        m_clusters[m_pixelToClusterMap[idx_top_left]].boundaryNodes.erase(i);
        node.clusters.erase(m_pixelToClusterMap[idx_top_left]);
      }

      nodes_to_create.insert(std::make_tuple(
          FractionalCoord{{x, 0}, {y, 0}}, left_neighbor, top_neighbor, new_clusters
      ));
    }
  }

  // create duplicated corner nodes for diagonal links
  for (auto [pos, neigh1, neigh2, clusters] : nodes_to_create) {
    int corner_dup = createPathNode(pos, PathGraphNode::CORNER);

    addPathNodeNeighbor(corner_dup, neigh1);
    addPathNodeNeighbor(corner_dup, neigh2);

    m_pathGraph[corner_dup].clusters = clusters;
    for (int clusterIdx : clusters) {
      m_clusters[clusterIdx].boundaryNodes.insert(corner_dup);
    }
  }

  for (int i = 0; i < m_clusters.size(); ++i) {
    findExternalBoundary(i);
    updateClusterArea(i, true);
  }
}

void DepixelizationPipeline::getPathGraphBuffers(
    std::vector<float>& vertices, std::vector<unsigned int>& indices
) const {
  int height = m_image.rows;
  int width = m_image.cols;

  vertices.clear();
  indices.clear();

  for (const auto& node : m_pathGraph) {
    vertices.push_back(node.pos.x / static_cast<float>(width) * 2.0f - 1.0f);
    vertices.push_back(1.0f - node.pos.y / static_cast<float>(height) * 2.0f);
  }

  for (int i = 0; i < m_pathGraph.size(); ++i) {
    for (int neighbor : m_pathGraph[i].neighbors) {
      if (i > neighbor) continue;
      indices.push_back(i);
      indices.push_back(neighbor);
    }
  }
}

void DepixelizationPipeline::updateClusterArea(int clusterIdx, bool initial) {
  // shoelace formula
  const PixelCluster& cluster = m_clusters[clusterIdx];
  int n_nodes = cluster.externalBoundary.size();
  float area = 0.0f;
  glm::vec2 centroid(0.0f, 0.0f);

  for (int i = 0; i < n_nodes; ++i) {
    auto n1 = m_pathGraph[cluster.externalBoundary[i]];
    auto n2 = m_pathGraph[cluster.externalBoundary[(i + 1) % n_nodes]];

    area += (n1.pos.x + n2.pos.x) * (n1.pos.y - n2.pos.y);
    centroid += n1.pos;
  }

  area /= 2.0f;
  centroid /= static_cast<float>(n_nodes);

  m_clusters[clusterIdx].area = std::abs(area);
  if (initial) m_clusters[clusterIdx].initialArea = m_clusters[clusterIdx].area;

  m_clusters[clusterIdx].centroid = centroid;
}

float DepixelizationPipeline::calculateNodeForces() {
  m_nodeForces.clear();
  m_nodeForces.resize(m_pathGraph.size());

  float f_max_abs = 0.0f;
  for (int i = 0; i < m_pathGraph.size(); ++i) {
    const PathGraphNode& node = m_pathGraph[i];

    // Fo calculation
    glm::vec2 originalPos{
        fractionToFloat(node.originalPos.first), fractionToFloat(node.originalPos.second)
    };
    glm::vec2 fo = (originalPos - node.pos) * glm::length(originalPos - node.pos) * node.Ko;
    m_nodeForces[i] += fo;

    // neighbors force
    for (auto& neigh_idx : node.neighbors) {
      m_nodeForces[i] += (m_pathGraph[neigh_idx].pos - node.pos) * node.Kn;
    }

    // area force
    for (int clusterIdx : node.clusters) {
      const PixelCluster& cluster = m_clusters[clusterIdx];

      m_nodeForces[i] +=
          (node.pos - cluster.centroid) * (1.0f - std::sqrt(cluster.area / cluster.initialArea));
    }

    f_max_abs = std::max(f_max_abs, std::abs(glm::length(m_nodeForces[i])));
  }

  return f_max_abs;
}

void DepixelizationPipeline::computeSpringSimulation() {
  // compute Ko for each node
  for (auto& node : m_pathGraph) {
    node.resetOriginSpringStiffness();
  }

  // compute simulation
  float f_max = INFINITY;  // always compute first round
  while (f_max > STOPPING_THRESHOLD) {
    f_max = calculateNodeForces();
    float step = MAX_STEP_SIZE / std::max(1.0f, f_max);

    for (int i = 0; i < m_pathGraph.size(); ++i) m_pathGraph[i].pos += step * m_nodeForces[i];

    // update cluster areas
    for (int i = 0; i < m_clusters.size(); ++i) {
      updateClusterArea(i);
    }
  }

  // order clusters by area (largest first)
  std::sort(m_clusters.begin(), m_clusters.end(), [](const PixelCluster& a, const PixelCluster& b) {
    return a.area > b.area;
  });
}

void DepixelizationPipeline::exportSvg(const std::string& filename) const {
  Svg svg(m_image.cols, m_image.rows);

  for (const auto& cluster : m_clusters) {
    if (cluster.externalBoundary.empty()) continue;

    std::vector<cv::Point2d> points;
    for (int nodeIdx : cluster.externalBoundary) {
      const auto& node = m_pathGraph[nodeIdx];
      points.emplace_back(node.pos.x, node.pos.y);
    }

    points.push_back(points.front());
    std::vector<std::vector<cv::Point2d>> ctrl;
    BezierCurve::fit(points, ctrl);

    svg.startPath();

    bool first = true;
    for (const auto& segment : ctrl) {
      svg.addCurveToPath(segment, first);
      first = false;
    }

    svg.finalizePath(cluster.avgColor);
  }

  svg.writeToFile(filename);
}

bool DepixelizationPipeline::checkAmbiguousCrossingCollision(const glm::vec2& pos) {
  // check if the position is within any ambiguous crossing
  std::pair<int, int> posTrunc = {std::round(pos.x), std::round(pos.y)};

  if (m_ambiguousCrossingsPositions.find(posTrunc) != m_ambiguousCrossingsPositions.end()) {
    int crossingIdx = m_ambiguousCrossingsPositions[posTrunc];
    AmbiguousCrossing& crossing = m_ambiguousCrossings[crossingIdx];

    // check if the position is within the bounding box of the crossing
    if (pos.x >= posTrunc.first - 0.25f && pos.x <= posTrunc.first + 0.25f &&
        pos.y >= posTrunc.second - 0.25f && pos.y <= posTrunc.second + 0.25f) {
      if (crossing.diagonal_kept) {
        addSimilarityEdge(crossing.antidiagonal.first, crossing.antidiagonal.second);
        removeSimilarityEdge(crossing.diagonal.first, crossing.diagonal.second);
      } else {
        addSimilarityEdge(crossing.diagonal.first, crossing.diagonal.second);
        removeSimilarityEdge(crossing.antidiagonal.first, crossing.antidiagonal.second);
      }

      crossing.diagonal_kept = !crossing.diagonal_kept;

      return true;
    }
  }

  return false;
}
