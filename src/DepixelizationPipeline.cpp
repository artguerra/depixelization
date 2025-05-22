#include "DepixelizationPipeline.h"

#include <map>

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
  cv::flip(m_image, m_image, 0);

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

                  if (min_cardinality_diag <= min_cardinality_antidiag) {
                    can_add_edge = true;
                    removeSimilarityEdge(antidiagonal.first, antidiagonal.second);
                  } else {
                    can_add_edge = false;
                    removeSimilarityEdge(cur_idx, neigh_idx);
                  }
                }
              }
            }

            if (can_add_edge && !hasSimilarityEdge(cur_idx, neigh_idx)) {
              m_similarity[cur_idx].push_back(neigh_idx);
              m_similarity[neigh_idx].push_back(cur_idx);
            }
          }
        }
      }
    }
  }
}

void DepixelizationPipeline::getSimilarityGraphBuffers(
    std::vector<float>& vertices, std::vector<unsigned int>& indices
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
      vertices.push_back((y + 0.5f) / static_cast<float>(height) * 2.0f - 1.0f);  // y-coordinate

      // add indices for edges in only one direction to avoid duplicates
      for (int neighbor : m_similarity[idx]) {
        if (idx > neighbor) continue;  // avoid duplicate edges
        indices.push_back(idx);
        indices.push_back(neighbor);
      }
    }
  }
}

void DepixelizationPipeline::findPixelClusters() {
  m_clusters.clear();

  int num_pixels = m_image.rows * m_image.cols;

  std::vector<bool> visited(num_pixels, false);
  for (int i = 0; i < num_pixels; ++i) {
    if (!visited[i]) {
      std::set<int> cluster;
      cv::Vec4f color_sum = findPixelClusterDFS(i, cluster, visited, cv::Vec4f(0.0f));

      m_clusters.push_back({cluster, color_sum / static_cast<float>(cluster.size())});
    }
  }
}

cv::Vec4f DepixelizationPipeline::findPixelClusterDFS(
    int idx, std::set<int>& cluster, std::vector<bool>& visited, cv::Vec4f sum
) {
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
      sum = findPixelClusterDFS(neigh, cluster, visited, sum);
    }
  }

  return sum;
}

void DepixelizationPipeline::colorClusters() {
  m_result = cv::Mat::zeros(m_image.size(), m_image.type());

  for (const auto& cluster : m_clusters) {
    for (int pixel : cluster.pixels) {
      auto [x, y] = indexToCoordinate(pixel);
      m_result.at<cv::Vec4b>(y, x) = cluster.avgColor;
    }
  }
}

int DepixelizationPipeline::createPathNode(ComparablePos pos, PathGraphNode::Type type) {
  glm::vec2 position = {pos.first, pos.second};
  m_pathGraph.push_back({position, position, {}, type});
  return static_cast<int>(m_pathGraph.size()) - 1;
}

void DepixelizationPipeline::addPathNodeNeighbor(int nodeIdx, int neighborIdx) {
  m_pathGraph[nodeIdx].neighbors.insert(neighborIdx);
  m_pathGraph[neighborIdx].neighbors.insert(nodeIdx);
}

int DepixelizationPipeline::getOrCreatePathNode(
    std::map<ComparablePos, int>& path_node_map, ComparablePos pos, PathGraphNode::Type type
) {
  if (path_node_map.find(pos) == path_node_map.end())
    path_node_map[pos] = createPathNode(pos, type);

  return path_node_map[pos];
}

void DepixelizationPipeline::createBoundaryNodes(
    std::map<ComparablePos, int>& path_node_map, int x, int y, bool vertical
) {
  // corner nodes
  int corner1 = getOrCreatePathNode(path_node_map, {x, y}, PathGraphNode::CORNER);
  int corner2 = getOrCreatePathNode(
      path_node_map, {x + (vertical ? 0 : 1), y + (vertical ? 1 : 0)}, PathGraphNode::CORNER
  );

  // edge nodes
  int edges[EDGE_NODES_PER_PIXEL];
  float delta = 1.0f / (EDGE_NODES_PER_PIXEL + 1);
  for (int i = 1; i <= EDGE_NODES_PER_PIXEL; ++i) {
    ComparablePos pos = {x + (vertical ? 0 : i * delta), y + (vertical ? i * delta : 0)};
    edges[i - 1] = getOrCreatePathNode(path_node_map, pos, PathGraphNode::EDGE);
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

  std::map<ComparablePos, int> path_node_map;
  for (auto& cluster : m_clusters) {
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
      if (is_left_boundary) createBoundaryNodes(path_node_map, x, y, true);
      if (is_right_boundary) createBoundaryNodes(path_node_map, x + 1, y, true);
      if (is_top_boundary) createBoundaryNodes(path_node_map, x, y, false);
      if (is_bottom_boundary) createBoundaryNodes(path_node_map, x, y + 1, false);
    }
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
    vertices.push_back(node.pos.y / static_cast<float>(height) * 2.0f - 1.0f);
  }

  for (int i = 0; i < m_pathGraph.size(); ++i) {
    for (int neighbor : m_pathGraph[i].neighbors) {
      if (i > neighbor) continue;
      indices.push_back(i);
      indices.push_back(neighbor);
    }
  }
}
