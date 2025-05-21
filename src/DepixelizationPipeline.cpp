#include "DepixelizationPipeline.h"

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
