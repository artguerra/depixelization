#include "Application.h"

#include <imgui.h>

#include "opencv2/core.hpp"
#include "opencv2/core/base.hpp"

void Application::render() {
  ImGui::Begin("Interactive Depixelization");
  ImGui::Text("You can use the mouse to pan and zoom.");
  ImGui::Checkbox("Show clustering graph", &m_isSimilarityGraphVisible);
  ImGui::End();

  m_canvas.render();

  if (m_isSimilarityGraphVisible) {
    renderSimilarityGraph();
  }
}

bool Application::loadImage(char* path) {
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

  // load image as a texture and bind the buffers for rendering
  m_canvas.setTexture(m_image);
  return true;
}

std::pair<int, int> Application::indexToCoordinate(int index) const {
  int x = index % m_image.cols;
  int y = index / m_image.cols;

  if (x >= m_image.cols || y >= m_image.rows) return {-10, -10};

  return {x, y};
}

int Application::coordinateToIndex(int x, int y) const { return y * m_image.cols + x; }

void Application::computeSimilarityGraph() {
  int height = m_image.rows;
  int width = m_image.cols;

  m_similarity.clear();
  m_similarity.resize(height * width);
  m_similarity.reserve(height * width);

  // compute similarity graph
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      cv::Vec4b color = m_image.at<cv::Vec4b>(y, x);

      // check neighboring pixel colors and compute color distances
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) continue;  // skip the center pixel

          int neigh_x = x + dx;
          int neigh_y = y + dy;

          if (neigh_x < 0 || neigh_x >= width || neigh_y < 0 || neigh_y >= height) continue;

          cv::Vec4b neighborColor = m_image.at<cv::Vec4b>(neigh_y, neigh_x);
          if (cv::norm(color, neighborColor, cv::NORM_L2) < 50.0f) {
            m_similarity[coordinateToIndex(x, y)].push_back(coordinateToIndex(neigh_x, neigh_y));
          }
        }
      }
    }
  }

  // initialize similarity graph buffers
  std::vector<float> vertices;
  std::vector<unsigned int> indices;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      vertices.push_back((x + 0.5f) / static_cast<float>(width) * 2.0f - 1.0f);   // x-coordinate
      vertices.push_back((y + 0.5f) / static_cast<float>(height) * 2.0f - 1.0f);  // y-coordinate
      vertices.push_back(0.0f);                                                   // z-coordinate

      for (int neighbor : m_similarity[coordinateToIndex(x, y)]) {
        indices.push_back(coordinateToIndex(x, y));
        indices.push_back(neighbor);
      }
    }
  }

  m_canvas.initializeSimilarityGraphBuffers(vertices, indices);
}

void Application::renderSimilarityGraph() { m_canvas.renderSimilarityGraph(); }
