#ifndef __SVG_H__
#define __SVG_H__

#include <fstream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "opencv2/core/types.hpp"

class Svg {
 public:
  Svg(int width, int height) : m_width(width), m_height(height) {}

  void addPolygon(const std::vector<cv::Point2d>& points, const cv::Scalar& color) {
    std::string polygon = "<polygon points=\"";
    for (const auto& point : points) {
      polygon += std::to_string(point.x) + "," + std::to_string(point.y) + " ";
    }
    polygon +=
        "\" fill=\"#" +
        cv::format("%02x%02x%02x%02x", (int)color[0], (int)color[1], (int)color[2], (int)color[3]) +
        "\" />";

    m_elements.push_back(polygon);
  }

  void writeToFile(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
      throw std::runtime_error("Could not open file for writing: " + filename);
    }

    file << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << m_width << "\" height=\""
         << m_height << "\">\n";
    for (const auto& element : m_elements) {
      file << "  " << element << "\n";
    }
    file << "</svg>\n";

    file.close();
  }

 private:
  int m_width;
  int m_height;
  std::vector<std::string> m_elements;
};

#endif  // __SVG_H__
