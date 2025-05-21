#ifndef __APPLICATION_H__
#define __APPLICATION_H__

#include <vector>

#include <opencv2/opencv.hpp>

#include "Canvas.h"

class Application {
 public:
  void render();
  bool loadImage(char* path);

  void computeSimilarityGraph();

  // camera controls
  void setWindowSize(int width, int height) { m_canvas.setViewportSize(width, height); }
  void setMousePos(double x, double y) { m_canvas.setMousePos(x, y); }
  void zoom(float factor) { m_canvas.zoom(factor); }
  void pan(float dx, float dy) { m_canvas.pan(dx, dy); }

 private:
  cv::Mat m_image;
  Canvas m_canvas;

  // control variables
  bool m_isSimilarityGraphVisible = false;

  // application logic & computation
  std::vector<std::vector<int>> m_similarity;

  void renderSimilarityGraph();

  // helper functions
  void removeSimilarityEdge(int idx1, int idx2);

  std::pair<int, int> indexToCoordinate(int index) const;
  int coordinateToIndex(int x, int y) const;
};

#endif  // __APPLICATION_H__
