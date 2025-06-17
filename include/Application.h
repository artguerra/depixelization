#ifndef __APPLICATION_H__
#define __APPLICATION_H__

#include "Canvas.h"
#include "DepixelizationPipeline.h"

class Application {
 public:
  void render();
  bool loadImage(char* path);

  // Camera controls
  void setWindowSize(int width, int height) { m_canvas.setViewportSize(width, height); }
  void setMousePos(double x, double y) { m_canvas.setMousePos(x, y); }
  void zoom(float factor) { m_canvas.zoom(factor); }
  void pan(float dx, float dy) { m_canvas.pan(dx, dy); }
  void resetCamera() { m_canvas.reset(); }

  // Mouse interaction
  void handleMouseClick(double x, double y);

 private:
  // core components
  DepixelizationPipeline m_pipeline;
  Canvas m_canvas;

  // control variables
  bool m_isSimilarityGraphVisible = false;
  bool m_isPathGraphVisible = false;
  bool m_isAmbiguousCrossingsVisible = false;
  float m_colorSimilarityThreshold = 50.0f;

  // rendering functions
  void updateSimilarityGraph();
  void updatePathGraph();
};

#endif  // __APPLICATION_H__
