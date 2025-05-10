#ifndef __CANVAS_H__
#define __CANVAS_H__

#include <opencv2/core/mat.hpp>

#include "Shader.h"

class Canvas {
 public:
  Canvas() : m_shader(VERT_SHADER_PATH, FRAG_SHADER_PATH) {}

  void render();
  void setTexture(cv::Mat& image);
  void zoom(float factor);
  void pan(float dx, float dy);

 private:
  Shader m_shader;
  unsigned int m_textureID;
  unsigned int m_VAO;
  unsigned int m_VBO;
  unsigned int m_EBO;

  void initBuffers();

  static constexpr char VERT_SHADER_PATH[] = "shaders/canvas.vert";
  static constexpr char FRAG_SHADER_PATH[] = "shaders/canvas.frag";
};

#endif  // __CANVAS_H__
