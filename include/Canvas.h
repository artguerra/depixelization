#ifndef __CANVAS_H__
#define __CANVAS_H__

#include <glm/glm.hpp>
#include <opencv2/core/mat.hpp>

#include "Camera.h"
#include "Shader.h"

class Canvas {
 public:
  Canvas() : m_shader(VERT_SHADER_PATH, FRAG_SHADER_PATH) {}

  void render();
  void setTexture(cv::Mat& image);

  // camera controls
  void zoom(float factor) { m_camera.zoom(factor); }
  void pan(float dx, float dy) { m_camera.pan(dx, dy); }
  void reset() { m_camera.reset(); }

  void setViewportSize(int width, int height) {
    m_aspectRatio = static_cast<float>(width) / static_cast<float>(height);
  }

 private:
  Camera m_camera{glm::vec2(0.0f, 0.0f), 0.8f};
  Shader m_shader;

  // dimensions
  int m_imgHeight{}, m_imgWidth{};
  float m_aspectRatio{};

  // opengl state
  unsigned int m_textureID{};
  unsigned int m_VAO{};
  unsigned int m_VBO{};
  unsigned int m_EBO{};

  void initBuffers();

  static constexpr char VERT_SHADER_PATH[] = "shaders/canvas.vert";
  static constexpr char FRAG_SHADER_PATH[] = "shaders/canvas.frag";
};

#endif  // __CANVAS_H__
