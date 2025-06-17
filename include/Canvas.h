#ifndef __CANVAS_H__
#define __CANVAS_H__

#include <vector>

#include <glm/glm.hpp>
#include <opencv2/core/mat.hpp>

#include "Camera.h"
#include "Shader.h"

class Canvas {
 public:
  Canvas()
      : m_shader(VERT_SHADER_PATH, FRAG_SHADER_PATH),
        m_lineShader(LINE_VERT_SHADER_PATH, LINE_FRAG_SHADER_PATH) {}

  void render();
  void renderSimilarityGraph();
  void renderAmbiguousEdges();
  void renderPathGraph();

  void setTexture(const cv::Mat& image);
  void initializeSimilarityGraphBuffers(
      const std::vector<float>& vertices, const std::vector<unsigned int>& indices,
      const std::vector<float>& ambiguousCrossings,
      const std::vector<unsigned int>& ambiguousCrossingsIndices
  );
  void initializePathGraphBuffers(
      const std::vector<float>& vertices, const std::vector<unsigned int>& indices
  );

  // helper functions
  glm::vec2 getPointedPixel(double x, double y);

  // camera controls
  void zoom(float factor) { m_camera.zoom(factor); }
  void pan(float dx, float dy) { m_camera.pan(dx, dy); }
  void reset() { m_camera.reset(); }

  // viewport calculations
  void setViewportSize(int width, int height) {
    m_viewportWidth = width;
    m_viewportHeight = height;
    m_aspectRatio = static_cast<float>(width) / static_cast<float>(height);
  }

  void setMousePos(double x, double y) {
    m_mouseX = x;
    m_mouseY = y;
  }

 private:
  Camera m_camera{glm::vec2(0.0f, 0.0f), 0.8f};
  Shader m_shader, m_lineShader;

  // dimensions
  int m_imgHeight{}, m_imgWidth{};
  float m_aspectRatio{};

  // viewport
  int m_viewportWidth{}, m_viewportHeight{};
  double m_mouseX{}, m_mouseY{};

  // opengl state
  unsigned int m_textureID{};
  unsigned int m_VAO{};
  unsigned int m_VBO{};
  unsigned int m_EBO{};

  // similarity graph opengl state
  unsigned int m_similarityVAO{};
  unsigned int m_similarityVBO{};
  unsigned int m_similarityEBO{};
  int m_similarityEBOCount{};

  unsigned int m_ambiguousCrossingsVAO{};
  unsigned int m_ambiguousCrossingsVBO{};
  unsigned int m_ambiguousCrossingsEBO{};
  int m_ambiguousCrossingsCount{};

  // path graph opengl state
  unsigned int m_pathGraphVAO{};
  unsigned int m_pathGraphVBO{};
  unsigned int m_pathGraphEBO{};
  int m_pathGraphVAOCount{};
  int m_pathGraphEBOCount{};

  void initBuffers();

  static constexpr char VERT_SHADER_PATH[] = "shaders/canvas.vert";
  static constexpr char FRAG_SHADER_PATH[] = "shaders/canvas.frag";
  static constexpr char LINE_VERT_SHADER_PATH[] = "shaders/line.vert";
  static constexpr char LINE_FRAG_SHADER_PATH[] = "shaders/line.frag";
};

#endif  // __CANVAS_H__
