#ifndef __CAMERA_H__
#define __CAMERA_H__

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

class Camera {
 public:
  Camera(glm::vec2 position, float zoom) : m_position(position), m_zoom(zoom) {}

  glm::mat4 getOrthoMatrix(float aspectRatio) const {
    float orthoHeight = 1.0f / m_zoom;
    float orthoWidth = orthoHeight * aspectRatio;
    return glm::ortho(
        -orthoWidth + m_position.x, orthoWidth + m_position.x,   // left, right
        -orthoHeight + m_position.y, orthoHeight + m_position.y  // bottom, top
    );
  }

  void setPosition(float x, float y) { m_position = glm::vec2(x, y); }
  void setZoom(float zoom) { m_zoom = zoom; }
  void pan(float dx, float dy) { m_position += glm::vec2(dx, dy); }

  void zoom(float factor) {
    m_zoom *= factor;
    if (m_zoom < 0.1f) m_zoom = 0.1f;    // prevent zooming out too much
    if (m_zoom > 10.0f) m_zoom = 10.0f;  // prevent zooming in too much
  }

  void reset() {
    m_position = glm::vec2(0.0f, 0.0f);
    m_zoom = 1.0f;
  }

 private:
  glm::vec2 m_position;
  float m_zoom;
};

#endif  // __CAMERA_H__
