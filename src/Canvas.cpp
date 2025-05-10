#include "Canvas.h"

#include <glad/glad.h>

const float VERTICES[] = {
    // positions        // texture coords
    -1.0f, -1.0f, 0.0f, 0.0f, 0.0f,  // bottom left
    1.0f,  -1.0f, 0.0f, 1.0f, 0.0f,  // bottom right
    1.0f,  1.0f,  0.0f, 1.0f, 1.0f,  // top right
    -1.0f, 1.0f,  0.0f, 0.0f, 1.0f,  // top left
};

const unsigned int INDICES[] = {0, 1, 2, 2, 3, 0};

void Canvas::render() {
  m_shader.use();

  // set texture
  glActiveTexture(GL_TEXTURE0);
  m_shader.setInt("tex", 0);
  m_shader.setInt("imgWidth", m_imgWidth);
  m_shader.setInt("imgHeight", m_imgHeight);
  m_shader.setMat4("projection", m_camera.getOrthoMatrix(m_aspectRatio));
  m_shader.setVec2("mousePos", getPointedPixel());

  glBindTexture(GL_TEXTURE_2D, m_textureID);

  // draw
  glBindVertexArray(m_VAO);
  glDrawElements(GL_TRIANGLES, sizeof(INDICES), GL_UNSIGNED_INT, 0);

  // unbind
  glBindVertexArray(0);
  glBindTexture(GL_TEXTURE_2D, 0);
}

void Canvas::setTexture(cv::Mat& image) {
  m_imgHeight = image.rows;
  m_imgWidth = image.cols;

  glGenTextures(1, &m_textureID);
  glBindTexture(GL_TEXTURE_2D, m_textureID);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

  glTexImage2D(
      GL_TEXTURE_2D, 0, GL_RGB, image.cols, image.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, image.data
  );

  initBuffers();
}

void Canvas::initBuffers() {
  glGenVertexArrays(1, &m_VAO);
  glBindVertexArray(m_VAO);

  glGenBuffers(1, &m_VBO);
  glBindBuffer(GL_ARRAY_BUFFER, m_VBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(VERTICES), VERTICES, GL_STATIC_DRAW);

  glGenBuffers(1, &m_EBO);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_EBO);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(INDICES), INDICES, GL_STATIC_DRAW);

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
  glEnableVertexAttribArray(0);
  glEnableVertexAttribArray(1);
}

// get mouse coordinates in pixel space
glm::vec2 Canvas::getPointedPixel() {
  float ndcX = 2.0f * (m_mouseX / m_viewportWidth) - 1.0f;
  float ndcY = 1.0f - 2.0f * (m_mouseY / m_viewportHeight);

  glm::mat4 invProj = glm::inverse(m_camera.getOrthoMatrix(m_aspectRatio));
  glm::vec4 worldPos = invProj * glm::vec4(ndcX, ndcY, 0.0f, 1.0f);

  // map to image pixel coordinates (0 to width, 0 to height)
  float imgX = (worldPos.x * 0.5f + 0.5f) * m_imgWidth;
  float imgY = (worldPos.y * 0.5f + 0.5f) * m_imgHeight;

  return glm::vec2(imgX, imgY);
}

