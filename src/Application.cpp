#include "Application.h"

#include <imgui.h>

#include "opencv2/core.hpp"

void Application::render() {
  ImGui::Begin("Hello, ImGui!");
  ImGui::Text("This is a window!");
  ImGui::End();

  m_canvas.render();
}

bool Application::loadImage(char* path) {
  m_image.release();
  m_image = cv::imread(path, cv::IMREAD_COLOR);

  cv::cvtColor(m_image, m_image, cv::COLOR_BGR2RGB);  // convert BGR to RGB
  cv::flip(m_image, m_image, 0);                      // flip the image vertically

  if (!m_image.data) {
    return false;
  }

  // load image as a texture and bind the buffers for rendering
  m_canvas.setTexture(m_image);

  return true;
}
