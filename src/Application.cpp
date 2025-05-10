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
