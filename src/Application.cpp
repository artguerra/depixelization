#include "Application.h"

#include <imgui.h>

void Application::render() {
    ImGui::Begin("Hello, ImGui!");
    ImGui::Text("This is a window!");
    ImGui::End();
}
void Application::loadImage(const cv::Mat& image) {}
void Application::setWindowSize(int width, int height) {}