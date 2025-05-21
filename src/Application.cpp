#include "Application.h"

#include <imgui.h>

void Application::render() {
  // ------------ imgui rendering ------------
  ImGui::Begin("Interactive Depixelization", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
  ImGui::Text("You can use the mouse to pan and zoom.");

  if (ImGui::CollapsingHeader("Similarity Graph Controls")) {
    ImGui::SliderFloat("Similarity Threshold", &m_colorSimilarityThreshold, 0.0f, 100.0f);
    if (ImGui::IsItemDeactivatedAfterEdit()) {
      updateSimilarityGraph();
    }
    ImGui::Checkbox("Show similarity graph", &m_isSimilarityGraphVisible);
  }

  ImGui::End();

  // ------------ main render ------------
  m_canvas.render();

  if (m_isSimilarityGraphVisible) {
    m_canvas.renderSimilarityGraph();
  }
}

bool Application::loadImage(char* path) {
  if (!m_pipeline.loadImage(path)) {
    return false;
  }

  m_canvas.setTexture(m_pipeline.getImage());
  updateSimilarityGraph();

  return true;
}

void Application::updateSimilarityGraph() {
  // compute the similarity graph
  m_pipeline.computeSimilarityGraph(m_colorSimilarityThreshold);

  // get the graph buffers
  std::vector<float> vertices;
  std::vector<unsigned int> indices;
  m_pipeline.getSimilarityGraphBuffers(vertices, indices);

  // send to canvas for rendering
  m_canvas.initializeSimilarityGraphBuffers(vertices, indices);
}
