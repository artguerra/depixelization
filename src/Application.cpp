#include "Application.h"

#include <imgui.h>

void Application::render() {
  // ------------ imgui rendering ------------
  ImGui::Begin("Interactive Depixelization", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
  ImGui::Text("You can use the mouse to pan and zoom.");

  // similarity graph controls
  if (ImGui::CollapsingHeader("Similarity Graph Controls")) {
    ImGui::SliderFloat("Similarity Threshold", &m_colorSimilarityThreshold, 0.0f, 100.0f);
    if (ImGui::IsItemDeactivatedAfterEdit()) {
      m_pipeline.computeSimilarityGraph(m_colorSimilarityThreshold);
      updateSimilarityGraph();

      m_pipeline.computePathGeneration();
      updatePathGraph();
    }
    ImGui::Checkbox("Show similarity graph", &m_isSimilarityGraphVisible);
  }

  // path graph controls
  if (ImGui::CollapsingHeader("Path Graph Controls")) {
    ImGui::Text("Path graph is generated automatically.");
    ImGui::Checkbox("Show path graph", &m_isPathGraphVisible);
  }

  // simulation controls
  if (ImGui::CollapsingHeader("Spring Simulation Controls")) {
    ImGui::Text("Path graph is generated automatically.");
    ImGui::Text("Simulation is run automatically after path graph generation.");

    if (ImGui::Button("Run spring simulation")) {
      m_pipeline.computeSpringSimulation();
      updatePathGraph();
    }
  }

  ImGui::End();

  // ------------ main render ------------
  m_canvas.render();
  if (m_isSimilarityGraphVisible) m_canvas.renderSimilarityGraph();
  if (m_isPathGraphVisible) m_canvas.renderPathGraph();
}

bool Application::loadImage(char* path) {
  if (!m_pipeline.loadImage(path)) {
    return false;
  }

  m_canvas.setTexture(m_pipeline.getImage());

  // compute everything in the beginning
  m_pipeline.computeSimilarityGraph(m_colorSimilarityThreshold);
  m_pipeline.computePathGeneration();
  m_pipeline.computeSpringSimulation();

  updateSimilarityGraph();
  updatePathGraph();

  return true;
}

void Application::updateSimilarityGraph() {
  // get the graph buffers
  std::vector<float> vertices;
  std::vector<unsigned int> indices;
  m_pipeline.getSimilarityGraphBuffers(vertices, indices);

  // send to canvas for rendering
  m_canvas.initializeSimilarityGraphBuffers(vertices, indices);
}

void Application::updatePathGraph() {
  // get the path graph buffers
  std::vector<float> vertices;
  std::vector<unsigned int> indices;
  m_pipeline.getPathGraphBuffers(vertices, indices);

  // send to canvas for rendering
  m_canvas.initializePathGraphBuffers(vertices, indices);
}
