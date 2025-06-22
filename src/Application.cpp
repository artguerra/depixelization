#include "Application.h"

#include <imgui.h>

void Application::render() {
  // ------------ imgui rendering ------------
  ImGui::Begin("Interactive Depixelization", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
  ImGui::Text("Use the right mouse button to pan and the scroll to zoom.");

  // similarity graph controls
  if (ImGui::CollapsingHeader("Similarity Graph Controls")) {
    ImGui::SliderFloat("Similarity Threshold", &m_colorSimilarityThreshold, 0.0f, 100.0f);
    if (ImGui::IsItemDeactivatedAfterEdit()) {
      m_pipeline.computeSimilarityGraph(m_colorSimilarityThreshold);
      updateSimilarityGraph();

      m_pipeline.computePathGeneration();
      m_pipeline.computeSpringSimulation();
      updatePathGraph();
    }
    ImGui::Checkbox("Show similarity graph", &m_isSimilarityGraphVisible);
    ImGui::Checkbox("Show ambiguous crossings", &m_isAmbiguousCrossingsVisible);
  }

  // path graph controls
  if (ImGui::CollapsingHeader("Path Graph Controls")) {
    ImGui::Text("Path graph is generated automatically.");
    ImGui::Checkbox("Show path graph", &m_isPathGraphVisible);
  }

  // interaction controls (simulation tweaks)
  if (ImGui::CollapsingHeader("Interaction Controls")) {
    ImGui::Text("Use the left mouse button to edit.");
    ImGui::Text("Press 'R' to reset the camera.");

    if (ImGui::Button("Edit ambiguous crossings")) {
      m_isSimilarityGraphVisible = true;
      m_isAmbiguousCrossingsVisible = true;
      m_isPathGraphVisible = false;
    }
  }

  ImGui::End();

  // ------------ main render ------------
  m_canvas.render();
  if (m_isSimilarityGraphVisible) m_canvas.renderSimilarityGraph();
  if (m_isAmbiguousCrossingsVisible) m_canvas.renderAmbiguousEdges();
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
  std::vector<float> vertices, ambiguousCrossings;
  std::vector<unsigned int> indices, ambiguousCrossingsIndices;
  m_pipeline.getSimilarityGraphBuffers(
      vertices, indices, ambiguousCrossings, ambiguousCrossingsIndices
  );

  // send to canvas for rendering
  m_canvas.initializeSimilarityGraphBuffers(
      vertices, indices, ambiguousCrossings, ambiguousCrossingsIndices
  );
}

void Application::updatePathGraph() {
  // get the path graph buffers
  std::vector<float> vertices;
  std::vector<unsigned int> indices;
  m_pipeline.getPathGraphBuffers(vertices, indices);

  // send to canvas for rendering
  m_canvas.initializePathGraphBuffers(vertices, indices);

  m_pipeline.exportSvg("output.svg");
}

void Application::handleMouseClick(double x, double y) {
  if (m_isAmbiguousCrossingsVisible) {
    if (m_pipeline.checkAmbiguousCrossingClick(m_canvas.getPointedPixel(x, y))) {
      m_pipeline.computePathGeneration();
      m_pipeline.computeSpringSimulation();

      updateSimilarityGraph();
      updatePathGraph();
    }
  }
}
