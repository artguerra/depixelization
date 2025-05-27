#ifndef __DEPIXELIZATION_PIPELINE_H__
#define __DEPIXELIZATION_PIPELINE_H__

#include <set>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <opencv2/opencv.hpp>

using Fractional = std::pair<int, int>;
using FractionalCoord = std::pair<Fractional, Fractional>;

struct PixelCluster {
  std::set<int> pixels;
  cv::Vec4b avgColor;
};

struct PathGraphNode {
  FractionalCoord originalPos;
  glm::vec2 pos;
  std::set<int> neighbors;

  float Ko{0.0f};  // spring constant for original position force
  float Kn{0.5f};  // spring constant for neighbor forces

  enum Type { CORNER, EDGE } type;

  void resetOriginSpringStiffness(float stiffnessEdge, float stiffnessCorner) {
    Ko = std::max(
        0.0f, stiffnessEdge + (stiffnessCorner - stiffnessEdge) *
                                  std::abs((2.0f * neighbors.size()) / 3.0f - 1.0f)
    );
  }
};

class DepixelizationPipeline {
 public:
  DepixelizationPipeline() = default;

  // image loading and retrieving
  bool loadImage(char* path);
  const cv::Mat& getImage() const { return m_image; }

  // main processing functions
  void computeSimilarityGraph(float similarityThreshold);
  void computePathGeneration();
  void computeSpringSimulation();

  // coordinate conversion
  std::pair<int, int> indexToCoordinate(int index) const;
  int coordinateToIndex(int x, int y) const;

  // generate rendering data
  void getSimilarityGraphBuffers(std::vector<float>& vertices, std::vector<unsigned int>& indices)
      const;
  void getPathGraphBuffers(std::vector<float>& vertices, std::vector<unsigned int>& indices) const;

 private:
  cv::Mat m_image;
  cv::Mat m_result;

  // algorithm structures
  std::vector<std::vector<int>> m_similarity;
  std::vector<PixelCluster> m_clusters;
  std::vector<PathGraphNode> m_pathGraph;
  std::vector<glm::vec2> m_nodeForces;

  // ----------------------- algorithm helper functions -----------------------
  bool hasSimilarityEdge(int idx1, int idx2) const;
  void removeSimilarityEdge(int idx1, int idx2);

  int createPathNode(FractionalCoord pos, PathGraphNode::Type type);
  void addPathNodeNeighbor(int nodeIdx, int neighborIdx);
  void removePathNodeNeighbor(int nodeIdx, int neighborIdx);
  int getOrCreatePathNode(
      std::map<FractionalCoord, int>& path_node_map, FractionalCoord pos, PathGraphNode::Type type
  );
  void createBoundaryNodes(
      std::map<FractionalCoord, int>& path_node_map, int x, int y, bool vertical
  );

  // cluster logic
  void colorClusters();
  void findPixelClusters();
  cv::Vec4f findPixelClusterDFS(int, std::set<int>&, std::vector<bool>&, cv::Vec4f);

  // simulation
  float calculateNodeForces();

  // general helper functions
  float fractionToFloat(const Fractional& frac, float delta = 1.0f / (EDGE_NODES_PER_PIXEL + 1))
      const {
    return static_cast<float>(frac.first) + static_cast<float>(frac.second) * delta;
  }

  // constants
  constexpr static int EDGE_NODES_PER_PIXEL = 3;
  constexpr static float STOPPING_THRESHOLD = 0.02f;
  constexpr static float MAX_STEP_SIZE = 0.1f;
  constexpr static float STIFFNESS_CORNER = 0.2f;
  constexpr static float STIFFNESS_EDGE = -0.1f;
};

#endif  // __DEPIXELIZATION_PIPELINE_H__
