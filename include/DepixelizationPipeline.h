#ifndef __DEPIXELIZATION_PIPELINE_H__
#define __DEPIXELIZATION_PIPELINE_H__

#include <set>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <opencv2/opencv.hpp>

// constants
constexpr int EDGE_NODES_PER_PIXEL{3};
constexpr float STOPPING_THRESHOLD{0.03f};
constexpr float MAX_STEP_SIZE{0.1f};
constexpr float STIFFNESS_CORNER{0.2f};
constexpr float STIFFNESS_EDGE{-0.1f};

using Fractional = std::pair<int, int>;
using FractionalCoord = std::pair<Fractional, Fractional>;

// general helper functions
static float fractionToFloat(
    const Fractional& frac, float delta = 1.0f / (EDGE_NODES_PER_PIXEL + 1)
) {
  return static_cast<float>(frac.first) + static_cast<float>(frac.second) * delta;
}

struct PixelCluster {
  std::set<int> pixels;
  std::set<int> boundaryNodes;
  std::vector<int> externalBoundary;
  cv::Vec4b avgColor;

  float area{0.0f};
  float initialArea{0.0f};
  glm::vec2 centroid{0.0f, 0.0f};

  PixelCluster(const std::set<int>& pixels, const cv::Vec4b& avgColor)
      : pixels{pixels}, avgColor{avgColor} {}
};

struct PathGraphNode {
  FractionalCoord originalPos;
  glm::vec2 pos;
  std::set<int> neighbors;
  std::set<int> clusters;

  float Ko{0.0f};  // spring constant for original position force
  float Kn{0.9f};  // spring constant for neighbor forces

  enum Type { CORNER, EDGE } type;

  PathGraphNode(FractionalCoord pos, Type type)
      : originalPos{pos},
        pos{fractionToFloat(pos.first), fractionToFloat(pos.second)},
        type{type} {}

  void resetOriginSpringStiffness() {
    Ko = std::max(
        0.0f, STIFFNESS_EDGE + (STIFFNESS_CORNER - STIFFNESS_EDGE) *
                                   std::abs((2.0f * neighbors.size()) / 3.0f - 1.0f)
    );
  }
};

struct AmbiguousCrossing {
  std::pair<int, int> diagonal;
  std::pair<int, int> antidiagonal;
  bool diagonal_kept;
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

  // exporting results
  void exportSvg(const std::string& filename) const;

  // coordinate conversion
  std::pair<int, int> indexToCoordinate(int index) const;
  int coordinateToIndex(int x, int y) const;

  // user interaction
  bool checkAmbiguousCrossingClick(const glm::vec2& pos);

  // generate rendering data
  void getSimilarityGraphBuffers(
      std::vector<float>& vertices, std::vector<unsigned int>& indices,
      std::vector<float>& ambiguousCrossings, std::vector<unsigned int>& ambiguousCrossingsIndices
  ) const;
  void getPathGraphBuffers(std::vector<float>& vertices, std::vector<unsigned int>& indices) const;

  std::vector<PixelCluster>& getClusters() { return m_clusters; }

 private:
  cv::Mat m_image;
  cv::Mat m_result;

  // algorithm structures
  std::vector<std::vector<int>> m_similarity;
  std::vector<AmbiguousCrossing> m_ambiguousCrossings;
  std::map<std::pair<int, int>, int>
      m_ambiguousCrossingsPositions;  // maps pixel position to ambiguous crossing index

  std::vector<PixelCluster> m_clusters;
  std::vector<PathGraphNode> m_pathGraph;
  std::vector<glm::vec2> m_nodeForces;

  // helper structures
  std::vector<int> m_pixelToClusterMap;  // stores index of cluster for each pixel

  // ----------------------- algorithm helper functions -----------------------
  bool hasSimilarityEdge(int idx1, int idx2) const;
  void addSimilarityEdge(int idx1, int idx2);
  void removeSimilarityEdge(int idx1, int idx2);

  int createPathNode(FractionalCoord pos, PathGraphNode::Type type);
  void addPathNodeNeighbor(int nodeIdx, int neighborIdx);
  void removePathNodeNeighbor(int nodeIdx, int neighborIdx);
  int getOrCreatePathNode(
      std::map<FractionalCoord, int>& path_node_map, FractionalCoord pos, PathGraphNode::Type type,
      int clusterIdx
  );
  void createBoundaryNodes(
      std::map<FractionalCoord, int>& path_node_map, int x, int y, bool vertical, int clusterIdx
  );

  // cluster logic
  void colorClusters();
  void findPixelClusters();
  cv::Vec4f findPixelClusterDFS(int, std::set<int>&, std::vector<bool>&, cv::Vec4f);
  void findExternalBoundary(int clusterIdx);
  void updateClusterArea(int clusterIdx, bool initial = false);

  // simulation
  float calculateNodeForces();
};

#endif  // __DEPIXELIZATION_PIPELINE_H__
