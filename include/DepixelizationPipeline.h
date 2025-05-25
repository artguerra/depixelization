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
  glm::vec2 pos;
  FractionalCoord originalPos;
  std::set<int> neighbors;
  enum Type { CORNER, EDGE } type;
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

  // algorithm helper functions
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

  // general helper functions
  float fractionToFloat(const Fractional& frac, float delta) const {
    return static_cast<float>(frac.first) + static_cast<float>(frac.second) * delta;
  }

  // cluster logic
  void colorClusters();
  void findPixelClusters();
  cv::Vec4f findPixelClusterDFS(int, std::set<int>&, std::vector<bool>&, cv::Vec4f);

  // constants
  constexpr static int EDGE_NODES_PER_PIXEL = 3;
};

#endif  // __DEPIXELIZATION_PIPELINE_H__
