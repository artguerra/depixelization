#ifndef __DEPIXELIZATION_PIPELINE_H__
#define __DEPIXELIZATION_PIPELINE_H__

#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>

class DepixelizationPipeline {
 public:
  DepixelizationPipeline() = default;

  // image loading and retrieving
  bool loadImage(char* path);
  const cv::Mat& getImage() const { return m_image; }

  // main processing functions
  void computeSimilarityGraph(float similarityThreshold);

  // coordinate conversion
  std::pair<int, int> indexToCoordinate(int index) const;
  int coordinateToIndex(int x, int y) const;

  // generate rendering data
  void getSimilarityGraphBuffers(std::vector<float>& vertices, std::vector<unsigned int>& indices)
      const;

 private:
  cv::Mat m_image;
  std::vector<std::vector<int>> m_similarity;

  // helper functions
  bool hasSimilarityEdge(int idx1, int idx2) const;
  void removeSimilarityEdge(int idx1, int idx2);
};

#endif  // __DEPIXELIZATION_PIPELINE_H__
