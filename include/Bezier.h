#ifndef __BEZIER_H__
#define __BEZIER_H__

#include <vector>

#include <opencv2/core.hpp>

#include "opencv2/core/types.hpp"

class BezierCurve {
 public:
  void fit(const std::vector<cv::Point2d>& pts, std::vector<std::vector<cv::Point2d>>& ctrlPts);

  void fitBezier(const std::vector<cv::Point2d>& pts, int d, std::vector<cv::Point2d>& ctrlPts);

 private:
  inline double binom(int n, int k) const {
    static double C[4][4] = {{1, 0, 0, 0}, {1, 1, 0, 0}, {1, 2, 1, 0}, {1, 3, 3, 1}};
    return C[n][k];
  }

  cv::Point2d evaluate(const std::vector<cv::Point2d>& ctrlPts, double t) const;

  double calculateError(
      const std::vector<cv::Point2d>& pts, const std::vector<cv::Point2d>& ctrlPts
  ) const;
};

#endif  // __BEZIER_H__
