#include <Bezier.h>
#include <cassert>

cv::Point2d BezierCurve::evaluate(const std::vector<cv::Point2d>& ctrlPts, double t) {
  int n = int(ctrlPts.size());
  cv::Point2d pt(0.0, 0.0);

  for (int i = 0; i < n; ++i) {
    double coeff = binom(n - 1, i) * pow(1 - t, n - 1 - i) * pow(t, i);
    pt.x += coeff * ctrlPts[i].x;
    pt.y += coeff * ctrlPts[i].y;
  }

  return pt;
}

double BezierCurve::calculateError(
    const std::vector<cv::Point2d>& pts, const std::vector<cv::Point2d>& ctrlPts
) {
  double error = 0.0;
  int n = int(pts.size());

  for (int i = 0; i < n; ++i) {
    double t = double(i) / (n - 1);
    cv::Point2d bezier_pt = evaluate(ctrlPts, t);
    error += cv::norm(pts[i] - bezier_pt);
  }

  return error / n;
}

void BezierCurve::fitBezier(
    const std::vector<cv::Point2d>& pts, int d, std::vector<cv::Point2d>& ctrlPts
) {
  assert(d >= 1 && d <= 3 && "Degree must be between 1 and 3");

  int n = int(pts.size());
  assert(n > d && "Number of points must be greater than degree");

  // build A matrix (bernstein basis)
  cv::Mat A(n, d + 1, CV_64F);
  for (int i = 0; i < n; ++i) {
    double t = double(i) / (n - 1), u = 1 - t;
    double t_pow[4] = {1, t, t * t, t * t * t};
    double u_pow[4] = {1, u, u * u, u * u * u};
    for (int j = 0; j <= d; ++j) {
      A.at<double>(i, j) = binom(d, j) * u_pow[d - j] * t_pow[j];
    }
  }

  // build Px, Py
  cv::Mat Px(n, 1, CV_64F), Py(n, 1, CV_64F);
  for (int i = 0; i < n; ++i) {
    Px.at<double>(i, 0) = pts[i].x;
    Py.at<double>(i, 0) = pts[i].y;
  }

  // solve bernstein coefficients
  cv::Mat Cx, Cy;
  cv::solve(A, Px, Cx, cv::DECOMP_SVD);
  cv::solve(A, Py, Cy, cv::DECOMP_SVD);

  // copy to ctrlPts
  ctrlPts.resize(d + 1);
  for (int j = 0; j <= d; ++j) {
    ctrlPts[j] = {Cx.at<double>(j, 0), Cy.at<double>(j, 0)};
  }
}

void BezierCurve::fit(
    const std::vector<cv::Point2d>& pts, std::vector<std::vector<cv::Point2d>>& ctrlPts
) {
  const double errorThreshold = 0.01;

  ctrlPts.clear();

  // try to fit a single curve to all points
  // start trying a straight line, then quadratic bezier, then cubic bezier
  std::vector<cv::Point2d> fitted_bezier;

  for (int d = 1; d <= 3; ++d) {
    fitBezier(pts, d, fitted_bezier);
    double error = calculateError(pts, fitted_bezier);

    // error is acceptable
    if (error <= errorThreshold) {
      ctrlPts.push_back(fitted_bezier);
      return;
    }
  }

  // error too high, subdivide points into two segments
  int midPoint = pts.size() / 2;

  // create left segment
  std::vector<cv::Point2d> left_pts(pts.begin(), pts.begin() + midPoint);
  std::vector<std::vector<cv::Point2d>> left_ctrl;
  fit(left_pts, left_ctrl);

  // create right segment
  std::vector<cv::Point2d> right_pts(pts.begin() + midPoint - 1, pts.end());
  std::vector<std::vector<cv::Point2d>> right_ctrl;
  fit(right_pts, right_ctrl);

  // combine results
  ctrlPts.insert(ctrlPts.end(), left_ctrl.begin(), left_ctrl.end());
  ctrlPts.insert(ctrlPts.end(), right_ctrl.begin(), right_ctrl.end());
}
