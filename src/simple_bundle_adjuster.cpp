#include "tools.h"

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

// Bundle Adjustment dataset.

class BBAProblem {

public:
  BBAProblem(const char* filename) { LoadFile(filename); }
  // Getters
  const int num_observations() const { return num_observations_; }
  const int num_parameters() const { return num_parameters_; }
  const int num_cameras() const { return num_cameras_; }
  // const std::vector<int>& point_indexes() const { return point_index_; }
  // const std::vector<int>& camera_indexes() const { return camera_index_; }
  const std::vector<double>& observations() const { return observations_; }
  std::vector<double>* parameters() { return &parameters_; }

  void PrintObservations() const {
    std::cout << "Number of observations: " << num_observations_ << std::endl;
    std::cout << "Number of parameters: " << num_parameters_ << std::endl;
    std::cout << "Observations: " << std::endl;
    for (int i = 0; i < num_observations_; i += num_cameras_) {
      std::cout << "Point " << point_index_.at(i) << std::endl;
      for (int j = 0; j < num_cameras_; j++) {
        std::cout << "Camera " << camera_index_.at(i + j) << " ";
        std::cout << observations_.at(2 * i + 2 * j + 0) << " "
                  << observations_.at(2 * i + 2 * j + 1) << std::endl;
      }
    }
  }

  void PrintCameras() const {
    std::cout << "Cameras:" << std::endl;
    for (int i = 0; i < num_cameras_; i++) {
      std::cout << "Cam " << i << std::endl;
      // std::cout << "t: " << parameters_[0] << std::endl;
    }
  }

private:
  int num_cameras_;
  int num_camera_params_ = 11;
  int num_points_;
  int num_observations_;
  int num_parameters_;
  std::vector<int> point_index_;
  std::vector<int> camera_index_;
  std::vector<double> observations_;
  std::vector<double> parameters_;

  // Create BBA problem from file, according to BAL structure
  // http://grail.cs.washington.edu/projects/bal
  bool LoadFile(const char* filename) {

    std::string f_name = filename;
    std::ifstream ifs;
    ifs.open(f_name);
    if (ifs.fail()) {
      throw std::runtime_error("ERROR: unable to open input file");
    }
    ifs >> num_cameras_ >> num_points_ >> num_observations_;

    num_parameters_ = num_camera_params_ * num_cameras_ + 3 * num_points_;

    // Read observations
    int cam_idx, pt_idx;
    double x, y;
    for (int i = 0; i < num_observations_; ++i) {
      ifs >> cam_idx >> pt_idx >> x >> y;
      camera_index_.push_back(cam_idx);
      point_index_.push_back(pt_idx);
      observations_.push_back(x);
      observations_.push_back(y);
    }

    // Read Parameters
    double k;
    for (int i = 0; i < num_parameters_; ++i) {
      ifs >> k;
      parameters_.push_back(k);
    }

    return true;
  }
};

// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
// focal length and 2 for radial distortion. The principal point is not modeled
// (i.e. it is assumed be located at the image center).
struct SnavelyReprojectionError {
  SnavelyReprojectionError(double observed_x, double observed_y)
      : observed_x(observed_x), observed_y(observed_y) {}
  template <typename T>
  bool operator()(const T* const camera, const T* const point,
                  T* residuals) const {
    // camera[0,1,2] are the angle-axis rotation.
    T p[3];
    ceres::AngleAxisRotatePoint(camera, point, p);
    // camera[3,4,5] are the translation.
    p[0] += camera[3];
    p[1] += camera[4];
    p[2] += camera[5];
    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes, whereby
    // the camera coordinate system has a negative z axis.
    T xp = -p[0] / p[2];
    T yp = -p[1] / p[2];
    // Apply second and fourth order radial distortion.
    const T& l1 = camera[9];
    const T& l2 = camera[10];
    T r2 = xp * xp + yp * yp;
    T distortion = 1.0 + r2 * (l1 + l2 * r2);
    // Compute final projected point position.
    const T& focal = camera[6];
    T predicted_x = focal * distortion * xp;
    T predicted_y = focal * distortion * yp;
    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - observed_x;
    residuals[1] = predicted_y - observed_y;
    return true;
  }
  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y) {
    return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 11, 3>(
        new SnavelyReprojectionError(observed_x, observed_y)));
  }
  double observed_x;
  double observed_y;
};

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  if (argc != 2) {
    std::cerr << "usage: simple_bundle_adjuster <bal_problem>\n";
    return 1;
  }

  BBAProblem bba_problem(argv[1]);

  // bba_problem.PrintObservations();

  const std::vector<double> observations = bba_problem.observations();
  std::vector<double>* params = bba_problem.parameters();

  params->at(0) = 0;

  // Create residuals for each observation in the bundle
  // adjustment problem. The parameters for cameras and points
  // are added automatically.
  ceres::Problem problem;
  for (int i = 0; i < bba_problem.num_observations(); ++i) {
    // Each Residual block takes a point and a camera as input and outputs a
    // dimensional residual. Internally, the cost function stores the observed
    // image location and compares the reprojection against the observation.
    ceres::CostFunction* cost_function = SnavelyReprojectionError::Create(
        observations[2 * i + 0], observations[2 * i + 1]);
    // problem.AddResidualBlock(cost_function, nullptr /* squared loss */,
    //                          bba_problem.mutable_camera_for_observation(i),
    //                          bba_problem.mutable_point_for_observation(i));
  }
  // Make Ceres automatically detect the bundle structure. Note that the
  // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
  // for standard bundle adjustment problems.

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.FullReport() << "\n";
  return 0;
}