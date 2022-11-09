#include "tools.h"

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

const int num_sensor_params = 13;
const int num_camera_params = 9;
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
  std::vector<double>& parameters() { return parameters_; }

  double* mutable_camera_by_observation(int i) {
    /* Returns the pointer to the first parameter of the camera i */
    return &parameters_.at(camera_index_.at(i) * num_camera_params_);
  }
  double* mutable_sensor_by_observation(int i) {
    /* Returns the pointer to the first parameter of the camera i */
    return &parameters_.at(camera_index_.at(i) * num_camera_params_ + 6);
  }

  double* mutable_point_by_observation(int i) {
    /* Returns the pointer to the X coordinate of the point i */
    return &parameters_.at(num_camera_params_ * num_cameras_ +
                           point_index_.at(i) * 3);
  }

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

  // void PrintCameras() const {
  //   std::cout << "Cameras:" << std::endl;
  //   for (int i = 0; i < num_cameras_; i++) {
  //     std::cout << "Cam " << i << std::endl;
  //     // std::cout << "t: " << parameters_[0] << std::endl;
  //   }
  // }

private:
  int num_cameras_;
  int num_sensor_params_ = num_sensor_params;
  int num_camera_params_ = num_sensor_params_ + 6;
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
    for (int i = 0; i < num_observations_; ++i) {
      int cam_idx, pt_idx;
      double x, y;
      ifs >> cam_idx >> pt_idx >> x >> y;
      camera_index_.push_back(cam_idx);
      point_index_.push_back(pt_idx);
      observations_.push_back(x);
      observations_.push_back(y);
    }

    // Read Parameters
    for (int i = 0; i < num_parameters_; ++i) {
      double x;
      ifs >> x;
      parameters_.push_back(x);
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
    const T& l1 = camera[7];
    const T& l2 = camera[8];
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
    return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
        new SnavelyReprojectionError(observed_x, observed_y)));
  }
  double observed_x;
  double observed_y;
};

struct CollinearityError {
  CollinearityError(double observed_x, double observed_y)
      : observed_x(observed_x), observed_y(observed_y) {}

  template <typename T>
  bool operator()(const T* const sensor, const T* const camera,
                  const T* const point, T* residuals) const {
    // camera[0,1,2] are the angle-axis rotation (rodriguez vector).
    // camera[3,4,5] are the translation.
    T p[3];
    T pt[3];
    pt[0] = point[0] - camera[3];
    pt[1] = point[1] - camera[4];
    pt[2] = point[2] - camera[5];
    ceres::AngleAxisRotatePoint(camera, pt, p);

    // sensor[0 = w,1 = h,2 = F,3 = Cx,4 = Cy,
    //        5 = K1,6 = K2,7 = K3,8 = K4,
    //        9 = P1, 10 = P2, 11 = B1, 12 = B2]

    T x = -p[0] / p[2];
    T y = p[1] / p[2];
    T r2 = x * x + y * y;
    T rad = (T(1.0) + sensor[5] * r2 + sensor[6] * r2 * r2 +
             sensor[7] * r2 * r2 * r2 + sensor[8] * r2 * r2 * r2 * r2);
    T xp = x * rad + sensor[9] * (r2 + T(2.0) * x * x) +
           T(2.0) * sensor[10] * x * y;
    T yp = y * rad + sensor[10] * (r2 + T(2.0) * y * y) +
           T(2.0) * sensor[9] * x * y;

    T u = sensor[0] * T(0.5) + sensor[3] + xp * sensor[2] + xp * sensor[11] +
          yp * sensor[12];
    T v = sensor[1] * T(0.5) + sensor[4] + yp * sensor[2];

    // The error is the difference between the predicted and observed position.
    residuals[0] = u - T(observed_x);
    residuals[1] = v - T(observed_y);

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* CreateAutoDiff(const double observed_x,
                                             const double observed_y) {
    return (new ceres::AutoDiffCostFunction<CollinearityError, 2, 13, 6, 3>(
        new CollinearityError(observed_x, observed_y)));
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

  // double* sens = bba_problem.mutable_sensor_by_observation(0);
  // for (int i = 0; i < 5; i++) {
  //   std::cout << sens[i] << std::endl;
  // }

  // Create residuals for each observation in the bundle
  // adjustment problem. The parameters for cameras and points
  // are added automatically.
  const std::vector<double> observations = bba_problem.observations();
  ceres::Problem problem;
  for (int i = 0; i < bba_problem.num_observations(); ++i) {
    // Each Residual block takes a point and a camera as input and outputs a
    // dimensional residual. Internally, the cost function stores the observed
    // image location and compares the reprojection against the observation.
    // ceres::CostFunction* cost_function = SnavelyReprojectionError::Create(
    //     observations[2 * i + 0], observations[2 * i + 1]);
    // problem.AddResidualBlock(cost_function, nullptr /* squared loss */,
    //                          bba_problem.mutable_camera_by_observation(i),
    //                          bba_problem.mutable_point_by_observation(i));

    ceres::CostFunction* cost_function = CollinearityError::CreateAutoDiff(
        observations[2 * i + 0], observations[2 * i + 1]);
    problem.AddResidualBlock(cost_function, nullptr /* squared loss */,
                             bba_problem.mutable_sensor_by_observation(i),
                             bba_problem.mutable_camera_by_observation(i),
                             bba_problem.mutable_point_by_observation(i));
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