#define _USE_MATH_DEFINES

#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Householder>
#include <vector>

class Kinematics {
public:
  // convert euler angles to rotation matrix
  static const Eigen::Matrix3d eul2rotm(const Eigen::Vector3d &eul);
  // convert rotation matrix to euler angles
  static const Eigen::Vector3d rotm2eul(const Eigen::Matrix3d &rotm);
  // create homogeneous matrix from position and orientation
  static const Eigen::Matrix4d create_homogeneous_matrix(const Eigen::Vector3d &position, const Eigen::Matrix3d &orientation);
  // get end-effector position using homogeneous matrix
  static const Eigen::Vector3d get_position(const Eigen::Matrix4d &homo_matrix);
  // get end-effector orientation using homogeneouse matrix
  static const Eigen::Matrix3d get_orientation(const Eigen::Matrix4d &homo_matrix);
  // get trajectory in order to reach end-effector position and orientation
  static const Eigen::MatrixXd p2p(const Eigen::VectorXd &qEs, const Eigen::VectorXd &qEf, double maxT);
  // get joint angles from end-effector position and orientation
  static const Eigen::MatrixXd ik(const Eigen::Matrix4d &T60);
  // get homogeneous matrix from forward kinematics
  static const Eigen::Matrix4d fk(const Eigen::VectorXd &theta);

  // TODO: remove
  // get best angles for p2p trajectory
  static const Eigen::VectorXd best_angles(const Eigen::VectorXd &actual, const Eigen::MatrixXd &possible);

private:
  static constexpr double minT = 0.0;
  //static constexpr double maxT = 3.0;
  static constexpr double step = 0.01;

  // link lengths
  static const Eigen::VectorXd a;
  // link offset
  static const Eigen::VectorXd d;
  // link twist
  static const Eigen::VectorXd alpha;

private:
  // return matrix based on theta and frame
  static const Eigen::Matrix4d compute_matrix(double theta, short frame);
  // check if matrix contains NaN values
  static const bool check_row(const Eigen::VectorXd &vec);
  // set 0-closed thetas
  static const double closest_theta(double th);
};