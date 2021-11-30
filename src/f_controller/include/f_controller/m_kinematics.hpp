#define _USE_MATH_DEFINES

#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>

class K_UR5 {
public:
  K_UR5();
  ~K_UR5();
  static const Eigen::VectorXd a;                                      // link lengths
  static const Eigen::VectorXd d;                                      // link offset
  static const Eigen::VectorXd alpha;                                  // link twist
  Eigen::MatrixXd ik(const Eigen::Matrix4d &T60);                      // get joint angles from end-effector position and orientation
  Eigen::Matrix4d fk(const Eigen::VectorXd &theta);                    // get homogeneous matrix from forward kinematics
  Eigen::Vector3d get_position(const Eigen::Matrix4d &homo_matrix);    // get end-effector position using homogeneous matrix
  Eigen::Matrix3d get_orientation(const Eigen::Matrix4d &homo_matrix); // get end-effector orientation using homogeneouse matrix

private:
  Eigen::Matrix4d computeMatrix(double theta, short frame);
};