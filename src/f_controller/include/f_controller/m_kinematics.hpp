#define _USE_MATH_DEFINES

#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

class K_UR5 {
public:
  K_UR5();
  ~K_UR5();

  Eigen::MatrixXd ik(const Eigen::Matrix4d &T60);                      // get joint angles from end-effector position and orientation
  Eigen::Matrix4d fk(const Eigen::VectorXd &theta);                    // get homogeneous matrix from forward kinematics
  Eigen::Vector3d get_position(const Eigen::Matrix4d &homo_matrix);    // get end-effector position using homogeneous matrix
  Eigen::Matrix3d get_orientation(const Eigen::Matrix4d &homo_matrix); // get end-effector orientation using homogeneouse matrix

  Eigen::MatrixXd p2p(const Eigen::VectorXd &qEs, const Eigen::Vector3d &xEf, const Eigen::Vector3d &phiEf); // get trajectory in order to reach end-effector position and orientation

private:
  static constexpr float minT = 0;
  static constexpr float maxT = 3;
  static constexpr float step = 0.01;

  static const Eigen::VectorXd a;     // link lengths
  static const Eigen::VectorXd d;     // link offset
  static const Eigen::VectorXd alpha; // link twist

  Eigen::Matrix4d computeMatrix(double theta, short frame); // return matrix based on theta and frame

  Eigen::Matrix3d eul2rotm(const Eigen::Vector3d &eul);                                                         // convert euler angles to rotation matrix
  Eigen::Vector3d rotm2eul(const Eigen::Matrix3d &rotm);                                                         // convert rotation matrix to euler angles
  Eigen::Matrix4d createHomogeneousMatrix(const Eigen::Vector3d &position, const Eigen::Matrix3d &orientation); // create homogeneous matrix from position and orientation
};