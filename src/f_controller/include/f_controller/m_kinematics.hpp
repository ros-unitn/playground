#define _USE_MATH_DEFINES

#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Householder>

class KIN {
public:
  KIN();
  ~KIN();

  static const Eigen::MatrixXf ik(const Eigen::Matrix4f &T60);                      // get joint angles from end-effector position and orientation
  static const Eigen::Matrix4f fk(const Eigen::VectorXf &theta);                    // get homogeneous matrix from forward kinematics
  static const Eigen::Vector3f get_position(const Eigen::Matrix4f &homo_matrix);    // get end-effector position using homogeneous matrix
  static const Eigen::Matrix3f get_orientation(const Eigen::Matrix4f &homo_matrix); // get end-effector orientation using homogeneouse matrix

  static const Eigen::MatrixXf p2p(const Eigen::VectorXf &qEs, const Eigen::Vector3f &xEf, const Eigen::Vector3f &phiEf); // get trajectory in order to reach end-effector position and orientation

  static const Eigen::Matrix3f eul2rotm(const Eigen::Vector3f &eul);                                                         // convert euler angles to rotation matrix
  static const Eigen::Vector3f rotm2eul(const Eigen::Matrix3f &rotm);                                                        // convert rotation matrix to euler angles
  static const Eigen::Matrix4f createHomogeneousMatrix(const Eigen::Vector3f &position, const Eigen::Matrix3f &orientation); // create homogeneous matrix from position and orientation

private:
  static constexpr float minT = 0.0;
  static constexpr float maxT = 3.0;
  static constexpr float step = 0.01;

  static const Eigen::VectorXf a;     // link lengths
  static const Eigen::VectorXf d;     // link offset
  static const Eigen::VectorXf alpha; // link twist

  static const Eigen::Matrix4f computeMatrix(float theta, short frame); // return matrix based on theta and frame
  static const Eigen::VectorXf checkNan(const Eigen::MatrixXf &matrix); // check if matrix contains NaN values
};