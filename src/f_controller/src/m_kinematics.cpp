#include "f_controller/m_kinematics.hpp"

const Eigen::VectorXd K_UR5::a = (Eigen::VectorXd() << 0, 0, -0.425, 0.39225, 0, 0).finished();
const Eigen::VectorXd K_UR5::d = (Eigen::VectorXd() << 0.089159, 0, 0, 0.10915, 0.09465, 0.0823).finished();
const Eigen::VectorXd K_UR5::alpha = (Eigen::VectorXd() << 0, M_PI / 2, 0, 0, M_PI / 2, -M_PI / 2).finished();

Eigen::Matrix4d K_UR5::computeMatrix(double theta, short frame) {
  Eigen::Matrix4d T;
  T << cos(theta), -sin(theta), 0, a(frame),
      sin(theta) * cos(alpha(frame)), cos(theta) * cos(alpha(frame)), -sin(alpha(frame)), -sin(alpha(frame)) * d(frame),
      sin(theta) * sin(alpha(frame)), cos(theta) * sin(alpha(frame)), cos(alpha(frame)), cos(alpha(frame)) * d(frame),
      0, 0, 0, 1;
  return T;
}

Eigen::Matrix4d K_UR5::fk(const Eigen::VectorXd &theta) {

  Eigen::Matrix4d T10;
  T10 << cos(theta(0)), -sin(theta(0)), 0, a(0),
      sin(theta(0)) * cos(alpha(0)), cos(theta(0)) * cos(alpha(0)), -sin(alpha(0)), -sin(alpha(0)) * d(0),
      sin(theta(0)) * sin(alpha(0)), cos(theta(0)) * sin(alpha(0)), cos(alpha(0)), cos(alpha(0)) * d(0),
      0, 0, 0, 1;

  Eigen::Matrix4d T21;
  T21 << cos(theta(1)), -sin(theta(1)), 0, a(1),
      sin(theta(1)) * cos(alpha(1)), cos(theta(1)) * cos(alpha(1)), -sin(alpha(1)), -sin(alpha(1)) * d(1),
      sin(theta(1)) * sin(alpha(1)), cos(theta(1)) * sin(alpha(1)), cos(alpha(1)), cos(alpha(1)) * d(1),
      0, 0, 0, 1;

  Eigen::Matrix4d T32;
  T32 << cos(theta(2)), -sin(theta(2)), 0, a(2),
      sin(theta(2)) * cos(alpha(2)), cos(theta(2)) * cos(alpha(2)), -sin(alpha(2)), -sin(alpha(2)) * d(2),
      sin(theta(2)) * sin(alpha(2)), cos(theta(2)) * sin(alpha(2)), cos(alpha(2)), cos(alpha(2)) * d(2),
      0, 0, 0, 1;

  Eigen::Matrix4d T43;
  T43 << cos(theta(3)), -sin(theta(3)), 0, a(3),
      sin(theta(3)) * cos(alpha(3)), cos(theta(3)) * cos(alpha(3)), -sin(alpha(3)), -sin(alpha(3)) * d(3),
      sin(theta(3)) * sin(alpha(3)), cos(theta(3)) * sin(alpha(3)), cos(alpha(3)), cos(alpha(3)) * d(3),
      0, 0, 0, 1;

  Eigen::Matrix4d T54;
  T54 << cos(theta(4)), -sin(theta(4)), 0, a(4),
      sin(theta(4)) * cos(alpha(4)), cos(theta(4)) * cos(alpha(4)), -sin(alpha(4)), -sin(alpha(4)) * d(4),
      sin(theta(4)) * sin(alpha(4)), cos(theta(4)) * sin(alpha(4)), cos(alpha(4)), cos(alpha(4)) * d(4),
      0, 0, 0, 1;

  Eigen::Matrix4d T65;
  T65 << cos(theta(5)), -sin(theta(5)), 0, a(5),
      sin(theta(5)) * cos(alpha(5)), cos(theta(5)) * cos(alpha(5)), -sin(alpha(5)), -sin(alpha(5)) * d(5),
      sin(theta(5)) * sin(alpha(5)), cos(theta(5)) * sin(alpha(5)), cos(alpha(5)), cos(alpha(5)) * d(5),
      0, 0, 0, 1;

  Eigen::Matrix4d T60;
  T60 = T10 * T21 * T32 * T43 * T54 * T65;

  return T60;
}

Eigen::MatrixXd K_UR5::ik(const Eigen::Matrix4d &T60) {

  Eigen::Vector3d p60 = K_UR5::get_position(T60);
  Eigen::Matrix3d R60 = K_UR5::get_orientation(T60);

  Eigen::Vector4d tmp;
  tmp << 0, 0, -d(5), 1;
  Eigen::Vector4d p50;
  p50 = T60 * tmp;

  // find theta1
  double th1_1 = std::real(atan2(p50(1), p50(0)) + acos(d(3) / hypot(p50(1), p50(0)))) + M_PI / 2;
  double th1_2 = std::real(atan2(p50(1), p50(0)) - acos(d(3) / hypot(p50(1), p50(0)))) + M_PI / 2;

  // finding theta5
  double th5_1 = +std::real(acos((p60(0) * sin(th1_1) - p60(1) * cos(th1_1) - d(3)) / d(5)));
  double th5_2 = -std::real(acos((p60(0) * sin(th1_1) - p60(1) * cos(th1_1) - d(3)) / d(5)));
  double th5_3 = +std::real(acos((p60(0) * sin(th1_2) - p60(1) * cos(th1_2) - d(3)) / d(5)));
  double th5_4 = -std::real(acos((p60(0) * sin(th1_2) - p60(1) * cos(th1_2) - d(3)) / d(5)));

  double p61y = p60(0) * -sin(th1_1) + p60(1) * cos(th1_1);
  // assert(abs(p61y-d(3))<=abs(d(5)));
  p61y = p60(0) * -sin(th1_2) + p60(1) * cos(th1_2);
  // assert(abs(p61y-d(3))<=abs(d(5)));

  Eigen::Matrix4d T06 = T60.inverse();
  Eigen::Vector3d xhat;
  xhat << T06(0, 0), T06(1, 0), T06(2, 0);
  Eigen::Vector3d yhat;
  yhat << T06(0, 1), T06(1, 1), T06(2, 1);

  assert(sin(th5_1) != 0);
  assert(sin(th5_3) != 0);

  // find theta6
  double th6_1 = std::real(atan2(((-xhat(1) * sin(th1_1) + yhat(1) * cos(th1_1))) / sin(th5_1), ((xhat(0) * sin(th1_1) - yhat(0) * cos(th1_1))) / sin(th5_1)));
  double th6_2 = std::real(atan2(((-xhat(1) * sin(th1_1) + yhat(1) * cos(th1_1))) / sin(th5_2), ((xhat(0) * sin(th1_1) - yhat(0) * cos(th1_1))) / sin(th5_2)));
  double th6_3 = std::real(atan2(((-xhat(1) * sin(th1_2) + yhat(1) * cos(th1_2))) / sin(th5_3), ((xhat(0) * sin(th1_2) - yhat(0) * cos(th1_2))) / sin(th5_3)));
  double th6_4 = std::real(atan2(((-xhat(1) * sin(th1_2) + yhat(1) * cos(th1_2))) / sin(th5_4), ((xhat(0) * sin(th1_2) - yhat(0) * cos(th1_2))) / sin(th5_4)));

  Eigen::Matrix4d T41m;

  T41m = computeMatrix(th1_1, 0).inverse() * T60 * computeMatrix(th6_1, 5).inverse() * computeMatrix(th5_1, 4).inverse();
  Eigen::Vector3d p41_1;
  p41_1 << T41m(0, 3), T41m(1, 3), T41m(2, 3);
  double p41xz_1 = hypot(p41_1(0), p41_1(2));

  T41m = computeMatrix(th1_1, 0).inverse() * T60 * computeMatrix(th6_2, 5).inverse() * computeMatrix(th5_2, 4).inverse();
  Eigen::Vector3d p41_2;
  p41_2 << T41m(0, 3), T41m(1, 3), T41m(2, 3);
  double p41xz_2 = hypot(p41_2(0), p41_2(2));

  T41m = computeMatrix(th1_2, 0).inverse() * T60 * computeMatrix(th6_3, 5).inverse() * computeMatrix(th5_3, 4).inverse();
  Eigen::Vector3d p41_3;
  p41_3 << T41m(0, 3), T41m(1, 3), T41m(2, 3);
  double p41xz_3 = hypot(p41_3(0), p41_3(2));

  T41m = computeMatrix(th1_2, 0).inverse() * T60 * computeMatrix(th6_4, 5).inverse() * computeMatrix(th5_4, 4).inverse();
  Eigen::Vector3d p41_4;
  p41_4 << T41m(0, 3), T41m(1, 3), T41m(2, 3);
  double p41xz_4 = hypot(p41_4(0), p41_4(2));

  // find theta3
  double th3_1 = std::real(acos((pow(p41xz_1, 2) - pow(a(2), 2) - pow(a(3), 2)) / (2.0 * a(2) * a(3))));
  double th3_2 = std::real(acos((pow(p41xz_2, 2) - pow(a(2), 2) - pow(a(3), 2)) / (2.0 * a(2) * a(3))));
  double th3_3 = std::real(acos((pow(p41xz_3, 2) - pow(a(2), 2) - pow(a(3), 2)) / (2.0 * a(2) * a(3))));
  double th3_4 = std::real(acos((pow(p41xz_4, 2) - pow(a(2), 2) - pow(a(3), 2)) / (2.0 * a(2) * a(3))));

  double th3_5 = -th3_1;
  double th3_6 = -th3_2;
  double th3_7 = -th3_3;
  double th3_8 = -th3_4;

  double th2_1 = std::real(atan2(-p41_1(2), -p41_1(0)) - asin((-a(3) * sin(th3_1)) / p41xz_1));
  double th2_2 = std::real(atan2(-p41_2(2), -p41_2(0)) - asin((-a(3) * sin(th3_2)) / p41xz_2));
  double th2_3 = std::real(atan2(-p41_3(2), -p41_3(0)) - asin((-a(3) * sin(th3_3)) / p41xz_3));
  double th2_4 = std::real(atan2(-p41_4(2), -p41_4(0)) - asin((-a(3) * sin(th3_4)) / p41xz_4));

  double th2_5 = std::real(atan2(-p41_1(2), -p41_1(0)) - asin((a(3) * sin(th3_1)) / p41xz_1));
  double th2_6 = std::real(atan2(-p41_2(2), -p41_2(0)) - asin((a(3) * sin(th3_2)) / p41xz_2));
  double th2_7 = std::real(atan2(-p41_3(2), -p41_3(0)) - asin((a(3) * sin(th3_3)) / p41xz_3));
  double th2_8 = std::real(atan2(-p41_4(2), -p41_4(0)) - asin((a(3) * sin(th3_4)) / p41xz_4));

  Eigen::Matrix4d T43m;

  T43m = computeMatrix(th3_1, 2).inverse() * computeMatrix(th2_1, 1).inverse() * computeMatrix(th1_1, 0).inverse() * T60 * computeMatrix(th6_1, 5).inverse() * computeMatrix(th5_1, 4).inverse();
  Eigen::Vector3d xhat43;
  xhat43 << T43m(0, 0), T43m(1, 0), T43m(2, 0);
  double th4_1 = std::real(atan2(xhat43(1), xhat43(0)));

  T43m = computeMatrix(th3_2, 2).inverse() * computeMatrix(th2_2, 1).inverse() * computeMatrix(th1_1, 0).inverse() * T60 * computeMatrix(th6_2, 5).inverse() * computeMatrix(th5_2, 4).inverse();
  xhat43 << T43m(0, 0), T43m(1, 0), T43m(2, 0);
  double th4_2 = std::real(atan2(xhat43(1), xhat43(0)));

  T43m = computeMatrix(th3_3, 2).inverse() * computeMatrix(th2_3, 1).inverse() * computeMatrix(th1_2, 0).inverse() * T60 * computeMatrix(th6_3, 5).inverse() * computeMatrix(th5_3, 4).inverse();
  xhat43 << T43m(0, 0), T43m(1, 0), T43m(2, 0);
  double th4_3 = std::real(atan2(xhat43(1), xhat43(0)));

  T43m = computeMatrix(th3_4, 2).inverse() * computeMatrix(th2_4, 1).inverse() * computeMatrix(th1_2, 0).inverse() * T60 * computeMatrix(th6_4, 5).inverse() * computeMatrix(th5_4, 4).inverse();
  xhat43 << T43m(0, 0), T43m(1, 0), T43m(2, 0);
  double th4_4 = std::real(atan2(xhat43(1), xhat43(0)));

  T43m = computeMatrix(th3_5, 2).inverse() * computeMatrix(th2_5, 1).inverse() * computeMatrix(th1_1, 0).inverse() * T60 * computeMatrix(th6_1, 5).inverse() * computeMatrix(th5_1, 4).inverse();
  xhat43 << T43m(0, 0), T43m(1, 0), T43m(2, 0);
  double th4_5 = std::real(atan2(xhat43(1), xhat43(0)));

  T43m = computeMatrix(th3_6, 2).inverse() * computeMatrix(th2_6, 1).inverse() * computeMatrix(th1_1, 0).inverse() * T60 * computeMatrix(th6_2, 5).inverse() * computeMatrix(th5_2, 4).inverse();
  xhat43 << T43m(0, 0), T43m(1, 0), T43m(2, 0);
  double th4_6 = std::real(atan2(xhat43(1), xhat43(0)));

  T43m = computeMatrix(th3_7, 2).inverse() * computeMatrix(th2_7, 1).inverse() * computeMatrix(th1_2, 0).inverse() * T60 * computeMatrix(th6_3, 5).inverse() * computeMatrix(th5_3, 4).inverse();
  xhat43 << T43m(0, 0), T43m(1, 0), T43m(2, 0);
  double th4_7 = std::real(atan2(xhat43(1), xhat43(0)));

  T43m = computeMatrix(th3_8, 2).inverse() * computeMatrix(th2_8, 1).inverse() * computeMatrix(th1_2, 0).inverse() * T60 * computeMatrix(th6_4, 5).inverse() * computeMatrix(th5_4, 4).inverse();
  xhat43 << T43m(0, 0), T43m(1, 0), T43m(2, 0);
  double th4_8 = std::real(atan2(xhat43(1), xhat43(0)));

  Eigen::MatrixXd Th(8, 6);

  // display the results
  Th << th1_1, th2_1, th3_1, th4_1, th5_1, th6_1,
      th1_1, th2_2, th3_2, th4_2, th5_2, th6_2,
      th1_2, th2_3, th3_3, th4_3, th5_3, th6_3,
      th1_2, th2_4, th3_4, th4_4, th5_4, th6_4,
      th1_1, th2_5, th3_5, th4_5, th5_1, th6_1,
      th1_1, th2_6, th3_6, th4_6, th5_2, th6_2,
      th1_2, th2_7, th3_7, th4_7, th5_3, th6_3,
      th1_2, th2_8, th3_8, th4_8, th5_4, th6_4;

  return Th;
}

Eigen::Vector3d K_UR5::get_position(const Eigen::Matrix4d &homo_matrix) {
  return (Eigen::Vector3d() << homo_matrix(0, 3), homo_matrix(1, 3), homo_matrix(2, 3)).finished();
}

Eigen::Matrix3d K_UR5::get_orientation(const Eigen::Matrix4d &homo_matrix) {
  return (Eigen::Matrix3d() << homo_matrix(0, 0), homo_matrix(0, 1), homo_matrix(0, 2),
          homo_matrix(1, 0), homo_matrix(1, 1), homo_matrix(1, 2),
          homo_matrix(2, 0), homo_matrix(2, 1), homo_matrix(2, 2))
      .finished();
}