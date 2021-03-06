#include "x_controller/kinematics.hpp"
#include <iostream>

constexpr double Kinematics::minT;
constexpr double Kinematics::step;

const Eigen::VectorXd Kinematics::a = (Eigen::VectorXd(6) << 0, -0.425, -0.39225, 0, 0, 0).finished();
const Eigen::VectorXd Kinematics::d = (Eigen::VectorXd(6) << 0.089159, 0, 0, 0.10915, 0.09465, 0.0823).finished();
const Eigen::VectorXd Kinematics::alpha = (Eigen::VectorXd(6) << 0, M_PI_2, 0, 0, M_PI_2, -M_PI_2).finished();

const Eigen::Matrix4d Kinematics::compute_matrix(double theta, short frame) {
  return (Eigen::Matrix4d() << cos(theta), -sin(theta), 0, a((((frame - 1) % 6) + 6) % 6),
          sin(theta) * cos(alpha(frame)), cos(theta) * cos(alpha(frame)), -sin(alpha(frame)), -sin(alpha(frame)) * d(frame),
          sin(theta) * sin(alpha(frame)), cos(theta) * sin(alpha(frame)), cos(alpha(frame)), cos(alpha(frame)) * d(frame),
          0, 0, 0, 1)
      .finished();
}

const bool Kinematics::check_row(const Eigen::VectorXd &vec) {
  bool validRow = true;
  for (int i = 0; i < vec.rows() && validRow; i++) {
    if (std::isnan(vec(i))) {
      validRow = false;
    }
  }
  return validRow;
}

const Eigen::VectorXd Kinematics::best_angles(const Eigen::VectorXd &actual, const Eigen::MatrixXd &possible) {
  std::vector<std::pair<double, short>> diffs;

  for (int i = 0; i < possible.rows(); i++) {
    double diff = 0;
    if (check_row(possible.row(i).transpose()) && abs(possible.row(i).transpose()(0)) < 2.5 && possible.row(i)(4) < 0) {
      for (int j = 1; j < 4; j++) {
        diff += abs(possible.row(i).transpose()(j) - actual(j));
      }
      diffs.push_back(std::make_pair(diff, i));
    }
  }

  if(diffs.size() == 0) {
    return Eigen::VectorXd::Zero(6);
  }

  std::sort(diffs.begin(), diffs.end());
  return possible.row(diffs[0].second).transpose();
}

const Eigen::Matrix3d Kinematics::eul2rotm(const Eigen::Vector3d &eul) {
  return (Eigen::Matrix3d() << cos(eul(0)) * cos(eul(1)), cos(eul(0)) * sin(eul(1)) * sin(eul(2)) - sin(eul(0)) * cos(eul(2)), cos(eul(0)) * sin(eul(1)) * cos(eul(2)) + sin(eul(0)) * sin(eul(2)),
          sin(eul(0)) * cos(eul(1)), sin(eul(0)) * sin(eul(1)) * sin(eul(2)) + cos(eul(0)) * cos(eul(2)), sin(eul(0)) * sin(eul(1)) * cos(eul(2)) - cos(eul(0)) * sin(eul(2)),
          -sin(eul(1)), cos(eul(1)) * sin(eul(2)), cos(eul(1)) * cos(eul(2)))
      .finished();
}

const Eigen::Matrix4d Kinematics::create_homogeneous_matrix(const Eigen::Vector3d &position, const Eigen::Matrix3d &orientation) {
  return (Eigen::Matrix4d() << orientation(0, 0), orientation(0, 1), orientation(0, 2), position(0),
          orientation(1, 0), orientation(1, 1), orientation(1, 2), position(1),
          orientation(2, 0), orientation(2, 1), orientation(2, 2), position(2),
          0, 0, 0, 1)
      .finished();
}

const Eigen::Vector3d Kinematics::get_position(const Eigen::Matrix4d &homo_matrix) {
  return (Eigen::Vector3d() << homo_matrix(0, 3), homo_matrix(1, 3), homo_matrix(2, 3)).finished();
}

const Eigen::Matrix3d Kinematics::get_orientation(const Eigen::Matrix4d &homo_matrix) {
  return (Eigen::Matrix3d() << homo_matrix(0, 0), homo_matrix(0, 1), homo_matrix(0, 2),
          homo_matrix(1, 0), homo_matrix(1, 1), homo_matrix(1, 2),
          homo_matrix(2, 0), homo_matrix(2, 1), homo_matrix(2, 2))
      .finished();
}

const Eigen::MatrixXd Kinematics::p2p(const Eigen::VectorXd &qEs, const Eigen::VectorXd &qEf, double maxT) {

  Eigen::MatrixXd A(6, 6);

  Eigen::MatrixXd M = (Eigen::MatrixXd(6, 6) << 1, minT, minT * minT, minT * minT * minT, minT * minT * minT * minT, minT * minT * minT * minT * minT,
                       0, 1, 2 * minT, 3 * minT * minT, 4 * minT * minT * minT, 5 * minT * minT * minT * minT,
                       0, 0, 2, 6 * minT, 12 * minT * minT, 20 * minT * minT * minT,
                       1, maxT, maxT * maxT, maxT * maxT * maxT, maxT * maxT * maxT * maxT, maxT * maxT * maxT * maxT * maxT,
                       0, 1, 2 * maxT, 3 * maxT * maxT, 4 * maxT * maxT * maxT, 5 * maxT * maxT * maxT * maxT,
                       0, 0, 2, 6 * maxT, 12 * maxT * maxT, 20 * maxT * maxT * maxT)
                          .finished();

  Eigen::HouseholderQR<Eigen::MatrixXd> m(M);

  for (int i = 0; i < qEs.size(); i++) {

    Eigen::VectorXd b = (Eigen::VectorXd(6) << qEs(i), 0, 0, qEf(i), 0, 0).finished();
    A.row(i) = m.solve(b).transpose();
  }

  int n = (maxT - minT) / step;
  Eigen::MatrixXd Th(n, 6);

  int rowIndex = 0;

  for (double t = minT; fabs(t-maxT) > 1E-3; t+=step, rowIndex++) {
    for (int i = 0; i < qEs.size(); i++) {
      Th(rowIndex, i) = A(i, 0) + A(i, 1) * t + A(i, 2) * t * t + A(i, 3) * t * t * t + A(i, 4) * t * t * t * t + A(i, 5) * t * t * t * t * t;
    }
  }

  return Th;
}

const double Kinematics::closest_theta(double th) {
  double two_pi = M_PI * 2;
  if (th > 0) {
    return (abs(th - two_pi) < abs(th)) ? th - two_pi : th;
  } else {
    return (abs(th + two_pi) < abs(th)) ? th + two_pi : th;
  }
}

const Eigen::MatrixXd Kinematics::ik(const Eigen::Matrix4d &T60) {

  Eigen::Vector3d p60 = Kinematics::get_position(T60);

  Eigen::Vector4d pos = (Eigen::Vector4d() << 0, 0, -d(5), 1).finished();
  Eigen::Vector4d p50 = T60 * pos;

  // find theta1
  double th1_1 = std::real(atan2(p50(1), p50(0)) + acos(d(3) / hypot(p50(1), p50(0)))) + M_PI_2;
  double th1_2 = std::real(atan2(p50(1), p50(0)) - acos(d(3) / hypot(p50(1), p50(0)))) + M_PI_2;

  // finding theta5
  double th5_1 = +std::real(acos((p60(0) * sin(th1_1) - p60(1) * cos(th1_1) - d(3)) / d(5)));
  double th5_2 = -std::real(acos((p60(0) * sin(th1_1) - p60(1) * cos(th1_1) - d(3)) / d(5)));
  double th5_3 = +std::real(acos((p60(0) * sin(th1_2) - p60(1) * cos(th1_2) - d(3)) / d(5)));
  double th5_4 = -std::real(acos((p60(0) * sin(th1_2) - p60(1) * cos(th1_2) - d(3)) / d(5)));

  Eigen::Matrix4d T06 = T60.inverse();
  Eigen::Vector3d xhat = (Eigen::Vector3d() << T06(0, 0), T06(1, 0), T06(2, 0)).finished();
  Eigen::Vector3d yhat = (Eigen::Vector3d() << T06(0, 1), T06(1, 1), T06(2, 1)).finished();

  // find theta6
  double th6_1 = std::real(atan2(((-xhat(1) * sin(th1_1) + yhat(1) * cos(th1_1))) / sin(th5_1), ((xhat(0) * sin(th1_1) - yhat(0) * cos(th1_1))) / sin(th5_1)));
  double th6_2 = std::real(atan2(((-xhat(1) * sin(th1_1) + yhat(1) * cos(th1_1))) / sin(th5_2), ((xhat(0) * sin(th1_1) - yhat(0) * cos(th1_1))) / sin(th5_2)));
  double th6_3 = std::real(atan2(((-xhat(1) * sin(th1_2) + yhat(1) * cos(th1_2))) / sin(th5_3), ((xhat(0) * sin(th1_2) - yhat(0) * cos(th1_2))) / sin(th5_3)));
  double th6_4 = std::real(atan2(((-xhat(1) * sin(th1_2) + yhat(1) * cos(th1_2))) / sin(th5_4), ((xhat(0) * sin(th1_2) - yhat(0) * cos(th1_2))) / sin(th5_4)));

  Eigen::Matrix4d T41m;

  T41m = compute_matrix(th1_1, 0).inverse() * T60 * compute_matrix(th6_1, 5).inverse() * compute_matrix(th5_1, 4).inverse();
  Eigen::Vector3d p41_1 = (Eigen::Vector3d() << T41m(0, 3), T41m(1, 3), T41m(2, 3)).finished();
  double p41xz_1 = hypot(p41_1(0), p41_1(2));

  T41m = compute_matrix(th1_1, 0).inverse() * T60 * compute_matrix(th6_2, 5).inverse() * compute_matrix(th5_2, 4).inverse();
  Eigen::Vector3d p41_2 = (Eigen::Vector3d() << T41m(0, 3), T41m(1, 3), T41m(2, 3)).finished();
  double p41xz_2 = hypot(p41_2(0), p41_2(2));

  T41m = compute_matrix(th1_2, 0).inverse() * T60 * compute_matrix(th6_3, 5).inverse() * compute_matrix(th5_3, 4).inverse();
  Eigen::Vector3d p41_3 = (Eigen::Vector3d() << T41m(0, 3), T41m(1, 3), T41m(2, 3)).finished();
  double p41xz_3 = hypot(p41_3(0), p41_3(2));

  T41m = compute_matrix(th1_2, 0).inverse() * T60 * compute_matrix(th6_4, 5).inverse() * compute_matrix(th5_4, 4).inverse();
  Eigen::Vector3d p41_4 = (Eigen::Vector3d() << T41m(0, 3), T41m(1, 3), T41m(2, 3)).finished();
  double p41xz_4 = hypot(p41_4(0), p41_4(2));

  // find theta3
  double th3_1 = std::real(acos((pow(p41xz_1, 2) - pow(a(1), 2) - pow(a(2), 2)) / (2.0 * a(1) * a(2))));
  double th3_2 = std::real(acos((pow(p41xz_2, 2) - pow(a(1), 2) - pow(a(2), 2)) / (2.0 * a(1) * a(2))));
  double th3_3 = std::real(acos((pow(p41xz_3, 2) - pow(a(1), 2) - pow(a(2), 2)) / (2.0 * a(1) * a(2))));
  double th3_4 = std::real(acos((pow(p41xz_4, 2) - pow(a(1), 2) - pow(a(2), 2)) / (2.0 * a(1) * a(2))));

  double th3_5 = -th3_1;
  double th3_6 = -th3_2;
  double th3_7 = -th3_3;
  double th3_8 = -th3_4;

  double th2_1 = std::real(atan2(-p41_1(2), -p41_1(0)) - asin((-a(2) * sin(th3_1)) / p41xz_1));
  double th2_2 = std::real(atan2(-p41_2(2), -p41_2(0)) - asin((-a(2) * sin(th3_2)) / p41xz_2));
  double th2_3 = std::real(atan2(-p41_3(2), -p41_3(0)) - asin((-a(2) * sin(th3_3)) / p41xz_3));
  double th2_4 = std::real(atan2(-p41_4(2), -p41_4(0)) - asin((-a(2) * sin(th3_4)) / p41xz_4));

  double th2_5 = std::real(atan2(-p41_1(2), -p41_1(0)) - asin((a(2) * sin(th3_1)) / p41xz_1));
  double th2_6 = std::real(atan2(-p41_2(2), -p41_2(0)) - asin((a(2) * sin(th3_2)) / p41xz_2));
  double th2_7 = std::real(atan2(-p41_3(2), -p41_3(0)) - asin((a(2) * sin(th3_3)) / p41xz_3));
  double th2_8 = std::real(atan2(-p41_4(2), -p41_4(0)) - asin((a(2) * sin(th3_4)) / p41xz_4));

  Eigen::Matrix4d T43m;
  Eigen::Vector3d xhat43;

  T43m = compute_matrix(th3_1, 2).inverse() * compute_matrix(th2_1, 1).inverse() * compute_matrix(th1_1, 0).inverse() * T60 * compute_matrix(th6_1, 5).inverse() * compute_matrix(th5_1, 4).inverse();
  xhat43 << T43m(0, 0), T43m(1, 0), T43m(2, 0);
  double th4_1 = std::real(atan2(xhat43(1), xhat43(0)));

  T43m = compute_matrix(th3_2, 2).inverse() * compute_matrix(th2_2, 1).inverse() * compute_matrix(th1_1, 0).inverse() * T60 * compute_matrix(th6_2, 5).inverse() * compute_matrix(th5_2, 4).inverse();
  xhat43 << T43m(0, 0), T43m(1, 0), T43m(2, 0);
  double th4_2 = std::real(atan2(xhat43(1), xhat43(0)));

  T43m = compute_matrix(th3_3, 2).inverse() * compute_matrix(th2_3, 1).inverse() * compute_matrix(th1_2, 0).inverse() * T60 * compute_matrix(th6_3, 5).inverse() * compute_matrix(th5_3, 4).inverse();
  xhat43 << T43m(0, 0), T43m(1, 0), T43m(2, 0);
  double th4_3 = std::real(atan2(xhat43(1), xhat43(0)));

  T43m = compute_matrix(th3_4, 2).inverse() * compute_matrix(th2_4, 1).inverse() * compute_matrix(th1_2, 0).inverse() * T60 * compute_matrix(th6_4, 5).inverse() * compute_matrix(th5_4, 4).inverse();
  xhat43 << T43m(0, 0), T43m(1, 0), T43m(2, 0);
  double th4_4 = std::real(atan2(xhat43(1), xhat43(0)));

  T43m = compute_matrix(th3_5, 2).inverse() * compute_matrix(th2_5, 1).inverse() * compute_matrix(th1_1, 0).inverse() * T60 * compute_matrix(th6_1, 5).inverse() * compute_matrix(th5_1, 4).inverse();
  xhat43 << T43m(0, 0), T43m(1, 0), T43m(2, 0);
  double th4_5 = std::real(atan2(xhat43(1), xhat43(0)));

  T43m = compute_matrix(th3_6, 2).inverse() * compute_matrix(th2_6, 1).inverse() * compute_matrix(th1_1, 0).inverse() * T60 * compute_matrix(th6_2, 5).inverse() * compute_matrix(th5_2, 4).inverse();
  xhat43 << T43m(0, 0), T43m(1, 0), T43m(2, 0);
  double th4_6 = std::real(atan2(xhat43(1), xhat43(0)));

  T43m = compute_matrix(th3_7, 2).inverse() * compute_matrix(th2_7, 1).inverse() * compute_matrix(th1_2, 0).inverse() * T60 * compute_matrix(th6_3, 5).inverse() * compute_matrix(th5_3, 4).inverse();
  xhat43 << T43m(0, 0), T43m(1, 0), T43m(2, 0);
  double th4_7 = std::real(atan2(xhat43(1), xhat43(0)));

  T43m = compute_matrix(th3_8, 2).inverse() * compute_matrix(th2_8, 1).inverse() * compute_matrix(th1_2, 0).inverse() * T60 * compute_matrix(th6_4, 5).inverse() * compute_matrix(th5_4, 4).inverse();
  xhat43 << T43m(0, 0), T43m(1, 0), T43m(2, 0);
  double th4_8 = std::real(atan2(xhat43(1), xhat43(0)));

  Eigen::MatrixXd Th(8, 6);

  // save the values in the matrix
  Th << th1_1, th2_1, th3_1, th4_1, th5_1, th6_1,
      th1_1, th2_2, th3_2, th4_2, th5_2, th6_2,
      th1_2, th2_3, th3_3, th4_3, th5_3, th6_3,
      th1_2, th2_4, th3_4, th4_4, th5_4, th6_4,
      th1_1, th2_5, th3_5, th4_5, th5_1, th6_1,
      th1_1, th2_6, th3_6, th4_6, th5_2, th6_2,
      th1_2, th2_7, th3_7, th4_7, th5_3, th6_3,
      th1_2, th2_8, th3_8, th4_8, th5_4, th6_4;

  Th = Th.unaryExpr(&closest_theta);

  return Th;
}
