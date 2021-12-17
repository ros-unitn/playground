#include "f_controller/m_kinematics.hpp"
#include <iostream>

const Eigen::VectorXd KIN::a = (Eigen::VectorXd(6) << 0, -0.425, -0.39225, 0, 0, 0).finished();
const Eigen::VectorXd KIN::d = (Eigen::VectorXd(6) << 0.089159, 0, 0, 0.10915, 0.09465, 0.0823).finished();
const Eigen::VectorXd KIN::alpha = (Eigen::VectorXd(6) << 0, M_PI_2, 0, 0, M_PI_2, -M_PI_2).finished();

constexpr double KIN::minT;
constexpr double KIN::maxT;
constexpr double KIN::step;

const Eigen::Matrix3d KIN::eul2rotm(const Eigen::Vector3d &eul) {
  return (Eigen::Matrix3d() << cos(eul(0)) * cos(eul(1)), cos(eul(0)) * sin(eul(1)) * sin(eul(2)) - sin(eul(0)) * cos(eul(2)), cos(eul(0)) * sin(eul(1)) * cos(eul(2)) + sin(eul(0)) * sin(eul(2)),
          sin(eul(0)) * cos(eul(1)), sin(eul(0)) * sin(eul(1)) * sin(eul(2)) + cos(eul(0)) * cos(eul(2)), sin(eul(0)) * sin(eul(1)) * cos(eul(2)) - cos(eul(0)) * sin(eul(2)),
          -sin(eul(1)), cos(eul(1)) * sin(eul(2)), cos(eul(1)) * cos(eul(2)))
      .finished();
}

const Eigen::Vector3d KIN::rotm2eul(const Eigen::Matrix3d &rotm) {
  return rotm.eulerAngles(2, 1, 0);
}

const Eigen::Matrix4d KIN::computeMatrix(double theta, short frame) {
  return (Eigen::Matrix4d() << cos(theta), -sin(theta), 0, a((((frame-1)%6)+6)%6),
          sin(theta) * cos(alpha(frame)), cos(theta) * cos(alpha(frame)), -sin(alpha(frame)), -sin(alpha(frame)) * d(frame),
          sin(theta) * sin(alpha(frame)), cos(theta) * sin(alpha(frame)), cos(alpha(frame)), cos(alpha(frame)) * d(frame),
          0, 0, 0, 1)
      .finished();
}

const bool KIN::checkRow(const Eigen::VectorXd &vec) {
  bool validRow = true;
  for (int i = 0; i < vec.rows() && validRow; i++) {
      if (std::isnan(vec(i))) {
        validRow = false;
      }
  }
  return validRow;
}

const Eigen::VectorXd KIN::bestAngles(const Eigen::VectorXd &actual, const Eigen::MatrixXd &possible) {

  Eigen::VectorXd diff(possible.rows());

  for(int i=0; i<possible.rows(); i++){
    if(checkRow(possible.row(i).transpose())) {

      Eigen::VectorXd tmp(possible.cols());
      for(int j=0; j<possible.cols(); j++){
        tmp(j) = abs(possible(i,j)-actual(j));
      }
      diff(i) = tmp.sum();
    }
    else {
      diff(i) = std::numeric_limits<double>::max();
    }
  }

  Eigen::VectorXd::Index minIndex;
  diff.minCoeff(&minIndex);
  if(diff(minIndex)==std::numeric_limits<double>::max()) {
    throw std::runtime_error("No valid solution found");
  }
  return possible.row(minIndex).transpose();
}

const Eigen::MatrixXd KIN::p2p(const Eigen::VectorXd &qEs, const Eigen::Vector3d &xEf, const Eigen::Vector3d &phiEf) {

  Eigen::VectorXd qEf = bestAngles(qEs,ik(createHomogeneousMatrix(xEf, eul2rotm(phiEf))));
  //std::cout << "qEf: " << qEf.transpose() << std::endl;
  //std::cout << "Goal:\t" << KIN::get_position(fk(qEf)).transpose() << std::endl;

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

  int n = 1 + (maxT - minT) / step;
  Eigen::MatrixXd Th(n, 6);

  int rowIndex = 0;

  for (double t = minT; t <= maxT; t += step, rowIndex++) {
    ////std::cout << "t: " << t << std::endl;
    for (int i = 0; i < qEs.size(); i++) {
      Th(rowIndex, i) = A(i, 0) + A(i, 1) * t + A(i, 2) * t * t + A(i, 3) * t * t * t + A(i, 4) * t * t * t * t + A(i, 5) * t * t * t * t * t;
    }
  }

  //std::cout << "End: " << get_position(fk(Th.row(Th.rows()-1).transpose())).transpose() << std::endl;
  return Th;
}

const Eigen::Vector3d KIN::get_position(const Eigen::Matrix4d &homo_matrix) {
  return (Eigen::Vector3d() << homo_matrix(0, 3), homo_matrix(1, 3), homo_matrix(2, 3)).finished();
}

const Eigen::Matrix3d KIN::get_orientation(const Eigen::Matrix4d &homo_matrix) {
  return (Eigen::Matrix3d() << homo_matrix(0, 0), homo_matrix(0, 1), homo_matrix(0, 2),
          homo_matrix(1, 0), homo_matrix(1, 1), homo_matrix(1, 2),
          homo_matrix(2, 0), homo_matrix(2, 1), homo_matrix(2, 2))
      .finished();
}

const Eigen::Matrix4d KIN::createHomogeneousMatrix(const Eigen::Vector3d &position, const Eigen::Matrix3d &orientation) {
  return (Eigen::Matrix4d() << orientation(0, 0), orientation(0, 1), orientation(0, 2), position(0),
          orientation(1, 0), orientation(1, 1), orientation(1, 2), position(1),
          orientation(2, 0), orientation(2, 1), orientation(2, 2), position(2),
          0, 0, 0, 1)
      .finished();
}

const Eigen::Matrix4d KIN::fk(const Eigen::VectorXd &theta) {

  Eigen::Matrix4d T10 = computeMatrix(theta(0), 0);
  Eigen::Matrix4d T21 = computeMatrix(theta(1), 1);
  Eigen::Matrix4d T32 = computeMatrix(theta(2), 2);
  Eigen::Matrix4d T43 = computeMatrix(theta(3), 3);
  Eigen::Matrix4d T54 = computeMatrix(theta(4), 4);
  Eigen::Matrix4d T65 = computeMatrix(theta(5), 5);

  Eigen::Matrix4d T06 = T10 * T21 * T32 * T43 * T54 * T65;

  return T06;
}

const Eigen::MatrixXd KIN::ik(const Eigen::Matrix4d &T60) {

  Eigen::Vector3d p60 = KIN::get_position(T60);

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

  // double p61y = p60(0) * -sin(th1_1) + p60(1) * cos(th1_1);
  // assert(abs(p61y-d(3))<=abs(d(5)));
  // p61y = p60(0) * -sin(th1_2) + p60(1) * cos(th1_2);
  // assert(abs(p61y-d(3))<=abs(d(5)));

  Eigen::Matrix4d T06 = T60.inverse();
  Eigen::Vector3d xhat = (Eigen::Vector3d() << T06(0, 0), T06(1, 0), T06(2, 0)).finished();
  Eigen::Vector3d yhat = (Eigen::Vector3d() << T06(0, 1), T06(1, 1), T06(2, 1)).finished();

  // assert(sin(th5_1) != 0);
  // assert(sin(th5_3) != 0);

  // find theta6
  double th6_1 = std::real(atan2(((-xhat(1) * sin(th1_1) + yhat(1) * cos(th1_1))) / sin(th5_1), ((xhat(0) * sin(th1_1) - yhat(0) * cos(th1_1))) / sin(th5_1)));
  double th6_2 = std::real(atan2(((-xhat(1) * sin(th1_1) + yhat(1) * cos(th1_1))) / sin(th5_2), ((xhat(0) * sin(th1_1) - yhat(0) * cos(th1_1))) / sin(th5_2)));
  double th6_3 = std::real(atan2(((-xhat(1) * sin(th1_2) + yhat(1) * cos(th1_2))) / sin(th5_3), ((xhat(0) * sin(th1_2) - yhat(0) * cos(th1_2))) / sin(th5_3)));
  double th6_4 = std::real(atan2(((-xhat(1) * sin(th1_2) + yhat(1) * cos(th1_2))) / sin(th5_4), ((xhat(0) * sin(th1_2) - yhat(0) * cos(th1_2))) / sin(th5_4)));

  Eigen::Matrix4d T41m;

  T41m = computeMatrix(th1_1, 0).inverse() * T60 * computeMatrix(th6_1, 5).inverse() * computeMatrix(th5_1, 4).inverse();
  Eigen::Vector3d p41_1 = (Eigen::Vector3d() << T41m(0, 3), T41m(1, 3), T41m(2, 3)).finished();
  double p41xz_1 = hypot(p41_1(0), p41_1(2));

  T41m = computeMatrix(th1_1, 0).inverse() * T60 * computeMatrix(th6_2, 5).inverse() * computeMatrix(th5_2, 4).inverse();
  Eigen::Vector3d p41_2 = (Eigen::Vector3d() << T41m(0, 3), T41m(1, 3), T41m(2, 3)).finished();
  double p41xz_2 = hypot(p41_2(0), p41_2(2));

  T41m = computeMatrix(th1_2, 0).inverse() * T60 * computeMatrix(th6_3, 5).inverse() * computeMatrix(th5_3, 4).inverse();
  Eigen::Vector3d p41_3 = (Eigen::Vector3d() << T41m(0, 3), T41m(1, 3), T41m(2, 3)).finished();
  double p41xz_3 = hypot(p41_3(0), p41_3(2));

  T41m = computeMatrix(th1_2, 0).inverse() * T60 * computeMatrix(th6_4, 5).inverse() * computeMatrix(th5_4, 4).inverse();
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

  T43m = computeMatrix(th3_1, 2).inverse() * computeMatrix(th2_1, 1).inverse() * computeMatrix(th1_1, 0).inverse() * T60 * computeMatrix(th6_1, 5).inverse() * computeMatrix(th5_1, 4).inverse();
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

  // save the values in the matrix
  Th << th1_1, th2_1, th3_1, th4_1, th5_1, th6_1,
      th1_1, th2_2, th3_2, th4_2, th5_2, th6_2,
      th1_2, th2_3, th3_3, th4_3, th5_3, th6_3,
      th1_2, th2_4, th3_4, th4_4, th5_4, th6_4,
      th1_1, th2_5, th3_5, th4_5, th5_1, th6_1,
      th1_1, th2_6, th3_6, th4_6, th5_2, th6_2,
      th1_2, th2_7, th3_7, th4_7, th5_3, th6_3,
      th1_2, th2_8, th3_8, th4_8, th5_4, th6_4;

      //std::cout << get_position(fk(Th.row(0))).transpose() << std::endl;
      //std::cout << get_position(fk(Th.row(1))).transpose() << std::endl;
      //std::cout << get_position(fk(Th.row(2))).transpose() << std::endl;
      //std::cout << get_position(fk(Th.row(3))).transpose() << std::endl;
      //std::cout << get_position(fk(Th.row(4))).transpose() << std::endl;
      //std::cout << get_position(fk(Th.row(5))).transpose() << std::endl;
      //std::cout << get_position(fk(Th.row(6))).transpose() << std::endl;
      //std::cout << get_position(fk(Th.row(7))).transpose() << std::endl;

  return Th;
}
