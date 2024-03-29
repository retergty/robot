#include "Dense"
#include "Geometry"
#include "../unsupported/Eigen/MatrixFunctions"
#include <vector>
#include <list>
#include <math.h>
#include <iostream>
#include "HelperFunction.h"

template<typename scalar>
struct Pose;

template<typename scalar>
class Joint;

template<typename scalar>
std::ostream& operator<<(std::ostream& os, const Pose<scalar>& pose)
{
  os << "pose absolute position: " << std::endl << pose.abs_p << std::endl;
  os << "Pose rotation matirx: " << std::endl << pose.rot << std::endl;
  return os;
}

template<typename scalar>
std::ostream& operator<<(std::ostream& os,const Joint<scalar>& joint)
{
  os << static_cast<const Pose<scalar>>(joint);
  os << "joint relative position: " << joint._rel_p << std::endl;
  os << "joint axis: " << joint._axis << std::endl;
  os << "joint angle: " << joint._angle << std::endl;
  return os;
}

template<typename scalar>
struct Pose
{
  using Vector3 = Eigen::Vector<scalar, 3>;
  using Matrix33 = Eigen::Matrix<scalar, 3, 3>;

  Pose(const Vector3& p = Vector3::Zero(), const Matrix33& r = Matrix33::Identity()) : abs_p(p), rot(r) {};

  //copy constructor
  Pose(const Pose& other) : abs_p(other.abs_p), rot(other.rot) {};

  // assign operator
  Pose& operator=(const Pose& rhs)
  {
    abs_p = rhs.abs_p;
    rot = rhs.rot;
    return *this;
  }

  // set absolute position and rotation matrix
  inline void SetPose(const Vector3& p, const Matrix33& r)
  {
    abs_p = p;
    rot = r;
  }
  inline void SetPosition(const Vector3& position) { abs_p = position; };
  inline void SetRotation(const Matrix33& rotation) { rot = rotation; };

  inline Vector3 position() { return abs_p; };
  inline Matrix33 rotation() { return rot; };

  bool operator==(const Pose& other) {
    return (abs_p.isApprox(other.bas_p)) && (rot.isApprox(other.rot));
  }
  bool operator!=(const Pose& other) {
    return !this->operator==(other);
  }
  friend std::ostream& operator<<<scalar>(std::ostream& c, const Pose<scalar>& pose);
protected:
  Vector3 abs_p;
  Matrix33 rot;
};

template<typename scalar>
class Joint : public Pose<scalar>
{
public:
  using typename Pose<scalar>::Vector3;
  using typename Pose<scalar>::Matrix33;

  // init a joint relative to a pose
  Joint(const Vector3& rel_p, const Vector3& axis, const scalar init_ang, const Pose<scalar>& parent_pose) : _rel_p(rel_p), _axis(axis), _angle(init_ang)
  {
    this->abs_p = parent_pose.rot * _rel_p + parent_pose.abs_p;
    this->rot = parent_pose.rot * Eigen::AngleAxis<scalar>(_angle, _axis); // axis rotate need transpose rotation matrix
  }

  //calculate absolute position using parent pose
  inline Vector3 CalPosition(const Pose<scalar>& parent_pose)
  {
    return parent_pose.abs_p + parent_pose * _rel_p;
  }

  //calculate rotation matrix and absolute position vector using joint angle.
  void Forward_kinematics(const scalar angle, const Pose<scalar>& parent_pose)
  {
    _angle = angle;
    this->abs_p = parent_pose.rot * _rel_p + parent_pose.abs_p;
    this->rot = parent_pose.rot * Eigen::AngleAxis<scalar>(_angle, _axis);
  }

  //calculate rotation matrix and absolute position
  void Forward_kinematics(const scalar angle, const Matrix33& parent_rotation, const Vector3& parent_abs_p)
  {
    _angle = angle;
    this->abs_p = parent_rotation * _rel_p + parent_abs_p;
    this->rot = parent_rotation * Eigen::AngleAxis<scalar>(_angle, _axis);
  }

  // inverse kenematics,calculate angle using rotation
  void Inverse_kinematics(const Matrix33& rotation, const Pose<scalar>& parent_pose)
  {
    this->rot = rotation;
    this->abs_p = parent_pose.rot * _rel_p + parent_pose.abs_p;

    Eigen::Quaternion<scalar> parent_q(parent_pose.rot);
    Eigen::Quaternion<scalar> joint_q(this->rotation);

    _angle = Eigen::AngleAxis<scalar>(parent_q.inverse() * joint_q).angle();
  }

  inline bool operator==(const Joint<scalar>& other) {
    return (Pose<scalar>::operator==(other)) && (_rel_p.isApprox(other._rel_p)) && (isEqual(_angle, other._angle));
  }
  inline bool operator!=(const Joint<scalar>&other) {
    return !(*this == other);
  }
  friend std::ostream& operator<<<scalar>(std::ostream& os, const Joint<scalar>& joint);
private:
  const Vector3 _rel_p; // relative position vector, relative to its parent joint, defined in parent coordinate. doesn't change. (no so sure)
  const Vector3 _axis;   // rotate axis, it's the joint rotation direction, defined in parent coordinate. doesn't change. (no so sure)
  scalar _angle; // joint angle.
};

template<typename scalar>
class LegsRobot
{
public:
  using Vector3 = Eigen::Vector<scalar, 3>;
  using Matrix33 = Eigen::Matrix<scalar, 3, 3>;

  constexpr static scalar hip_length = 0.06;  // hip length in meters
  constexpr static scalar thigh_length = 0.1; // thigh length in meters
  constexpr static scalar shank_length = 0.15; //shank length in meters

  const Vector3 body_joint_position{ 0,0,-0.05 };

  LegsRobot() : _leg_center(body_joint_position)
  {
    const Vector3 right_hip_rel_p{ 0,-hip_length,0 };
    const Vector3 left_hip_rel_p = -right_hip_rel_p;

    const Vector3 right_thigh_rel_p = { 0,0,-thigh_length };
    const Vector3 left_thigh_rel_p = right_thigh_rel_p;

    const Vector3 right_shank_rel_p = { 0,0,-shank_length };
    const Vector3 left_shank_rel_p = right_shank_rel_p;

    _right_leg.emplace_back(right_hip_rel_p, Vector3::UnitZ(), 0, _leg_center); // J2
    _left_leg.emplace_back(left_hip_rel_p, Vector3::UnitZ(), 0, _leg_center); // J8

    _right_leg.emplace_back(Vector3::Zero(), Vector3::UnitX(), 0, _right_leg.back()); // J3
    _left_leg.emplace_back(Vector3::Zero(), Vector3::UnitX(), 0, _left_leg.back()); // J9

    _right_leg.emplace_back(Vector3::Zero(), Vector3::UnitY(), 0, _right_leg.back()); // J4
    _left_leg.emplace_back(Vector3::Zero(), Vector3::UnitY(), 0, _left_leg.back()); // J10

    _right_leg.emplace_back(right_thigh_rel_p, Vector3::UnitY(), 0, _right_leg.back()); // J5
    _left_leg.emplace_back(left_thigh_rel_p, Vector3::UnitY(), 0, _left_leg.back()); // J11

    _right_leg.emplace_back(right_shank_rel_p, Vector3::UnitY(), 0, _right_leg.back()); // J6
    _left_leg.emplace_back(left_shank_rel_p, Vector3::UnitY(), 0, _left_leg.back());// J12

    _right_leg.emplace_back(Vector3::Zero(), Vector3::UnitX(), 0, _right_leg.back()); // J7
    _left_leg.emplace_back(Vector3::Zero(), Vector3::UnitX(), 0, _left_leg.back()); // J13
  }

  // calculate every joint abs_p and rotation matrix
  void Forward_kinematics(const Eigen::Vector<scalar, 12>& angle)
  {
    Forward_kinematics_leg(angle.template segment<6>(0), _right_leg);
    Forward_kinematics_leg(angle.template segment<6>(6), _left_leg);
  }

  // inverse kinematics,call Forward_kinematics to set every joint new angle and new rotation matrix and abs position
  inline void Inverse_kinematics(const Pose<scalar>& leg_center, const Pose<scalar>& right_last, const Pose<scalar>& left_last)
  {
    _leg_center = leg_center;
    Forward_kinematics(Inverse_kinematics_angle(right_last, left_last));
  }
  // inverse kenematic, use pose of leg_center and right and left last joint is enought
  // @return 12 angles
  Eigen::Vector<scalar,12> Inverse_kinematics_angle(const Pose<scalar>& right_last, const Pose<scalar>& left_last)
  {
    Eigen::Vector<scalar, 12> angle;
    angle.template segment<6>(0) = Inverse_kinematics_leg<true>(right_last);
    angle.template segment<6>(6) = Inverse_kinematics_leg<false>(left_last);
    return angle;
  }

private:
  void Forward_kinematics_leg(const Eigen::Vector<scalar, 6>& angle, std::list<Joint<scalar>>& leg)
  {
    assert(leg.size() == 6);
    auto it = leg.begin();
    auto it2 = leg.begin();

    it->Forward_kinematics(angle(0), _leg_center); //calculate first joint
    ++it;

    for (size_t i = 1;i < 6;++i)
    {
      it->Forward_kinematics(angle(i), *it2);
      ++it;
      ++it2;
    }
  }

  // inverse kenematics using analysis
  // using pose of leg center and last joint is enough
  template<bool is_right_leg>
  Eigen::Vector<scalar,6> Inverse_kinematics_leg(const Pose<scalar>& last)
  {
    Eigen::Vector<scalar, 6> angle;
    Vector3 p2;
    if constexpr (is_right_leg == 1) {
      p2 = _right_leg[0].CalPosition(_leg_center);
    }
    else {
      p2 = _left_leg[0].CalPosition(_leg_center);
    }
    Vector3 r = last.rotation().transpose() * (last.position() - p2);

    angle(3) = M_PI - std::acos((thigh_length * thigh_length + shank_length * shank_length - r.squaredNorm()) / (2 * thigh_length * shank_length));

    scalar angle_ahy = std::asin(thigh_length * std::sin(M_PI - angle(3)) / r.Norm());
    
    angle(5) = std::atan2(r.y(), r.z());

    angle(4) = -std::atan2(r.x(), r.y() * std::sin(angle(5)) + r.z() * std::cos(angle(5)));

    Matrix33 tmp =
      _leg_center.rotation().transpose() *
      last.rotation().transpose() *
      Eigen::AngleAxis<scalar>(angle(5), Vector3::UnitX()).toRotationMatrix().transpose() *
      Eigen::AngleAxis<scalar>(angle(3) + angle(4), Vector3::UnitY()).toRotationMatrix.transpose();

    angle(0) = std::atan2(-tmp(0, 0), tmp(1, 1));
    angle(1) = std::atan2(tmp(2, 1), -tmp(0, 1) * std::sin(angle(0)) + tmp(1, 1) * std::cos(angle(0)));
    angle(2) = std::atan2(-tmp(2, 0), tmp(2, 2));
  }

  bool operator==(const LegsRobot<scalar>& other)
  {
    return (_leg_center == other._leg_center) &&
      (_right_leg == other._right_leg) &&
      (_left_leg == other._left_leg);
  }
  inline bool operator!=(const LegsRobot<scalar>& other)
  {
    return !(*this == other);
  }
  Pose<scalar> _leg_center;   // center of legs, its parent is global coordinate.
  std::vector<Joint<scalar>> _right_leg;
  std::vector<Joint<scalar>> _left_leg;
};

