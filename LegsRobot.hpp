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
class LegsRobot;

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
  os << "joint relative position: " << std::endl << joint._rel_p << std::endl;
  os << "joint axis: " << std::endl << joint._axis << std::endl;
  os << "joint angle: " << std::endl << joint._angle << std::endl;
  return os;
}

template<typename scalar>
std::ostream& operator<<(std::ostream& os, const LegsRobot<scalar>& legsrobot)
{
  os << "Leg Robot leg center:" << std::endl << legsrobot._leg_center << std::endl;
  size_t count = 0;
  for (auto it = legsrobot._right_leg.cbegin(), end = legsrobot._right_leg.cend();it != end;++it) {
    os << "right leg " << ++count << " joint :" << std::endl << *it;
  }
  count = 0;
  for (auto it = legsrobot._left_leg.cbegin(), end = legsrobot._left_leg.cend();it != end;++it) {
    os << "left leg " << ++count << " joint :" << std::endl << *it;
  }
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

  inline Vector3 position() const { return abs_p; };

  inline Matrix33 rotation() const { return rot; };

  bool operator==(const Pose& other) const{
    return (abs_p.isApprox(other.abs_p,1e-5)) && (rot.isApprox(other.rot,1e-5));
  }
  bool operator!=(const Pose& other) const{
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
    const Vector3 par_pos = parent_pose.position();
    const Matrix33 par_rot = parent_pose.rotation();
    this->abs_p = par_rot * _rel_p + par_pos;
    this->rot = par_rot * Eigen::AngleAxis<scalar>(_angle, _axis); // axis rotate need transpose rotation matrix
  }

  //copy constructor
  Joint(const Joint& other) : Pose<scalar>(other), _rel_p(other._rel_p), _axis(other._axis), _angle(other._angle){};

  //assignment operator
  Joint& operator=(const Joint& other)
  {
    Pose<scalar>::operator=(other);
    _rel_p = other._rel_p;
    _axis = other._axis;
    _angle = other._angle;
    return *this;
  }

  //calculate absolute position using parent pose
  inline Vector3 calPosition(const Pose<scalar>& parent_pose)
  {
    return parent_pose.position() + parent_pose.rotation() * _rel_p;
  }
  inline Vector3 calPosition(const Matrix33& parent_rotation, const Vector3& parent_abs_position)
  {
    return parent_abs_position + parent_rotation * _rel_p;
  }

  inline Matrix33 calRotation(const Pose<scalar>& parent_pose)
  {
    return parent_pose.rotation() * Eigen::AngleAxis<scalar>(_angle, _axis);
  }
  inline Matrix33 calRotation(const Matrix33& parent_rotation)
  {
    return parent_rotation * Eigen::AngleAxis<scalar>(_angle, _axis);
  }

  //calculate rotation matrix and absolute position vector using joint angle.
  void Forward_kinematics(const scalar angle, const Pose<scalar>& parent_pose)
  {
    _angle = angle;
    this->abs_p = calPosition(parent_pose);
    this->rot = calRotation(parent_pose);
  }

  //calculate rotation matrix and absolute position
  void Forward_kinematics(const scalar angle, const Matrix33& parent_rotation, const Vector3& parent_abs_p)
  {
    _angle = angle;
    this->abs_p = calPosition(parent_rotation, parent_abs_p);
    this->rot = calRotation(parent_rotation);
  }

  // inverse kenematics,calculate angle using rotation
  void Inverse_kinematics(const Matrix33& rotation, const Pose<scalar>& parent_pose)
  {
    const Matrix33 p_rotation = parent_pose.rotation();
    const Vector3 p_abs_pos = parent_pose.position();
    this->rot = rotation;
    this->abs_p = p_rotation * _rel_p + p_abs_pos;

    Eigen::Quaternion<scalar> parent_q(p_rotation);
    Eigen::Quaternion<scalar> joint_q(rotation);

    _angle = Eigen::AngleAxis<scalar>(parent_q.inverse() * joint_q).angle();
  }

  inline bool operator==(const Joint<scalar>& other) const {
    return (Pose<scalar>::operator==(other)) && (_rel_p.isApprox(other._rel_p,1e-5)) && (isEqual(_angle, other._angle));
  }
  inline bool operator!=(const Joint<scalar>&other) const {
    return !(*this == other);
  }
  inline scalar angle() const { return _angle; };
  inline Vector3 relposition() const { return _rel_p; };
  inline Vector3 axis() const { return _axis; };
  
  friend std::ostream& operator<<<scalar>(std::ostream& os, const Joint<scalar>& joint);
private:
  Vector3 _rel_p; // relative position vector, relative to its parent joint, defined in parent coordinate. doesn't change. (no so sure)
  Vector3 _axis;   // rotate axis, it's the joint rotation direction, defined in parent coordinate. doesn't change. (no so sure)
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

  //copy constuctor
  LegsRobot(const LegsRobot& other) = default;

  // assignment operator
  LegsRobot& operator=(const LegsRobot& rhs) {
    _leg_center = rhs._leg_center;
    _right_leg = rhs._right_leg;
    _left_leg = rhs._left_leg;
  }

  //get leg center
  inline Pose<scalar> legCenter() const { return _leg_center; };
  inline const std::vector<Joint<scalar>>& rightLeg() const { return _right_leg; };
  inline const std::vector<Joint<scalar>>& leftLeg() const { return _left_leg; };
  
  bool operator==(const LegsRobot<scalar>& other) const 
  {
    return (_leg_center == other._leg_center) &&
      (_right_leg == other._right_leg) &&
      (_left_leg == other._left_leg);
  }
  inline bool operator!=(const LegsRobot<scalar>& other) const 
  {
    return !(*this == other);
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
    angle.template segment<6>(0) = Inverse_kinematics_leg<LEG::RIGHT>(right_last);
    angle.template segment<6>(6) = Inverse_kinematics_leg<LEG::LEFT>(left_last);
    return angle;
  }

  friend std::ostream& operator<<<scalar>(std::ostream& os, const LegsRobot<scalar>& robot);
private:
  enum LEG {
    RIGHT,
    LEFT
  };
  void Forward_kinematics_leg(const Eigen::Vector<scalar, 6>& angle, std::vector<Joint<scalar>>& leg)
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
  template<LEG leg>
  Eigen::Vector<scalar,6> Inverse_kinematics_leg(const Pose<scalar>& last)
  {
    Eigen::Vector<scalar, 6> angle;
    Vector3 p2;
    if constexpr (leg == RIGHT) {
      p2 = _right_leg[0].calPosition(_leg_center);
    }
    else {
      p2 = _left_leg[0].calPosition(_leg_center);
    }
    Vector3 r = last.rotation().transpose() * (p2 - last.position());

    scalar sca_tmp = (thigh_length * thigh_length + shank_length * shank_length - r.squaredNorm()) / (2 * thigh_length * shank_length);
    sca_tmp =  LimitTo<double>(sca_tmp,-1,1);
    angle(3) = M_PI - std::acos(sca_tmp);

    sca_tmp = thigh_length * std::sin(M_PI - angle(3)) / r.norm();
    sca_tmp = LimitTo<double>(sca_tmp,-1,1);
    scalar angle_ahy = std::asin(sca_tmp);
    
    angle(5) = std::atan2(r.y(), r.z());

    angle(4) = -std::atan2(r.x(), r.y() * std::sin(angle(5)) + r.z() * std::cos(angle(5)))-angle_ahy;

    Matrix33 tmp =
      _leg_center.rotation().transpose() *
      last.rotation() *
      Eigen::AngleAxis<scalar>(angle(5), Vector3::UnitX()).toRotationMatrix().transpose() *
      Eigen::AngleAxis<scalar>(angle(3) + angle(4), Vector3::UnitY()).toRotationMatrix().transpose();

    angle(0) = std::atan2(-tmp(0, 1), tmp(1, 1));
    angle(1) = std::atan2(tmp(2, 1), -tmp(0, 1) * std::sin(angle(0)) + tmp(1, 1) * std::cos(angle(0)));
    angle(2) = std::atan2(-tmp(2, 0), tmp(2, 2));

    return angle;
  }
  Pose<scalar> _leg_center;   // center of legs, its parent is global coordinate.
  std::vector<Joint<scalar>> _right_leg;
  std::vector<Joint<scalar>> _left_leg;
};

