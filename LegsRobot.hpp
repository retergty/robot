#include "Dense"
#include "Geometry"
#include "../unsupported/Eigen/MatrixFunctions"
#include <list>
template<typename scalar>
class Joint
{
public:
  using Vector3 = Eigen::Vector<scalar, 3>;
  using Matrix33 = Eigen::Matrix<scalar, 3, 3>;

  // init a joint relatively
  Joint(const Vector3& rel_p, const Vector3& axis, const scalar init_ang, const Joint& parent_joint) : _rel_p(rel_p), _axis(axis), _angle(init_ang)
  {
    _abs_p = parent_joint._rotation * _rel_p + parent_joint._abs_p;
    _rotation = parent_joint._rotation * Eigen::AngleAxis<scalar>(_angle, _axis);
  }

  // init a default joint,its parent is global coordinate
  Joint(const Vector3& abs_p, const Vector3& axis, const scalar init_ang, const Matrix33& global_rotation = Matrix33::Identity()) :_rel_p(abs_p), _axis(axis),  _abs_p(abs_p), _angle(init_ang)
  {
    _rotation = global_rotation * Eigen::AngleAxis<scalar>(_angle, _axis);
  }

  //calculate rotation matrix and absolute position vector using joint angle.
  void Forward_kinematics(const scalar angle, const Joint& parent_joint)
  {
    _angle = angle;
    _abs_p = parent_joint._rotation * _rel_p + parent_joint._abs_p;
    _rotation = parent_joint._rotation * Eigen::AngleAxis<scalar>(_angle, _axis);
  }

private:
  const Vector3 _rel_p; // relative position vector, relative to its parent joint, defined in parent coordinate. doesn't change. (no so sure)
  const Vector3 _axis;   // rotate axis, it's the joint rotation direction, defined in parent coordinate. doesn't change. (no so sure)
  Vector3 _abs_p; // absolute position vector, defined in global coordinate.
  Matrix33 _rotation; // rotation matrix. rotate vector in the joint coordinate to the global coordinate.
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

  LegsRobot() : _body_joint(body_joint_position, Vector3::UnitX(), 0)
  {
    const Vector3 right_hip_rel_p{ 0,-hip_length,0 };
    const Vector3 left_hip_rel_p = -right_hip_rel_p;

    const Vector3 right_thigh_rel_p = { 0,0,-thigh_length };
    const Vector3 left_thigh_rel_p = right_thigh_rel_p;

    const Vector3 right_shank_rel_p = { 0,0,-shank_length };
    const Vector3 left_shank_rel_p = right_shank_rel_p;

    _right_leg.emplace_back(right_hip_rel_p, Vector3::UnitZ(), 0, _body_joint); // J2
    _left_leg.emplace_back(left_hip_rel_p, Vector3::UnitZ(), 0, _body_joint); // J8

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
    Forward_kinematics_legs(angle.segment<6>(0), _right_leg);
    Forward_kinematics_legs(angle.segment<6>(6), _left_leg);
  }

private:
  void Forward_kinematics_legs(const Eigen::Vector<scalar, 6>& angle,std::list<Joint<scalar>>& leg)
  {
    assert(leg.size() == 6);
    auto it = leg.begin();
    auto it2 = leg.begin();

    it->Forward_kinematics(angle(0), _body_joint); //calculate first joint
    ++it;

    for (size_t i = 1;i < 6;++i)
    {
      it->Forward_kinematics(angle(i), *it2);
      ++it;
      ++it2;
    }
  }

  Joint<scalar> _body_joint;   // FAKE joint, center of legs, represented the first joint, its parent is global coordinate.
  std::list<Joint<scalar>> _right_leg;
  std::list<Joint<scalar>> _left_leg;
};

