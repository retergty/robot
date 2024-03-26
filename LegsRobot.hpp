#include "Dense"
#include "Geometry"
#include "../unsupported/Eigen/MatrixFunctions"
#include <list>

template<typename scalar>
struct Pose
{
  using Vector3 = Eigen::Vector<scalar, 3>;
  using Matrix33 = Eigen::Matrix<scalar, 3, 3>;

  Pose(const Vector3& p = Vector3::Zero(), const Matrix33& r = Matrix33::Identity()) : abs_p(p), rotation(r) {};
  // set absolute position and rotation matrix
  inline void SetPose(const Vector3& p, const Matrix33& r)
  {
    abs_p = p;
    rotation = r;
  }

  Vector3 abs_p;
  Matrix33 rotation;
};

template<typename scalar>
class Joint : public Pose<scalar>
{
public:
  //using Vector3 = Eigen::Vector<scalar, 3>;
  //using Matrix33 = Eigen::Matrix<scalar, 3, 3>;
  using typename Pose<scalar>::Vector3;
  using typename Pose<scalar>::Matrix33;

  // init a joint relative to a pose
  Joint(const Vector3& rel_p, const Vector3& axis, const scalar init_ang, const Pose<scalar>& parent_pose) : _rel_p(rel_p), _axis(axis), _angle(init_ang)
  {
    this->abs_p = parent_pose.rotation * _rel_p + parent_pose.abs_p;
    this->rotation = parent_pose.rotation * Eigen::AngleAxis<scalar>(_angle, _axis); // axis rotate need transpose rotation matrix
  }

  //calculate rotation matrix and absolute position vector using joint angle.
  void Forward_kinematics(const scalar angle, const Pose<scalar>& parent_pose)
  {
    _angle = angle;
    this->abs_p = parent_pose.rotation * _rel_p + parent_pose.abs_p;
    this->rotation = parent_pose.rotation * Eigen::AngleAxis<scalar>(_angle, _axis);
  }

  //calculate rotation matrix and absolute position
  void Forward_kinematics(const scalar angle, const Matrix33& parent_rotation, const Vector3& parent_abs_p)
  {
    _angle = angle;
    this->abs_p = parent_rotation * _rel_p + parent_abs_p;
    this->rotation = parent_rotation * Eigen::AngleAxis<scalar>(_angle, _axis);
  }

  // inverse kenematics,calculate angle using rotation
  void Inverse_kinematics(const Matrix33& rotation, const Pose<scalar>& parent_pose)
  {
    this->rotation = rotation;
    this->abs_p = parent_pose.rotation * _rel_p + parent_pose.abs_p;

    Eigen::Quaternion<scalar> parent_q(parent_pose.rotation);
    Eigen::Quaternion<scalar> joint_q(this->rotation);

    _angle = Eigen::AngleAxis<scalar>(parent_q.inverse() * joint_q).angle();
  }

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
    Forward_kinematics_legs(angle.template segment<6>(0), _right_leg);
    Forward_kinematics_legs(angle.template segment<6>(6), _left_leg);
  }

private:
  void Forward_kinematics_legs(const Eigen::Vector<scalar, 6>& angle, std::list<Joint<scalar>>& leg)
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

  Pose<scalar> _leg_center;   // center of legs, its parent is global coordinate.
  std::list<Joint<scalar>> _right_leg;
  std::list<Joint<scalar>> _left_leg;
};

