#include "Dense"
#include "Geometry"

template<typename scalar>
class Joint
{
public:
  Joint();
private:
  Eigen::Vector<scalar,3> _rel_p; // relative position vector, relative to its parent joint, defined in parent coordinate.
  Eigen::Vector<scalar,3> _abs_p; // absolute position vector, defined in global coordinate.
  Eigen::Vector<scalar,3> _axis;   // rotate axis, it's the joint rotation direction, defined in parent coordinate.
  Eigen::Matrix<scalar,3,3> _rotation; // rotation matrix. rotate joint coordinate vector to global coordinate.
  scalar _angle; // joint angle.
};

class LegsRobot
{
private:

public:
};

