#include <limits>
#include <math.h>

template<typename scalar>
bool isEqual(scalar lhs, scalar rhs)
{
  return (std::abs<scalar>(lhs - rhs) <= std::numeric_limits<scalar>::epsilon());
}