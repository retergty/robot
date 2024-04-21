#include <limits>
#include <math.h>
#include "Core"

template<typename scalar>
bool isEqual(scalar lhs, scalar rhs,scalar presision = 1e-5)
{
  return (std::abs<scalar>(lhs - rhs) <= presision);
}

template<typename Type>
Type LimitTo(const Type t,Type min,Type max)
{
  return std::min(std::max(t, min), max);
}