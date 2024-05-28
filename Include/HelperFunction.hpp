#pragma once
#include <limits>
#include <math.h>
#include "Core"

template<typename scalar>
bool isEqual(scalar lhs, scalar rhs, double presision = 1e-5)
{
  return (std::abs<scalar>(lhs - rhs) <= presision);
}
template<typename scalar, int M, int N>
bool isMatrixEqual(Eigen::Matrix<scalar, M, N> lhs, Eigen::Matrix<scalar, M, N> rhs, double presision = 1e-5)
{
  for (int j = 0;j < N;++j) {
    for (int i = 0;i < M;++i) {
      if (!isEqual(lhs(i, j), rhs(i, j), presision))
        return false;
    }
  }
  return true;
}
template<typename Type>
Type LimitTo(const Type t, Type min, Type max)
{
  return std::min(std::max(t, min), max);
}

template<typename scalar>
constexpr scalar Deg2Rad = M_PI / 180.0;

template<typename scalar>
constexpr scalar Rad2Deg = 180.0 / M_PI;

template<typename T, typename D = int>
constexpr bool has_clear = false;

template<typename T>
constexpr bool has_clear<T, decltype(std::declval<T>().clear(), int())> = true;

template<typename T, typename D = void>
constexpr bool has_const_iterator = false;

template<typename T>
constexpr bool has_const_iterator<T, std::void_t<typename T::const_iterator>> = true;

template<typename Type>
struct xy_dim
{
  xy_dim() = default;
  xy_dim(const Type& in_x, const Type& in_y) :x(in_x), y(in_y) {};
  void clear() {
    if constexpr (has_clear<Type>) {
      x.clear();
      y.clear();
    }
    else {
      x = 0;
      y = 0;
    }
  }
  // Type is container
  template<typename T = Type, std::enable_if_t<has_const_iterator<T>, bool> = true>
  xy_dim<typename T::value_type> operator[](size_t index) const {
    return xy_dim<typename T::value_type>(x[index], y[index]);
  }
  // Type is not container
  template<typename T = Type, std::enable_if_t<!has_const_iterator<T>, bool> = true>
  xy_dim<T> operator[](size_t index) const {
    return xy_dim<T>(x, y);
  }
  Type x;
  Type y;
};

template <unsigned int p,typename scalar = double>
constexpr scalar IntPower(const scalar x)
{
  if constexpr (p == 0) return 1;
  if constexpr (p == 1) return x;

  scalar tmp = IntPower<p / 2>(x);
  if constexpr ((p % 2) == 0) { return tmp * tmp; }
  else { return x * tmp * tmp; }
}
