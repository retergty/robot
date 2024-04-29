#pragma once
namespace param
{
  constexpr double HIP_LENGTH = 0.030;
  constexpr double THIGH_LENGTH = 0.065;
  constexpr double SHANK_LENGTH = 0.1;
  constexpr double LEG_CENTER_RELATIVE_TO_COM_Z = -0.035;

  constexpr double LEG_CENTER_RELATIVE_TO_COM_X = 0;
  constexpr double LEG_CENTER_RELATIVE_TO_COM_Y = 0;
  constexpr double COM_X = 0;
  constexpr double COM_Y = 0;
  constexpr double COM_Z = SHANK_LENGTH + THIGH_LENGTH - LEG_CENTER_RELATIVE_TO_COM_Z;

  constexpr double STEP_TIME = 2;
  constexpr double START_SCALAR = 0.4;
  constexpr double DBL_SCALAR = 0.3;
  constexpr double REST_SCALAR = 0.3;

  constexpr double STEP_LENGTH = 0.08;
  constexpr double STEP_WIDTH = HIP_LENGTH*2;
  constexpr double STEP_HEIGHT = 0;

  constexpr double STEP_Z_PEEK = 0.035;
} // namespace robot
