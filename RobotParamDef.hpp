#pragma once
namespace param
{
  constexpr float HIP_LENGTH = 0.1;
  constexpr float THIGH_LENGTH = 0.08;
  constexpr float SHANK_LENGTH = 0.1;
  constexpr float LEG_CENTER_RELATIVE_TO_COM_Z = -0.05;

  constexpr float LEG_CENTER_RELATIVE_TO_COM_X = 0;
  constexpr float LEG_CENTER_RELATIVE_TO_COM_Y = 0;
  constexpr float COM_X = 0;
  constexpr float COM_Y = 0;
  constexpr float COM_Z = SHANK_LENGTH + THIGH_LENGTH - LEG_CENTER_RELATIVE_TO_COM_Z;

  constexpr float STEP_TIME = 2;
  constexpr float START_SCALAR = 0.4;
  constexpr float DBL_SCALAR = 0.3;
  constexpr float REST_SCALAR = 0.3;

  constexpr float STEP_LENGTH = 0.2;
  constexpr float STEP_WIDTH = 0.2;

} // namespace robot
