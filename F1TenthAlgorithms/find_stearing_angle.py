import numpy as np

# ---------------------------------------------------------------------------- #
# Enumerator with different controller types
# ---------------------------------------------------------------------------- #

# TODO: Add a enumerator which can be used to choose which controller to use

# ---------------------------------------------------------------------------- #
# Public Functions
# ---------------------------------------------------------------------------- #

# TODO: Add a top level function which will give steering control output

# ---------------------------------------------------------------------------- #
# Private Functions
# ---------------------------------------------------------------------------- #

# TODO: Add a function to find steering angle based on both sgn and propotional

# TODO: Add a function to find the steering angle based on sgn function


def _proportional_controller(desieredStearingAngle_rad: float,
                             actualHeadingAngle_rad: float,
                             proportionalController: float = 1,
                             absMaxStearingAngle_rad: float = np.pi / 4) -> float:
  '''
  @brief        Function which will find the stearing angle based on a
                proportional controller.

  @args
    desieredStearingAngle_rad
      Desiered stearing angle of the robot.

    actualHeadingAngle_rad
      Actual stearing angle of the robot.

  @kwargs
    proportionalController = 1
      Proportional coefficient of controller.

    absMaxStearingAngle_rad = np.pi / 4
      Absolute value of max stearing angle. It is assumed that the stearing
      angle range is symmetrical.

  @return
    desieredSearingAngle_rad
      Desiered stearig angle in radians of the robot.
  '''

  # Find stearing angle
  desieredSearingAngle_rad = proportionalController * (desieredStearingAngle_rad -
                                                       actualHeadingAngle_rad)

  # Check that the max stearing angle is within the acceptable range
  if desieredStearingAngle_rad < -absMaxStearingAngle_rad:
    desieredStearingAngle_rad = -absMaxStearingAngle_rad
  elif desieredStearingAngle_rad > absMaxStearingAngle_rad:
    desieredStearingAngle_rad = absMaxStearingAngle_rad

  return desieredSearingAngle_rad


def _find_desiered_heading(currentXPosition_fix_m: float,
                           currentYPosition_fix_m: float,
                           desieredXPosition_fix_m: float,
                           desieredYPosition_fix_m: float) -> float:
  '''
  @brief        Function which will find the desiered heading angle based on the
                current position of the robot and the desiered position of the
                robot.

  @args
    currentXPosition_fix_m
      Current x position of the robot

    currentYPosition_fix_m
      Current y position of the robot

    desieredXPosition_fix_m
      Desiered x position of the robot

    desieredYPosition_fix_m
      Desiered y position of the robot

  @kwargs
    - None

  @return
    desieredHeading_rad
      Desiered heading angle in radians 
  '''

  # Find the desiered angle in radians
  desieredHeading_rad = - np.atan2(desieredXPosition_fix_m - currentXPosition_fix_m,
                                   desieredYPosition_fix_m - currentYPosition_fix_m)

  return desieredHeading_rad


if __name__ == '__main__':
  pass
