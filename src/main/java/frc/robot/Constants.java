// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final double ARM_SCORING_ANGLE = 0;
  public static final double ARM_INTAKE_ANGLE = 4;

  public static final double CORAL_RELEASE_TIME = 1;

  public static final double INTAKE_SPEED = -1;
  public static final double EJECT_SPEED = 1;
  public static final double HOLDING_SPEED = -0.02;

  public static final double FREE_CANRANGE_DIST = 1;
  
  public static final double CONTROLLER_FEEDBACK_AMOUNT = 0.2;


  public static final double CANRANGE_DETECTION_DISTANCE = 1;

  public static boolean isFlipped() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public class CArm {
    public static final double TOLERANCE = 0.1;
    public static final double MANUAL_MAX_SPEED = 0.1;
    public static final double MIN_HEIGHT = 0;
    public static final double MAX_HEIGHT = 5;
    public static final double CLEARANCE_ANGLE = 0.1;
  }

  public class CIntake {
    // dummy values for now
    public static final double MAX_TARGET_SPEED = 100;
    public static final double MAX_MANUAL_SPEED = 100;
    public static final double SPEED_TOLERANCE = 0.2;
  }

  public class CDrivetrain {
    public static PathConstraints DEFAULT_PATH_CONSTRAINTS =
        new PathConstraints(3.5, 4, Math.PI * 3.5, Math.PI * 4);

    public static double TOLERANCE_DIST = 0.1;
    public static double TOLERANCE_ROT = 0.1;
  }
}
