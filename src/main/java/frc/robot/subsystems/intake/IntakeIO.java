package frc.robot.subsystems.intake;

import frc.robot.util.VTControlType;
import org.littletonrobotics.junction.AutoLog;

/**
 * Interface for arm Module IO implementations. This abstracts all hardware interactions for the
 * arm.
 */

/*
units:
radian
radians
seconds
volts
amperes
 */
public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    double rollerAmps = 0;
    double rollerVolts = 0;
    double rollerSpeed = 0;

    double targetSpeed = 0;

    double position = 0;
    double targetPosition = 0;

    String controlType = "none";
  }

  // updates the given inputs with new values(advantage kit stuff)
  public default void updateInputs(IntakeIOInputsAutoLogged inputsAutoLogged) {}

  // getters for motors

  // gets the height of the arm in meters
  public default double getCurrent() {
    return 0;
  }

  public default double getVoltage() {
    return 0;
  }

  // setters for motors
  public default void setVoltage(double volt) {}

  // sets the position of the rollers. This function will most likely not be implemented
  public default void setTargetPosition(double position) {}

  public default double getTargetPosition() {
    return 0;
  }
  // misc methods

  // rebuilds the pid constants of the motors
  public default void rebuildMotorsPID() {}

  /** Stops the motor immediately */
  public default void stop() {}
  ;

  public default void resetEncoders() {}

  public default void setBraked(boolean braked) {}

  // gets the highest possible height of the arm in radians
  public default double getMaxPosition() {
    return 0;
  }

  // gets the height of the arm in meters
  public default double getPosition() {
    return 0;
  }

  public default boolean isMaxPosition() {
    return false;
  }

  public default void setTargetSpeed(double speed) {}

  public default double getSpeed() {
    return 0;
  }

  public default double getTargetSpeed() {
    return 0;
  }

  public default boolean checkIfStalled() {
    return false;
  }

  public default VTControlType getControlType() {
    return VTControlType.MANUAL;
  }
}
