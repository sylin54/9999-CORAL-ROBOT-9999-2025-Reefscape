package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

/* 
public class IntakeIOMotor implements IntakeIO {
  private TalonFX intakemotor = new TalonFX(0);

  public void SetMotorSpeed(double Speed) {
    intakemotor.set(Speed);
  }

  public void updateInputs(IntakeIOInputsAutoLogged inputs) {
    inputs.CurrentMotorSpeed = intakemotor.get();
    inputs.CurrentMotorVoltage = intakemotor.getMotorVoltage(true).getValueAsDouble();
    inputs.CurrentMotorAmps = intakemotor.getStatorCurrent().getValueAsDouble();
  }
}
  */
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.util.VTControlType;

public class IntakeIOMotor implements IntakeIO {

  private PIDController controller;

  protected double tolerance = 0.1;
  protected double targetPosition = 0;
  protected double maxPosition = 0.0;
  protected double targetSpeed = 0;

  private VTControlType controlType = VTControlType.MANUAL;

  private TalonFX rollers;

  public IntakeIOMotor(int rollerID, String canbusName) {

    rollers = new TalonFX(rollerID, canbusName);

    rebuildMotorsPID();

    TalonFXConfiguration rollerMotorConfigs =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  // Swerve azimuth does not require much torque output, so we can set a
                  // relatively
                  // low
                  // stator current limit to help avoid brownouts without impacting performance.
                  .withStatorCurrentLimit(Amps.of(Constants.INTAKE_CURRENT_LIMIT))
                  .withStatorCurrentLimitEnable(true));
    rollers.getConfigurator().apply(rollerMotorConfigs);

    // Set motor to Brake mode by default.
    rollers.setNeutralMode(NeutralModeValue.Brake);
  }

  // updates the given inputs with new values(advantage kit stuff)
  @Override
  public void updateInputs(IntakeIOInputsAutoLogged inputsAutoLogged) {

    if (controlType == VTControlType.POSITION_PID) updatePID();

    inputsAutoLogged.rollerAmps = motorSim.getCurrentDrawAmps();
    inputsAutoLogged.rollerVolts = motorSim.getInputVoltage();
    inputsAutoLogged.position = motorSim.getAngularPositionRad();
    inputsAutoLogged.rollerSpeed = motorSim.getAngularVelocityRadPerSec();
    inputsAutoLogged.targetSpeed = targetSpeed;
    inputsAutoLogged.targetPosition = targetPosition;

    inputsAutoLogged.controlType = controlType.name();
  }

  // getters for motors

  // gets the height of the arm in meters
  @Override
  public double getCurrent() {
    return motorSim.getCurrentDrawAmps();
  }

  @Override
  public double getVoltage() {
    return motorSim.getInputVoltage();
  }

  // setters for motors
  @Override
  public void setVoltage(double volt) {
    controlType = VTControlType.MANUAL;

    motorSim.setInputVoltage(MathUtil.clamp(volt, -12, 12));
  }

  // sets the position of the rollers. This function will most likely not be implemented
  @Override
  public void setTargetPosition(double position) {
    controlType = VTControlType.POSITION_PID;

    targetPosition = position;
  }

  @Override
  public double getTargetPosition() {
    return targetPosition;
  }

  // misc methods

  // rebuilds the pid constants of the motors
  @Override
  public void rebuildMotorsPID() {
    controller = new PIDController(0.9, 0, 0.1);
  }

  // Stops the motor immediately
  @Override
  public void stop() {
    setVoltage(0);
  }
  ;

  @Override
  public void resetEncoders() {
    motorSim.setAngle(0);
  }

  @Override
  public void setBraked(boolean braked) {}

  // gets the highest possible height of the arm in radians
  @Override
  public double getMaxPosition() {
    return maxPosition;
  }

  // gets the height of the arm in meters
  @Override
  public double getPosition() {
    return motorSim.getAngularPositionRotations();
  }

  @Override
  public boolean isMaxPosition() {
    return getMaxPosition() - getPosition() < tolerance;
  }

  @Override
  public void setTargetSpeed(double speed) {
    controlType = VTControlType.SPEED_PID;
    targetSpeed = speed;

    motorSim.setInputVoltage(speed * Math.PI * 2);
  }

  @Override
  public double getTargetSpeed() {
    return motorSim.getAngularVelocityRadPerSec() * Math.PI * 2;
  }

  @Override
  public double getSpeed() {
    return motorSim.getInputVoltage();
  }

  protected void updatePID() {
    double currentAngle = motorSim.getAngularPositionRotations();
    double inputVoltage = controller.calculate(currentAngle, targetPosition);
    // System.out.println("Input volt: "+inputVoltage+" Target Angle: "+targetAngle);x
    setVoltage(inputVoltage);
  }

  @Override
  public VTControlType getControlType() {
    return controlType;
  }
}

/*
todolist:
-standardize io layers

 */