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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.communication.ControllerVibrateCommand;
import frc.robot.commands.communication.TellCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmSimulationIO;
import frc.robot.subsystems.canrange.RangeFinder;
import frc.robot.subsystems.canrange.RangeFinderIO;
import frc.robot.subsystems.canrange.RangeFinderSimulationIO;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSimulation;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public final Arm arm;
  public final RangeFinder canrange;
  public final Intake intake;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        arm = new Arm(new ArmIO() {});

        canrange = new RangeFinder(new RangeFinderIO() {});
        intake = new Intake(new IntakeIO() {}, canrange);

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        arm = new Arm(new ArmSimulationIO());

        canrange = new RangeFinder(new RangeFinderSimulationIO());
        intake = new Intake(new IntakeIOSimulation(), canrange);

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        arm = new Arm(new ArmIO() {});

        canrange = new RangeFinder(new RangeFinderIO() {});
        intake = new Intake(new IntakeIO() {}, canrange);

        break;
    }

    registerNamedCommandsAuto();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();

    arm.setDefaultCommand(
        Commands.either(
            arm.setTargetHeightCommand(Constants.ARM_INTAKE_ANGLE),
            arm.setTargetHeightCommand(Constants.ARM_SCORING_ANGLE),
            () -> canrange.getCanDistance() > Constants.CANRANGE_DETECTION_DISTANCE));

    intake.setDefaultCommand(
        Commands.either(
                intake.setTargetSpeedCommand(0),
                intake.setTargetSpeedCommand(Constants.HOLDING_SPEED),
                () -> canrange.getCanDistance() > Constants.CANRANGE_DETECTION_DISTANCE)
            .andThen(new TellCommand("Default command")));

    // if the canrange doesn't see anything set rollers to intake speed
    Command intakeCommand =
        // set the arm height to the floor
        arm.setTargetHeightCommand(Constants.ARM_INTAKE_ANGLE)
            .alongWith(
                // intakes
                intake
                    .setTargetSpeedCommand(Constants.INTAKE_SPEED)
                    // when it notices a coral inside it vibrates the controller
                    .until(() -> canrange.getCanDistance() < Constants.CANRANGE_DETECTION_DISTANCE)
                    .andThen(intake.instantSetTargetSpeedCommand(Constants.HOLDING_SPEED))
                    .andThen(
                        new ControllerVibrateCommand(
                            Constants.CONTROLLER_FEEDBACK_AMOUNT, controller)));

    Command scoringCommand =
        arm.setTargetHeightCommandConsistentEnd(Constants.ARM_SCORING_ANGLE)
            .andThen(intake.setTargetSpeedCommand(Constants.EJECT_SPEED))
            .alongWith(
                new WaitCommand(Constants.CORAL_RELEASE_TIME)
                    .andThen(new ControllerVibrateCommand(0.2, controller)));

    controller.leftTrigger().whileTrue(intakeCommand);

    controller.rightTrigger().whileTrue(scoringCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
  }

  public void registerNamedCommandsAuto() {

    boolean isReal = false;

    addNamedCommand(
        "CoralScore",
        arm.setTargetHeightCommandConsistentEnd(Constants.ARM_SCORING_ANGLE)
            .andThen(
                intake
                    .setTargetSpeedCommand(Constants.EJECT_SPEED)
                    .withDeadline(new WaitCommand(Constants.CORAL_RELEASE_TIME))),
        isReal);
    addNamedCommand(
        "PrepCoralScore", arm.setTargetHeightCommand(Constants.ARM_SCORING_ANGLE), isReal);
    addNamedCommand(
        "PrepCoralIntake", arm.setTargetHeightCommand(Constants.ARM_INTAKE_ANGLE), isReal);
    addNamedCommand(
        "CoralIntake",
        arm.setTargetHeightCommand(Constants.ARM_INTAKE_ANGLE)
            .andThen(
                intake
                    .setTargetSpeedCommand(Constants.INTAKE_SPEED)
                    .until(
                        () -> canrange.getCanDistance() < Constants.CANRANGE_DETECTION_DISTANCE)),
        isReal);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /**
   * function to add named commands because we need to add is an an event too and not just as a
   * command. This also handles simulation logging
   */
  public void addNamedCommand(String commandName, Command command, boolean isReal) {

    if (isReal) {
      NamedCommands.registerCommand(
          commandName, command.andThen(new TellCommand("just ran " + commandName)));
          
        new EventTrigger(commandName).onTrue(command);
    } else {
      // registers the named commands to print something out instead of actually running anything
      NamedCommands.registerCommand(
          commandName,
          new TellCommand(commandName + " auto command")
              .andThen(
                  new ControllerVibrateCommand(1, controller).withDeadline(new WaitCommand(0.2)))
              .alongWith(command));

                new EventTrigger(commandName)
            .onTrue(
                new TellCommand(commandName + " auto event trigger command")
                    .andThen(
                        new ControllerVibrateCommand(1, controller)
                            .withDeadline(new WaitCommand(0.2)))
                    .andThen(new WaitCommand(0.3)));
    }
  }

  public Arm getArm() {
    return arm;
  }

  public Intake getIntake() {
    return intake;
  }

  public RangeFinder getCanrange() {
    return canrange;
  }
}
