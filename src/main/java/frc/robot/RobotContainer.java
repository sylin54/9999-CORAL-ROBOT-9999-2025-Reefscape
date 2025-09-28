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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
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

    // SmartDashboard.putData(
    //     "intake command",
    //     (Sendable)
    //         this.arm
    //             .setTargetHeightCommand(4)
    //             .alongWith(intake.intakeUntilCanRangeIsDetected(5, 1)).until(() ->
    // canrange.getCanDistance() < 1));


    Command intakeCommand = arm.setTargetHeightCommand(Constants.ARM_INTAKE_ANGLE)
    .alongWith(intake.setTargetSpeedCommand(Constants.ARM_INTAKE_ANGLE)
    .unless(() -> canrange.getCanDistance() < Constants.CANRANGE_DETECTION_DISTANCE));

    SmartDashboard.putData("intake command", intakeCommand.andThen(new WaitUntilCommand(() -> false)));



    // .andThen(new WaitUntilCommand(() -> false))  

    SmartDashboard.putData(
        "arm set target angle 0", (Sendable) this.arm.setTargetHeightCommandConsistentEnd(0));
    SmartDashboard.putData(
        "arm set target angle 5", (Sendable) this.arm.setTargetHeightCommandConsistentEnd(5));

    SmartDashboard.putData(
        "intake set target speed 5", (Sendable) this.intake.setTargetSpeedCommand(5));
    SmartDashboard.putData(
        "intake set target speed 0", (Sendable) this.intake.setTargetSpeedCommand(0));
    SmartDashboard.putData(
        "intake set target speed -5", (Sendable) this.intake.setTargetSpeedCommand(-5));

    SmartDashboard.putData(
        "set canrange vision 0", (Sendable) this.canrange.setCanrangeDistanceCommand(0));
    SmartDashboard.putData(
        "set canrange vision 5", (Sendable) this.canrange.setCanrangeDistanceCommand(5));

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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
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
