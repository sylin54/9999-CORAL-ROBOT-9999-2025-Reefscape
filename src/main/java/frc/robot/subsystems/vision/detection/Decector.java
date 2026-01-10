package frc.robot.subsystems.vision.detection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.detection.detectionManagement.Detection;
import frc.robot.subsystems.vision.detection.detectionManagement.DetectionManager;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Decector extends SubsystemBase {
  private DetectionIO detectionIO;
  private DetectionIOInputsAutoLogged inputs = new DetectionIOInputsAutoLogged();

  private DetectionManager detectionManager;

  private Supplier<Pose2d> poseSupplier;

  public Decector(DetectionIO detectionIO, Supplier<Pose2d> poseSupplier) {
    this.detectionIO = detectionIO;

    this.poseSupplier = poseSupplier;

    detectionManager = new DetectionManager();
  }

  @Override
  public void periodic() {
    detectionIO.update();
    detectionIO.updateInputs(inputs);
    Logger.processInputs("detection", inputs);

    // gets and clears detections
    List<Detection> detections = detectionIO.getDetections();
    detectionIO.clearDetections();

    for (Detection detection : detections) {
      detectionManager.addDetection(detection);
    }

    detectionManager.periodic();
  }

  public boolean isDetected() {
    return detectionManager.isDetected();
  }

  public Pose2d getObjectPose() {

    Pose2d objectPose = detectionManager.getClosestObjectPosition(poseSupplier.get());

    // makes it so the robot will rotate towards where it is moving when driving to the pose
    Translation2d delta = objectPose.getTranslation().minus(poseSupplier.get().getTranslation());

    Rotation2d heading = new Rotation2d(delta.getX(), delta.getY());
    ;

    return new Pose2d(objectPose.getTranslation(), heading);
  }
}
