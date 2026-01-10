package frc.robot.subsystems.vision.detection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.LimelightHelpers.RawDetection;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.detection.detectionManagement.Detection;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class DetectionIOLimelight implements DetectionIO {

  private String name;

  private Supplier<Pose2d> robotPoseSupplier;

  private List<Detection> detections;

  public DetectionIOLimelight(String name, Supplier<Pose2d> robotPoseSupplier) {
    this.name = name;

    this.robotPoseSupplier = robotPoseSupplier;

    detections = new ArrayList<>();
  }

  /** updates the vision systems. THis should be called in the periodic function of the subsystem */
  public void update() {

    RawDetection[] rawDetections = LimelightHelpers.getRawDetections(name);

    double curTimeStamp = Timer.getFPGATimestamp();

    for (int i = 0; i < rawDetections.length; i++) {
      RawDetection rawDetection = rawDetections[i];

      int id = rawDetection.classId;

      Translation2d offset = calcDistToObject(rawDetection.txnc, rawDetection.tync);

      Detection detection =
          new Detection(curTimeStamp, robotPoseSupplier.get().getTranslation().plus(offset), id);

      detections.add(detection);
    }
  }

  /**
   * Should be cleared after obtaining
   *
   * @return a list of detections since it was cleared
   */
  public List<Detection> getDetections() {

    // creates a new list so we don't tamper with the old one
    List<Detection> returnVal = new ArrayList<>();
    returnVal.addAll(detections);

    return returnVal;
  }

  /** clears the list of new detections */
  public void clearDetections() {
    detections = new ArrayList<>();
  }

  private Translation2d calcDistToObject(double tx, double ty) {

    Transform3d cameraOffset = VisionConstants.ROBOT_TO_ARDUCAM_DETECTION;
    Distance algaeRad = VisionConstants.ALGAE_RADIUS;

    // verticlal angle
    double totalAngleY = Units.degreesToRadians(-ty) - cameraOffset.getRotation().getY();

    // System.out.println("total angle: " + totalAngleY);

    // all this is doing is : horizantol = z/tan(angle)
    Distance distAwayY =
        // the hieght of the camera
        cameraOffset
            .getMeasureZ()
            // aims at the lower end of the coral
            .minus(algaeRad)
            .div(Math.tan(totalAngleY)); // robot x
    // System.out.println("dist away y" + distAwayY);

    // distance of camera to ground on y axis I believe it
    Distance distHypotenuseYToGround =
        BaseUnits.DistanceUnit.of(
            Math.hypot(
                distAwayY.in(BaseUnits.DistanceUnit),
                cameraOffset.getMeasureZ().minus(algaeRad).in(BaseUnits.DistanceUnit)));

    // System.out.println("dist hypo to ground: " + distHypotenuseYToGround);

    // same thing as before
    double totalAngleX = Units.degreesToRadians(-tx) + cameraOffset.getZ();

    // System.out.println("total angle x: " + totalAngleX);

    Distance distAwayX = distHypotenuseYToGround.times(Math.tan(totalAngleX)); // robot y

    // System.out.println("dist away x: " + distAwayX);

    SmartDashboard.putNumber(name + "/tx", tx);
    SmartDashboard.putNumber(name + "/ty", ty);
    SmartDashboard.putNumber(
        name + "/Distance Away Y", distAwayY.in(edu.wpi.first.units.Units.Meters));
    SmartDashboard.putNumber(
        name + "/Distance Away X", distAwayX.in(edu.wpi.first.units.Units.Meters));
    SmartDashboard.putNumber(
        name + "/Distance Away Hyp ", distHypotenuseYToGround.in(edu.wpi.first.units.Units.Meters));

    return new Translation2d(distAwayY, distAwayX);
  }
}
