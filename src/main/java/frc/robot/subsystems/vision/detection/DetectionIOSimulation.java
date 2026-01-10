package frc.robot.subsystems.vision.detection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.detection.detectionManagement.Detection;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class DetectionIOSimulation implements DetectionIO {

  Pose2d pose2d = new Pose2d();
  Random random = new Random();

  List<Detection> detections = new ArrayList<>();

  /** updates the inputs for advantage kit logging purposes */

  /** updates the vision systems. THis should be called in the periodic function of the subsystem */
  public void update() {
    double x = random.nextDouble(0.5);
    double y = random.nextDouble(0.5);

    detections.add(new Detection(Timer.getFPGATimestamp(), new Pose2d(x, y, new Rotation2d()), 0));
  }
  ;

  /**
   * Should be cleared after obtaining
   *
   * @return a list of detections since it was cleared
   */
  public List<Detection> getDetections() {
    return detections;
  }

  /** clears the list of new detections */
  public void clearDetections() {
    detections = new ArrayList<>();
  }

  /** sets the pipeline of the limelight. This is currently not implemented */
  public void setPipeline(int pipeline) {}
}
