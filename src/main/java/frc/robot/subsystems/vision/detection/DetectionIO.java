package frc.robot.subsystems.vision.detection;

import frc.robot.subsystems.vision.detection.detectionManagement.Detection;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;

public interface DetectionIO {

  @AutoLog
  public static class DetectionIOInputs {}

  /** updates the inputs for advantage kit logging purposes */
  public default void updateInputs(DetectionIOInputsAutoLogged inputs) {}

  /** updates the vision systems. THis should be called in the periodic function of the subsystem */
  public default void update() {}
  ;

  /**
   * Should be cleared after obtaining
   *
   * @return a list of detections since it was cleared
   */
  public default List<Detection> getDetections() {
    return new ArrayList<>();
  }

  /** clears the list of new detections */
  public default void clearDetections() {}

  /** sets the pipeline of the limelight. This is currently not implemented */
  public default void setPipeline(int pipeline) {}
  ;
}
