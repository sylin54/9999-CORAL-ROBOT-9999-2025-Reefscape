package frc.robot.subsystems.vision.detection.detectionManagement;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Detection {
    //get units for this
    private double detectionTimeSec;
    private Pose2d position;

    public Detection(double detectionTimeSec, Pose2d position) {
        this.detectionTimeSec = detectionTimeSec;
        this.position = position;
    }

    public Detection(double detectionTimeSec, Translation2d position) {
        this(detectionTimeSec, new Pose2d(position, new Rotation2d()));
    }

    //getters
    /**
     * 
     * @return the timestamp when this detection (system clock in seconds, Timer.getFPGATTimestamp)
     */
    public double getDetectionTimeSec() {
        return detectionTimeSec;
    }

    /**
     * 
     * @return the position where this detection was taken
     */
    public Pose2d getPosition() {
        return position;
    }


}
