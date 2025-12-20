package frc.robot.subsystems.vision.detection.detectionManagement;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

public class DetectedObject {

    private List<Detection> detections = new ArrayList<>();

    private double lastDetectionTimestamp = 0;

    public DetectedObject(Detection detection) {
        addDetection(detection);
    }

    /**
     * adds a detection to the detected object using a kalman filter
     * @param detection
     */
    public void addDetection(Detection detection) {
        
        if(detection.getDetectionTimeSec() > lastDetectionTimestamp) {
            lastDetectionTimestamp = detection.getDetectionTimeSec();
        }
    }

    /**
     * 
     * @return the estimated position of the detection
     */
    public Pose2d getEstimatedPosition() {
        return new Pose2d();
    }

    /**
     * Gets the confidence that this object is real
     * @return confidence 0 to 1
     */
    public double getConfidence() {
        //here add logic to give it zero confidence if there aren't enough detections OR if the last detection was to long ago
        return 0;
    }

    /**
     * gets the last time this object was updated
     * @return time seconds
     */
    public double getLastTimeUpdated() {
        return lastDetectionTimestamp;
    }

    /**
     * Gets wether or not a given detection is likely to be part of this object, 
     * @param detection
     * @return corellation 0 - 1
     */
    public double getCorrelation(Detection detection) {
        return 0;
    }

        /**
     * 
     * @param pose
     * @return distance in meters
     */
    public double getDistance(Pose2d pose) {
        return getEstimatedPosition().getTranslation().getDistance(pose.getTranslation());
    }
}
