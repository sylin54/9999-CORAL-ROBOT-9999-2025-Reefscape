package frc.robot.subsystems.vision.detection.detectionManagement;

import java.util.UUID;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import frc.robot.subsystems.vision.VisionConstants;

public class DetectedObject {

    //counter for the amount of detections this object has had. That way if it hasn't had that much yet we can just ignore it
    private double detectionAmount = 0;

    private double lastDetectionTimestamp = 0;

    private UUID id;

    //tweak values later
    private ObjectKalmanFilter objectKalmanFilter = new ObjectKalmanFilter();

    public DetectedObject(Detection detection) {
        addDetection(detection);

        id = UUID.randomUUID();
    }

    /**
     * updates kalman filter. MUST BE CALLED PERIODICALLY
     */
    public void periodic() {
        objectKalmanFilter.periodic();
    }

    /**
     * adds a detection to the detected object using a kalman filter
     * @param detection
     */
    public void addDetection(Detection detection) {
        
        if(detection.getDetectionTimeSec() > lastDetectionTimestamp) {
            lastDetectionTimestamp = detection.getDetectionTimeSec();
        }

        detectionAmount++;

        objectKalmanFilter.addMeasurement(detection.getPosition().getTranslation());

    }

    /**
     * 
     * @return the estimated position of the detection
     */
    public Pose2d getEstimatedPosition() {
        return new Pose2d(objectKalmanFilter.getFilteredPosition(), new Rotation2d());
    }

    /**
     * Gets the confidence that this object is real. this only checks for if it has enough detections
     * @return confidence 0 to 1
     */
    public double getConfidence() {
        //here add logic to give it zero confidence if there aren't enough detections OR if the last detection was to long ago

        if(detectionAmount < VisionConstants.MIN_DETECTIONS_TO_CONSIDER_OBJECT) {
            return 0;
        }

        return 1;
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

        double distance = detection.getPosition().getTranslation().getDistance(getEstimatedPosition().getTranslation());

        double endZone = VisionConstants.kValCorellation * VisionConstants.ALGAE_RADIUS.in(Units.Meter);
        
        //if to far there is no corellation
        if(distance > endZone) {
            return 0;
        }

        //distnace = endzone then the corellation is 0.5. Distance = 0 then corellation is 1
        double corellation = endZone/(endZone + distance);

        return corellation;
    }

        /**
     * 
     * @param pose
     * @return distance in meters
     */
    public double getDistance(Pose2d pose) {
        return getEstimatedPosition().getTranslation().getDistance(pose.getTranslation());
    }

    /**
     * 
     * @return the pose of the detection
     */
    public Pose2d getPose() {
        return new Pose2d();
    }

    /**
     * 
     * @return the unique id of the detection
     */
    public UUID getID() {
        return id;
    }
}
