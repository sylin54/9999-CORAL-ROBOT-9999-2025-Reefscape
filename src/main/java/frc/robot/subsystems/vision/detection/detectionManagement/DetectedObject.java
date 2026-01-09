package frc.robot.subsystems.vision.detection.detectionManagement;

import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import frc.robot.subsystems.vision.VisionConstants;

public class DetectedObject {

    private double lastDetectionTimestamp = 0;

    private UUID id;

    //tweak values later
    private ObjectKalmanFilter objectKalmanFilter = new ObjectKalmanFilter();

    //list of detections for logging purposes
    private List<Detection> detections;

    public DetectedObject(Detection detection) {
        addDetection(detection);

        id = UUID.randomUUID();

        detections = new ArrayList<>();
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

        objectKalmanFilter.addMeasurement(detection.getPosition().getTranslation());

        detections.add(detection);

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

        if(getDetectionAmount() < VisionConstants.MIN_DETECTIONS_TO_CONSIDER_OBJECT) {
            return 0;
        }

        return 1;
    }

    /**
     * 
     * @return the total amount of detections put into this object
     */
    public int getDetectionAmount() {
        return detections.size();
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
     * @return the unique id of the detection
     */
    public UUID getID() {
        return id;
    }

    /**
     * @returns the position of all of the detections
     */
    public List<Pose2d> getDetectionsPose() {
        List<Pose2d> detectionsPose = new ArrayList<>();

        for(Detection detection : detections) {
            detectionsPose.add(detection.getPosition());
        }

        return detectionsPose;
    }
    
    /**
     * logs all of the values onto advantage kti
     * 
     * @param title the title put before the logged values
     */
    public void log(String title) {
        Logger.recordOutput(title + "/detectionAmount", getDetectionAmount());
        Logger.recordOutput(title + "/lastDetectionTimestamp", getLastTimeUpdated());
        Logger.recordOutput(title + "/id", getID().toString());
        Logger.recordOutput(title + "/confidence", getConfidence());
        Logger.recordOutput(title + "/estimatedPosition", getEstimatedPosition());
        Logger.recordOutput(title + "/detections", (Pose2d[]) getDetectionsPose().toArray());
    }
}
