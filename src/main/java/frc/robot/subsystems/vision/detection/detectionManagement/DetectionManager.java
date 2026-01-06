package frc.robot.subsystems.vision.detection.detectionManagement;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.VisionConstants;

public class DetectionManager {
    //this is a list which is bad for performance since we are looping through it multiple times a tick. I believe this is ok?? we should change hte data structe if we notice problems
    //maybe need to add cleanup to put objects close to each other so we don't run into two different objects?? I'm not sure
    public List<DetectedObject> objects;

    //dummy values for now i'm not really sure how this is gonna pan out later
    private double correlationThreshold = VisionConstants.OBJ_CORELLATION_THRESHOLD;
    private double confidenceThreshold = VisionConstants.OBJ_CONFIDENCETHRESHOLD;
    private double removalTimeSec = VisionConstants.OBJ_REMOVAL_TIME;

    public DetectionManager() {
        objects = new ArrayList<>();
    }

    /**
     * must be called about every tick
     */
    public void periodic() {
        checkDetections();

        for(DetectedObject object : objects) {
            object.periodic();
        }
    }

    /**
     * adds a vision detection to the sytem
     * @param detection
     */
    public void addDetection(Detection detection) {

        int bestIndex = -1;
        double bestCorrelation = 0;


        //finds the object that this detection matches the most too
        for(int i = 0; i < objects.size(); i++) {

            DetectedObject curObject = objects.get(i);
            double corellation = curObject.getCorrelation(detection);

            if(corellation > correlationThreshold && corellation > bestCorrelation) {
                bestIndex = i;
                bestCorrelation = corellation;
            }
        }

        //if nothing matched create a new detected object
        if(bestIndex == -1) {
            objects.add(new DetectedObject(detection));
            return;
        }

        objects.get(bestIndex).addDetection(detection);
    }

    /**
     * gets the closest valid object to the given pose
     * @param robotPose
     * @return
     */
    public Pose2d getClosestObjectPosition(Pose2d robotPose) {
        int bestIndex = -1;
        double bestDistance = 1000;

        for(int i = 0; i < objects.size(); i++) {
            DetectedObject curObject = objects.get(i);

            if(curObject.getConfidence() < confidenceThreshold) continue;
            
            double distance = curObject.getDistance(robotPose);

            if(distance > bestDistance) continue;

            bestIndex = i;
            bestDistance = distance;
        }

        //returns the robot pose to stop a robot from accidentally piding accross the field and hurting someone
        if(bestIndex == -1) {
            return robotPose;
        }

        return objects.get(bestIndex).getEstimatedPosition();
    }

    /**
     * 
     * @return if there any detections that match the confidence threshold
     */
    public boolean hasValidDetections() {
        for(DetectedObject detectedObject : objects) {
            if(detectedObject.getConfidence() > confidenceThreshold) return true;
        }
        
        return false;
    }

    /**
     * remove any detections that haven't been updated soon enough
     */
    private void checkDetections() {
        List<DetectedObject> updatedObjects = new ArrayList<>();
        
        for(DetectedObject object : objects) {
            if(Timer.getFPGATimestamp() - object.getLastTimeUpdated() > removalTimeSec) {
                continue;
            }

            updatedObjects.add(object);
        }

        objects = updatedObjects;
    }
}
