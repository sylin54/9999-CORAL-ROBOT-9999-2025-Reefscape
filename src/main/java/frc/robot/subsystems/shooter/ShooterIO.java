package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.detection.DetectionIOInputsAutoLogged;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        Pose2d closestDetectionPose = null;
        boolean isDetected = false;
        int numberOfObjects = 0;
        double objectDistance = 0;
    }

    /** updates the inputs for advantage kit logging purposes */
    public default void updateInputs(ShooterIOInputsAutoLogged inputs) {
    }

    public default void setSpeed(double speed) {
    }

    public default double getSpeed() {
        return 0;
    }

    public default boolean isOnTarget() {
        return false;
    }
}
