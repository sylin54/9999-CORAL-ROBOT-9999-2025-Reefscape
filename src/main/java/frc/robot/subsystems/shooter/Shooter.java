package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{

    //this shouldn't be here but it is for now because we're probably gonna move this
    public static final double TOLERANCE = 0.1;

    private DoubleSupplier distanceSupplier;

    private boolean isManual = true;

    private double manualSpeed = 0;

    private ShooterIO shooterIO;
    private ShooterIOInputsAutoLogged inputs;

    /**
     * 
     * @param shooterIO the hardware interface
     * @param distanceSupplierMeters the distance supplier for when it goes automatic
     */
    public Shooter(ShooterIO shooterIO, DoubleSupplier distanceSupplierMeters) {
        this.distanceSupplier = distanceSupplierMeters;
        this.shooterIO = shooterIO;
    }

    @Override
    public void periodic() {
        shooterIO.updateInputs(inputs);
        Logger.processInputs("detection", inputs);

        double speed = getSpeedTarget();
        shooterIO.setSpeed(speed);

    }

    /**
     * Sets the robot to use the distance supplier to determine shooting speed
     */
    public void setAutomatic() {
        isManual = false;
    }
 
    /**
     * 
     * @param speed the speed the flywheel will pid too
     */
    public void setManualSpeed(double speed) {
        this.isManual = true;
        this.manualSpeed = speed;
    }

    /**
     * @return the target speed that the flywheel is pid'ing to. Can be the manual or the automatic calculated speed
     */
    public double getSpeedTarget() {
        if(isManual) return manualSpeed;

        double automaticSpeed = getSpeedFromDistance(distanceSupplier.getAsDouble());
        return automaticSpeed;
    }

    
    public boolean isOnTarget() {
        return shooterIO.isOnTarget();
    }

    private double getSpeedFromDistance(double distance) {
        return 0;
    }
    
}


/*
 * two modes: supply a distance and have it wind up to that or
 * supply hand values and have it wind up to that
 * 
 */
