package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Shooter extends SubsystemBase{

    //this shouldn't be here but it is for now because we're probably gonna move this
    public static final double TOLERANCE = 0.1;

    private DoubleSupplier distanceSupplier;

    //just here for the logging
    @AutoLogOutput
    private double automaticSpeed = 0;

    @AutoLogOutput
    private boolean isManual = true;

    @AutoLogOutput
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

        //calculate speed that automatically updates with distance
        automaticSpeed = getSpeedFromDistance(distanceSupplier.getAsDouble());

        double speed = getSpeedTarget();
        shooterIO.setSpeed(speed);

    }

    //SUBSYSTEM METHODS

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
        return automaticSpeed;
    }

    /**
     * 
     * @return wether the speed is the target speed
     */
    public boolean isOnTarget() {
        return shooterIO.isOnTarget();
    }

    //COMMANDS
    /**
     * sets the manual speed of the flywheel then ends immediately
     * @param speed the speed of the flywheel
     * @return the finished command
     */
    public Command setManualSpeedCommand(double speed) {
        return new InstantCommand(() -> this.setManualSpeed(speed));
    }

    /**
     * sets the target speed command then ends when it reaches that speed
     * @param speed the speed it gets set to
     * @return the finished command
     */
    public Command setManualSpeedCommandConsistentEnd(double speed) {
        return new InstantCommand(() -> this.setManualSpeed(speed))
        .andThen(new WaitUntilCommand(() -> this.isOnTarget()));
    }

    /**
     * returns a command that revs up to shoot at the distance and ends immediately
     * @return
     */
    public Command setAutomaticCommand() {
        return new InstantCommand(() -> this.setAutomatic());
    }

    /**
     * returns a command that revs up to shoot at the distance then ends when it reaches that point
     * @return
     */
    public Command setAutomaticCommandConsistentEnd() {
        return new InstantCommand(() -> this.setAutomatic())
        .andThen(new WaitUntilCommand(() -> this.isOnTarget()));
    }

    //HELPER METHODS
    /**
     * gets the needed speed based on distance away. Right now this is linear bc idrk what we're gonna do for this yet. Look into 2024 reefscape where we had an algorithm from setpoints for ideas
     * @param distance
     * @return
     */
    private double getSpeedFromDistance(double distance) {
        return distance * 10;
    }
    
}


/*
 * two modes: supply a distance and have it wind up to that or
 * supply hand values and have it wind up to that
 * 
 */
