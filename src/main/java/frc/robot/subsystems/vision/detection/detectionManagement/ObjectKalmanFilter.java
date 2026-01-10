package frc.robot.subsystems.vision.detection.detectionManagement;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;

public class ObjectKalmanFilter {

  // two different kalman filters for each axis
  // inputs x/y measurement
  // outputs: x/y
  // states: x/y, x/y velocity
  // knobs: x/y momentum, x/y inertia

  // the momentum of the object, how much it sustains velocity
  private double kV;
  // the inertia of the object, how quickly it can change it's velocity
  private double kA;

  // the standard deviation/covariance matrix of the state/predict step
  private Matrix stateStdDevs;
  // the standard deviation/covariance matrix of the measurement step
  private Matrix measurementStdDevs;

  // the period/time it takes between repeats
  private double dtSeconds;

  // the modeling of the x/y axis of movement
  private LinearSystem<N2, N1, N2> xBallPlant;
  private LinearSystem<N2, N1, N2> yBallPlant;

  // filter for x/y
  private KalmanFilter<N2, N1, N1> xObserver;
  private KalmanFilter<N2, N1, N1> yObserver;

  // measurement manager variables
  private boolean hasNewMeasurement = false;
  private Translation2d measurement = null;

  /** default instantiation SHOULD ONLY BE USED FOR TESTING */
  public ObjectKalmanFilter() {
    this(0.8, 0.2, VecBuilder.fill(1.0, 2.0), VecBuilder.fill(1.0, 1.000), 0.02);
  }

  /**
   * @param kV momentum of the object
   * @param kA inertia of the object
   * @param stateStdDevs covariance of prediction
   * @param measurementStdDevs covariance of measurement
   * @param dtSeconds period of processing
   */
  public ObjectKalmanFilter(
      double kV,
      double kA,
      Matrix<N2, N1> stateStdDevs,
      Matrix<N2, N1> measurementStdDevs,
      double dtSeconds) {
    this.kV = kV;
    this.kA = kA;

    this.stateStdDevs = stateStdDevs;
    this.measurementStdDevs = measurementStdDevs;

    this.dtSeconds = dtSeconds;

    xBallPlant = LinearSystemId.identifyPositionSystem(kV, kA);
    yBallPlant = LinearSystemId.identifyPositionSystem(kV, kA);

    xObserver =
        new KalmanFilter(
            Nat.N2(), Nat.N1(), xBallPlant, stateStdDevs, measurementStdDevs, dtSeconds);
    yObserver =
        new KalmanFilter(
            Nat.N2(), Nat.N1(), yBallPlant, stateStdDevs, measurementStdDevs, dtSeconds);
  }

  /**
   * adds a measurement to processed next periodic call
   *
   * @param measurement
   */
  public void addMeasurement(Translation2d measurement) {
    this.measurement = measurement;
    hasNewMeasurement = true;
  }

  /** process updates + predicts new values. MUST BE CALLED PERIODICALLY */
  public void periodic() {
    xObserver.predict(VecBuilder.fill(0), dtSeconds);
    yObserver.predict(VecBuilder.fill(0), dtSeconds);

    // procccess any new measurement. Vector fill 0 because this doesn't account for speed
    if (hasNewMeasurement) {
      xObserver.correct(VecBuilder.fill(0), VecBuilder.fill(measurement.getX()));
      yObserver.correct(VecBuilder.fill(0), VecBuilder.fill(measurement.getY()));

      hasNewMeasurement = false;
    }
  }

  /**
   * @return the estimated position of the object
   */
  public Translation2d getFilteredPosition() {
    Translation2d output = new Translation2d(xObserver.getXhat(0), yObserver.getXhat(0));

    return output;
  }

  /**
   * @return the estimated speed of the object
   */
  public Translation2d getFilteredSpeed() {
    Translation2d output = new Translation2d(xObserver.getXhat(1), yObserver.getXhat(1));

    return output;
  }
}
/*
 * points of improvement
 * factor in timestamp to the kalman filter
 * maybe add it to it's own thread if we notice the calls are inconsistent
 */

// reference code. look at this if you wnat a sligthly simpler example.

/*

https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-observers.html
https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-flywheel-walkthrough.html
https://simondlevy.github.io/ekf-tutorial/

package frc.robot.subsystems.vision.detection.detectionManagement;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;

public class testLinearSystem {
    //inputs angle of detection
    //states: angle, angular velocity
    //knobs: momentum, inertia


    //momentum/ how much it sustains the velocity
    private static final double ballKv = 0.8;
    //how quickly it can change it's velocity
    private static final double ballKa = 0.2;

    private LinearSystem<N2, N1, N2> m_ballPlant = LinearSystemId.identifyPositionSystem(ballKv, ballKa);

    private final double dtSeconds = 0.020;

    private double measurement = 0;
    private boolean measurementFresh = false;

    //learn abotu xhat



    //first is angle, second is velocity
    //add fuzz because your unsure or because the system is super dynamic

    //we have the framerate because we're predicting mroe often then we have measurements. the higher the framerate the more often we get a measurement. But we do want to do something witht he infromation in between frames

    //we can go back and feed the timestamp into the kalman filter. maybe add later if we have lag
    private KalmanFilter<N2, N1, N1> observer = new KalmanFilter(Nat.N2(), Nat.N1(), m_ballPlant, VecBuilder.fill(1.0, 2.0), VecBuilder.fill(1.0), dtSeconds);


    //if we have issues maybve consider creating a threat ot do this seperately. Maybe??
    //check if valid limeligth
    public void periodic() {
        observer.predict(VecBuilder.fill(0), dtSeconds);

        if(measurementFresh) {
            observer.correct(VecBuilder.fill(0), VecBuilder.fill(measurement));
            measurementFresh = false;
        }
    }

    public void addMeasurement(double measurement) {
        this.measurement = measurement;
        measurementFresh = true;
    }

    public double getMeasurement() {
        return 0;
    }

    public double getFilteredAngle() {
        return observer.getXhat(0);
    }

    public double getFilteredVelocity() {
        return observer.getXhat(1);
    }


      // Volts per (radian per second)
    private static final double kFlywheelKv = 0.023;

    // Volts per (radian per second squared)
    private static final double kFlywheelKa = 0.001;

    // The plant holds a state-space model of our flywheel. This system has the following properties:
    //
    // States: [velocity], in radians per second.
    // Inputs (what we can "put in"): [voltage], in volts.
    // Outputs (what we can measure): [velocity], in radians per second.
    //
    // The Kv and Ka constants are found using the FRC Characterization toolsuite.
    private final LinearSystem<N1, N1, N1> m_flywheelPlant =
        LinearSystemId.identifyVelocitySystem(kFlywheelKv, kFlywheelKa);

      // The observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N1, N1, N1> m_observer =
      new KalmanFilter<>(
          Nat.N1(),
          Nat.N1(),
          m_flywheelPlant,
          VecBuilder.fill(3.0), // How accurate we think our model is
          VecBuilder.fill(0.01), // How accurate we think our encoder
          // data is
          0.020);

}

 */
