package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.Vector;
import edu.wpi.first.wpiutil.math.numbers.N1;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PTO.PTO;
import frc.robot.subsystems.UnitModel;
import webapp.FireLog;
import frc.robot.utils.VisionModule;
import java.io.InputStream;
import java.io.InputStreamReader;

import static frc.robot.Constants.Shooter.*;

/**
 * The shooter class represents the physical shooter.
 * The purpose of the flywheel is to provide a set of functions to handle a shooting situation in a game.
 *
 * @author Barel
 * @version 1.0
 * @using TalonFX
 * @since 2021
 */
public class Shooter extends SubsystemBase {
    private final TalonFX upMotor = new TalonFX(Ports.Shooter.UP);
    private final UnitModel unitModel = new UnitModel(TICKS_PER_ROTATION);

    private final LinearRegression velocityEstimator;
    private LinearSystemLoop<N1, N1, N1> upMotorStateSpacePredictor;
    private LinearSystemLoop<N1, N1, N1> stateSpacePredictor;
    private double lastOmega = 0;
    private final Timer shootingTimer = new Timer();
    private double lastTime = 1;

    public Shooter() {
        this.stateSpacePredictor = constructLinearSystem(J.get());
        this.velocityEstimator = new LinearRegression(readCSV());

        upMotor.setInverted(Ports.Shooter.UP_INVERTED);
        this.upMotorStateSpacePredictor = constructLinearSystem(UP_MOTOR_J.get());
    }

    /**
     * Initialize the linear system to the default values in order to use the state-space.
     *
     * @return an object that represents the model to reach the velocity at the best rate.
     */
    private LinearSystemLoop<N1, N1, N1> constructLinearSystem(double J) {
        if (J == 0) throw new RuntimeException("J must have non-zero value");
        // https://file.tavsys.net/control/controls-engineering-in-frc.pdf Page 76
        Vector<N1> A = VecBuilder.fill(-Math.pow(GEAR_RATIO, 2) * kT / (kV * OMEGA * J));
        Vector<N1> B = VecBuilder.fill((GEAR_RATIO * kT) / (OMEGA * J));
        LinearSystem<N1, N1, N1> stateSpace = new LinearSystem<>(A, B, Matrix.eye(Nat.N1()), new Matrix<>(Nat.N1(), Nat.N1()));
        KalmanFilter<N1, N1, N1> kalman = new KalmanFilter<>(Nat.N1(), Nat.N1(), stateSpace,
                VecBuilder.fill(MODEL_TOLERANCE),
                VecBuilder.fill(ENCODER_TOLERANCE),
                Constants.LOOP_PERIOD
        );
        LinearQuadraticRegulator<N1, N1, N1> lqr = new LinearQuadraticRegulator<>(stateSpace, VecBuilder.fill(VELOCITY_TOLERANCE),
                VecBuilder.fill(Constants.NOMINAL_VOLTAGE),
                Constants.LOOP_PERIOD // time between loops, DON'T CHANGE
        );

        return new LinearSystemLoop<>(stateSpace, lqr, kalman, Constants.NOMINAL_VOLTAGE, Constants.LOOP_PERIOD);
    }

    /**
     * Internal function to create an input stream reader, in order values from the shooting experiments.
     *
     * @return a reader to the CSV.
     */
    private InputStreamReader readCSV() {
        InputStream is = getClass().getResourceAsStream(PATH_TO_CSV);
        assert is != null : "Can't create input stream";

        return new InputStreamReader(is);
    }

    /**
     * @return the velocity of the shooter. [RPS]
     * @see #setVelocity(double, double)
     */
    public double getVelocity() {
        if (RobotContainer.pto.getState() == PTO.GearboxState.CLIMBER) {
            return 0;
        }

        return unitModel.toVelocity(RobotContainer.pto.getMaster().getSelectedSensorVelocity());
    }

    public double getCurrent() {
        return RobotContainer.pto.getMaster().getSupplyCurrent();
    }

    /**
     * Set the velocity to apply by the motor.
     *
     * @param velocity the desired velocity at which the motor will rotate. [RPS]
     * @see #setVelocity(double, double)
     */
    public void setVelocity(double velocity) {
        setVelocity(velocity, Constants.LOOP_PERIOD);
    }

    /**
     * Set the velocity to apply by the motor.
     *
     * @param velocity     the desired velocity at which the motor will rotate. [RPS]
     * @param timeInterval the time interval from the last call of this function. [sec]
     * @see #setPower(double)
     */
    public void setVelocity(double velocity, double timeInterval) {
        stateSpacePredictor.setNextR(VecBuilder.fill(velocity)); //r = reference (setpoint)
        stateSpacePredictor.correct(VecBuilder.fill(getVelocity()));
        stateSpacePredictor.predict(timeInterval);

        double voltageToApply = stateSpacePredictor.getU(0); // u = input, calculated by the input.
        // returns the voltage to apply (between -12 and 12)
        setPower(voltageToApply / Constants.NOMINAL_VOLTAGE); // map to be between -1 and 1
    }

    /**
     * Set the power to apply by the motor.
     *
     * @param power the power at which the motor will rotate. [percentage, between -1 and 1]
     * @see #stop()
     */
    public void setPower(double power) {
        if (RobotContainer.pto.getState() == PTO.GearboxState.CLIMBER) return;

        RobotContainer.pto.getMaster().set(TalonFXControlMode.PercentOutput, power,
                DemandType.ArbitraryFeedForward, Constants.Shooter.ARBITRARY_FEED_FORWARD);
        RobotContainer.pto.getSlave().set(TalonFXControlMode.PercentOutput, power,
                DemandType.ArbitraryFeedForward, Constants.Shooter.ARBITRARY_FEED_FORWARD);
    }

    /**
     * Set the velocity to apply by the motor.
     *
     * @param velocity the desired velocity at which the motor will rotate. [RPS]
     * @see #setVelocity(double, double)
     */
    public void setVelocityUp(double velocity) {
        setVelocityUp(velocity, Constants.LOOP_PERIOD);
    }

    /**
     * Set the velocity to apply by the motor.
     *
     * @param velocity     the desired velocity at which the motor will rotate. [RPS]
     * @param timeInterval the time interval from the last call of this function. [sec]
     * @see #setPower(double)
     */
    public void setVelocityUp(double velocity, double timeInterval) {
        upMotorStateSpacePredictor.setNextR(VecBuilder.fill(velocity)); //r = reference (setpoint)
        upMotorStateSpacePredictor.correct(VecBuilder.fill(getVelocity()));
        upMotorStateSpacePredictor.predict(timeInterval);

        double voltageToApply = upMotorStateSpacePredictor.getU(0); // u = input, calculated by the input.
        // returns the voltage to apply (between -12 and 12)
        setPowerUp(voltageToApply / Constants.NOMINAL_VOLTAGE); // map to be between -1 and 1
    }

    /**
     * Set the power to apply by the motor.
     *
     * @param power the power at which the motor will rotate. [percentage, between -1 and 1]
     * @see #stop()
     */
    public void setPowerUp(double power) {
        if (RobotContainer.pto.getState() == PTO.GearboxState.CLIMBER) return;

        upMotor.set(ControlMode.PercentOutput, power, DemandType.ArbitraryFeedForward, ARBITRARY_FEED_FORWARD_UP);
    }

    /**
     * Estimate the velocity that the shooter should apply in order to reach the target.
     *
     * @param distance the distance from the target. [meters]
     * @return the velocity that should be applied by the shooter in order to reach the target.[RPS]
     */
    public double estimateVelocityFromDistance(double distance) {
        return velocityEstimator.estimateVelocityFromDistance(distance);
    }

    /**
     * Get whether the flywheel has reached the desired velocity in order to reach the target.
     *
     * @param setpoint the desired velocity at the motor will rotate. [RPS]
     * @return whether the flywheel reaches the desired velocity.
     */
    public boolean hasReachedSetpoint(double setpoint) {
        return Math.abs(getVelocity() - setpoint) < Constants.Shooter.VELOCITY_TOLERANCE;
    }

    /**
     * Stop the shooter.
     */
    public void stop() {
        setPower(0);
        setPowerUp(0);
    }

    @Override
    public void periodic() {
        shootingTimer.start();
        final double currentTime = shootingTimer.get();
        double omega = getVelocity() * Units.inchesToMeters(4);
        FireLog.log("velocity", getVelocity());
        FireLog.log("up-velocity", unitModel.toVelocity(upMotor.getSelectedSensorVelocity()));
        FireLog.log("radial-velocity", omega);
        FireLog.log("setpoint", RobotContainer.velocity.get());
        FireLog.log("accl-omega", (omega - lastOmega) / (currentTime - lastTime));
        FireLog.log("voltage", 12);
        FireLog.log("current", getCurrent());
        lastTime = currentTime;

        lastOmega = omega;
        this.stateSpacePredictor = constructLinearSystem(J.get());
        this.upMotorStateSpacePredictor = constructLinearSystem(UP_MOTOR_J.get());
    }
}
