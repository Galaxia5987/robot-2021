package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.Vector;
import edu.wpi.first.wpiutil.math.numbers.N1;
import frc.robot.Constants;
import frc.robot.UnitModel;
import frc.robot.valuetuner.WebConstant;
import org.techfire225.webapp.FireLog;

import static frc.robot.Constants.SwerveModule.DEADBAND;

/**
 * The Swerve Module Subsystem controls the individual wheel with the controls from the Swerve Drive Subsystem.
 */
public class SwerveModule extends SubsystemBase {
    private final TalonFX driveMotor;
    private final TalonSRX angleMotor;

    private final int wheel;

    private final UnitModel driveUnitModel = new UnitModel(Constants.SwerveDrive.TICKS_PER_METER);
    private final UnitModel angleUnitModel = new UnitModel(Constants.SwerveDrive.TICKS_PER_RAD);
    private final WebConstant[] anglePIDF;
    //    private final double J = 0.0043;
    private final double J;
    private final Timer timer = new Timer();
    private double startAngle = 0;
    private LinearSystemLoop<N1, N1, N1> stateSpace;
    private double currentTime, lastTime;


    public SwerveModule(int wheel, int driveMotorPort, int angleMotorPort, boolean[] inverted, WebConstant[] anglePIDF) {
        J = Constants.SwerveDrive.J[wheel];

        this.anglePIDF = anglePIDF;
        driveMotor = new TalonFX(driveMotorPort);
        angleMotor = new TalonSRX(angleMotorPort);

        // configure feedback sensors
        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, Constants.TALON_TIMEOUT);

        angleMotor.setNeutralMode(NeutralMode.Brake);

        // set inversions
        angleMotor.setInverted(inverted[0]);
        driveMotor.setInverted(inverted[1]);

        angleMotor.setSensorPhase(inverted[2]);
        driveMotor.setSensorPhase(inverted[3]);

        // Set amperage limits
        SupplyCurrentLimitConfiguration currLimitConfig = new SupplyCurrentLimitConfiguration(
                Constants.ENABLE_CURRENT_LIMIT,
                Constants.SwerveDrive.MAX_CURRENT,
                Constants.SwerveModule.TRIGGER_THRESHOLD_CURRENT,
                Constants.SwerveModule.TRIGGER_THRESHOLD_TIME
        );

        driveMotor.configSupplyCurrentLimit(currLimitConfig);

//        angleMotor.configPeakCurrentLimit(Constants.SwerveDrive.MAX_CURRENT);
//        angleMotor.configPeakCurrentDuration(20);
//        angleMotor.configContinuousCurrentLimit(Constants.SwerveDrive.MAX_CURRENT);
        angleMotor.configSupplyCurrentLimit(currLimitConfig);
//        angleMotor.enableCurrentLimit(Constants.ENABLE_CURRENT_LIMIT);

        configPIDF();

        // set voltage compensation and saturation
        driveMotor.enableVoltageCompensation(Constants.SwerveModule.ENABLE_VOLTAGE_COMPENSATION);
        driveMotor.configVoltageCompSaturation(Constants.SwerveModule.VOLTAGE_SATURATION);

        angleMotor.enableVoltageCompensation(Constants.SwerveModule.ENABLE_VOLTAGE_COMPENSATION);
        angleMotor.configVoltageCompSaturation(Constants.SwerveModule.VOLTAGE_SATURATION);

        angleMotor.selectProfileSlot(0, 0);
        driveMotor.selectProfileSlot(1, 0);
        driveMotor.setSelectedSensorPosition(0);

        this.wheel = wheel;

        stateSpace = constructLinearSystem(J);
        timer.start();
    }

    private static double getTargetError(double angle, double currentAngle) {
        double cwDistance = angle - currentAngle;
        double ccwDistance = 2 * Math.PI - (Math.abs(cwDistance));
        if (Math.abs(cwDistance) < ccwDistance) {
            return cwDistance;
        } else if (cwDistance < 0) {
            return ccwDistance;
        }
        return -ccwDistance;
    }


    /**
     * Initialize the linear system to the default values in order to use the state-space.
     *
     * @return an object that represents the model to reach the velocity at the best rate.
     */
    private LinearSystemLoop<N1, N1, N1> constructLinearSystem(double J) {
        if (J == 0) throw new RuntimeException("J must have non-zero value");
        // https://file.tavsys.net/control/controls-engineering-in-frc.pdf Page 76
        Vector<N1> A = VecBuilder.fill(-Math.pow(7.5, 2) * Constants.SwerveModule.kT / (Constants.SwerveModule.kV * Constants.SwerveModule.OMEGA * J));
        Vector<N1> B = VecBuilder.fill((7.5 * Constants.SwerveModule.kT) / (Constants.SwerveModule.OMEGA * J));
        LinearSystem<N1, N1, N1> stateSpace = new LinearSystem<>(A, B, Matrix.eye(Nat.N1()), new Matrix<>(Nat.N1(), Nat.N1()));
        KalmanFilter<N1, N1, N1> kalman = new KalmanFilter<>(Nat.N1(), Nat.N1(), stateSpace,
                VecBuilder.fill(Constants.SwerveModule.MODEL_TOLERANCE),
                VecBuilder.fill(Constants.SwerveModule.ENCODER_TOLERANCE),
                Constants.LOOP_PERIOD
        );
        LinearQuadraticRegulator<N1, N1, N1> lqr = new LinearQuadraticRegulator<>(stateSpace, VecBuilder.fill(Constants.SwerveModule.VELOCITY_TOLERANCE),
                VecBuilder.fill(Constants.NOMINAL_VOLTAGE),
                Constants.LOOP_PERIOD // time between loops, DON'T CHANGE
        );

        return new LinearSystemLoop<>(stateSpace, lqr, kalman, Constants.NOMINAL_VOLTAGE, Constants.LOOP_PERIOD);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeed(), new Rotation2d(getAngle()));
    }

    public void setState(SwerveModuleState state) {
//        setSpeed(state.speedMetersPerSecond);
        FireLog.log("target-angle " + wheel, state.angle.getDegrees());
        setAngle(state.angle.getRadians());
    }

    public void setStateExperimental(SwerveModuleState state, double filter) {
        if (state.speedMetersPerSecond < 0) {
            setSpeed(-(Math.abs(state.speedMetersPerSecond) + filter));
        } else {
            setSpeed(state.speedMetersPerSecond + filter);
        }
        FireLog.log("target-angle " + wheel, state.angle.getDegrees());
        setAngle(state.angle.getRadians());
    }


    /**
     * @return the speed of the wheel in [m/s]
     */
    public double getSpeed() {
        if (wheel != 1)
            return driveUnitModel.toVelocity(driveMotor.getSelectedSensorVelocity(1));
        return driveUnitModel.toVelocity(driveMotor.getSelectedSensorVelocity(0));
    }

    /**
     * sets the speed of the the wheel in ticks per 100ms
     *
     * @param speed the speed of the wheel in [m/s]
     */
    public void setSpeed(double speed) {
        double timeInterval = Math.max(20, currentTime - lastTime);
        double currentSpeed = getSpeed() / (2 * Math.PI * Constants.SwerveModule.RADIUS); // [rps]
        double targetSpeed = speed / (2 * Math.PI * Constants.SwerveModule.RADIUS); // [rps]

        stateSpace.setNextR(VecBuilder.fill(targetSpeed)); // r = reference (setpoint)
        stateSpace.correct(VecBuilder.fill(currentSpeed));
        stateSpace.predict(timeInterval);

        double voltageToApply = stateSpace.getU(0); // u = input, calculated by the input.
        // returns the voltage to apply (between -12 and 12)
        driveMotor.set(ControlMode.PercentOutput, voltageToApply / Constants.NOMINAL_VOLTAGE);
    }

    /**
     * @return the angle of the wheel in radians
     */
    public double getAngle() {
        return Math.IEEEremainder(angleUnitModel.toUnits(angleMotor.getSelectedSensorPosition()) + startAngle, 2 * Math.PI);
    }

    /**
     * sets the angle of the wheel, in consideration of the shortest path to the target angle
     *
     * @param angle the target angle in radians
     */
    public void setAngle(double angle) {
        double targetAngle = Math.IEEEremainder(angle, 2 * Math.PI);
        if (Math.abs(angleUnitModel.toTicks(targetAngle - getAngle())) < Constants.SwerveDrive.ALLOWABLE_ANGLE_ERROR)
            return;

        double currentAngle = getAngle();
        double error = getTargetError(targetAngle, currentAngle);
        angleMotor.set(ControlMode.Position, angleMotor.getSelectedSensorPosition() + angleUnitModel.toTicks(error));
    }

    /**
     * stops the angle motor
     */
    public void stopAngleMotor() {
        angleMotor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * reset encoder value to 0
     */
    public void resetAngleEncoder() {
        angleMotor.setSelectedSensorPosition(0);
    }

    /**
     * config PIDF for the angle motor and drive motor
     */
    public void configPIDF() {
        // set PIDF - angle motor
        angleMotor.config_kP(0, anglePIDF[0].get(), Constants.TALON_TIMEOUT);
        angleMotor.config_kI(0, anglePIDF[1].get(), Constants.TALON_TIMEOUT);
        angleMotor.config_kD(0, anglePIDF[2].get(), Constants.TALON_TIMEOUT);
        angleMotor.config_kF(0, anglePIDF[3].get(), Constants.TALON_TIMEOUT);
    }

    public boolean hasReachedAngle(double reqAngle){
        return (Math.abs(reqAngle - getAngle()) < DEADBAND);
    }

    @Override
    public void periodic() {
        configPIDF();

        stateSpace = constructLinearSystem(J);
        lastTime = currentTime;
        currentTime = timer.get();
    }

    public void setEncoderAbsolute() {
        angleMotor.configFeedbackNotContinuous(true, Constants.TALON_TIMEOUT);
    }

    public void setEncoderRelative() {
        startAngle = Math.IEEEremainder(angleUnitModel.toUnits(angleMotor.getSelectedSensorPosition() - Constants.SwerveModule.ZERO_POSITIONS[wheel]), 2 * Math.PI);
        angleMotor.configFeedbackNotContinuous(false, Constants.TALON_TIMEOUT);
        angleMotor.setSelectedSensorPosition(0);
    }
}
