package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.Vector;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.UnitModel;
import frc.robot.Utils;
import frc.robot.valuetuner.WebConstant;
import org.techfire225.webapp.FireLog;

import static frc.robot.Constants.Shooter.OMEGA;
import static frc.robot.Constants.Shooter.*;
import static frc.robot.Constants.SwerveDrive.*;
import static frc.robot.Constants.SwerveModule.kT;
import static frc.robot.Constants.SwerveModule.kV;
import static frc.robot.Constants.SwerveModule.*;

/**
 * The Swerve Module Subsystem controls the individual wheel with the controls from the Swerve Drive Subsystem.
 */
public class SwerveModule extends SubsystemBase {
    private final TalonFX driveMotor;
    private final TalonSRX angleMotor;

    private final int wheel;

    private final UnitModel driveUnitModel = new UnitModel(Constants.SwerveDrive.TICKS_PER_METER);
    private final UnitModel angleUnitModel = new UnitModel(Constants.SwerveDrive.TICKS_PER_RAD);
    private final Timer timer = new Timer();
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(Units.feetToMeters(1.0), Units
            .feetToMeters(2.0)); // Max angle motor speed and acceleration.
    private final PIDController anglePID;
    private final WebConstant[] anglePIDF;
    private LinearSystemLoop<N1, N1, N1> stateSpace;
    private LinearSystemLoop<N2, N1, N1> angleStateSpace;
    private double currentTime, lastTime;
    private TrapezoidProfile.State lastProfiledReference = new TrapezoidProfile.State();
    private double lastAngle = 0;

    public SwerveModule(int wheel, int driveMotorPort, int angleMotorPort, boolean[] inverted, WebConstant[] anglePIDF) {
        anglePID = new PIDController(anglePIDF[0].get(), anglePIDF[1].get(), anglePIDF[2].get());
        this.anglePIDF = anglePIDF;
        anglePID.setTolerance(Math.toRadians(2));
        driveMotor = new TalonFX(driveMotorPort);
        angleMotor = new TalonSRX(angleMotorPort);


        // configure feedback sensors
        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, Constants.TALON_TIMEOUT);
        angleMotor.configFeedbackNotContinuous(Ports.SwerveDrive.IS_NOT_CONTINUOUS_FEEDBACK, Constants.TALON_TIMEOUT);
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

        angleMotor.configContinuousCurrentLimit(Constants.SwerveDrive.MAX_CURRENT);
        angleMotor.enableCurrentLimit(Constants.ENABLE_CURRENT_LIMIT);

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
        this.angleStateSpace = constructAngleStateSpace();
        this.stateSpace = constructLinearSystem(Constants.SwerveModule.J.get());
//        Constants.SwerveModule.J.setChangeListener(this::constructLinearSystem);
    }

    private static double getTargetError(double targetAngle, double currentAngle) {
        double option1 = targetAngle - currentAngle;
        double option2 = 2 * Math.PI - (Math.abs(option1));
        if (Math.abs(option1) < option2) {
            return option1;
        } else if (option1 < 0) {
            return option2;
        }
        return -option2;
    }

    /**
     * finds the target angle of the wheel based on the shortest distance from the current position
     *
     * @param angle        the current target angle
     * @param currentAngle the current angle of the wheel
     * @return the target angle
     */
    public static double getTargetAngle(double angle, double currentAngle) {
        // makes sure the value is between -pi and pi
        angle = Utils.floorMod(angle, 2 * Math.PI);
        double[] angles = {angle - 2 * Math.PI, angle, angle + 2 * Math.PI}; // An array of all possible target angles
        double targetAngle = currentAngle;
        double shortestDistance = Double.MAX_VALUE;
        for (double target : angles) { // for each possible angle
            if (Math.abs(target - currentAngle) < shortestDistance) // if the calculated distance is less than the current shortest distance
            {
                shortestDistance = Math.abs(target - currentAngle);
                targetAngle = target;
            }
        }
//        SmartDashboard.putNumber("setpoint", targetAngle);
        return targetAngle;
    }

    private LinearSystemLoop<N2, N1, N1> constructAngleStateSpace() {
        final LinearSystem<N2, N1, N1> stateSpace =
                new LinearSystem<>(
                        Matrix.mat(Nat.N2(), Nat.N2())
                                .fill(
                                        0,
                                        1,
                                        0,
                                        -Math.pow(63.0 / 2720, 2)
                                                * kT_775PRO
                                                / (kV_775PRO * OMEGA_775PRO * J_ANGLE.get())),
                        VecBuilder.fill(0, (63.0 / 2720) * kT_775PRO / (OMEGA_775PRO * J_ANGLE.get())),
                        Matrix.mat(Nat.N1(), Nat.N2()).fill(1, 0),
                        new Matrix<>(Nat.N1(), Nat.N1()));

        final KalmanFilter<N2, N1, N1> kalman =
                new KalmanFilter<>(
                        Nat.N2(),
                        Nat.N1(),
                        stateSpace,
                        VecBuilder.fill(0.015, 0.17), // How accurate we
                        VecBuilder.fill(0.01), // How accurate we think our encoder position
                        0.020);

        final LinearQuadraticRegulator<N2, N1, N1> lqr =
                new LinearQuadraticRegulator<>(
                        stateSpace,
                        VecBuilder.fill(Units.degreesToRadians(1.0), Units.degreesToRadians(1.0)),
                        VecBuilder.fill(12.0),
                        0.020);
        return new LinearSystemLoop<>(stateSpace, lqr, kalman, Constants.NOMINAL_VOLTAGE, Constants.LOOP_PERIOD);
    }

    /**
     * Initialize the linear system to the default values in order to use the state-space.
     *
     * @return an object that represents the model to reach the velocity at the best rate.
     */
    private LinearSystemLoop<N1, N1, N1> constructLinearSystem(double J) {
        if (J == 0) throw new RuntimeException("J must have non-zero value");
        // https://file.tavsys.net/control/controls-engineering-in-frc.pdf Page 76
        Vector<N1> A = VecBuilder.fill(-Math.pow(7.5, 2) * kT / (kV * OMEGA * J));
        Vector<N1> B = VecBuilder.fill((7.5 * kT) / (OMEGA * J));
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

    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeed(), new Rotation2d(getAngle()));
    }

    public void setState(SwerveModuleState state) {
        setAngle(state.angle.getRadians());
        setSpeed(state.speedMetersPerSecond);
    }

    /**
     * @return the speed of the wheel in [m/s]
     */
    public double getSpeed() {
        if (wheel == 1)
            return driveUnitModel.toVelocity(driveMotor.getSelectedSensorVelocity(1));
        return driveUnitModel.toVelocity(driveMotor.getSelectedSensorVelocity(0));
    }

    /**
     * sets the speed of the the wheel in ticks per 100ms
     *
     * @param speed the speed of the wheel in [m/s]
     */
    public void setSpeed(double speed) {
      /*  double timeInterval = Math.max(20, currentTime - lastTime);
        stateSpace.setNextR(VecBuilder.fill(speed)); //r = reference (setpoint)
        stateSpace.correct(VecBuilder.fill(getSpeed()));
        stateSpace.predict(timeInterval);

        double voltageToApply = stateSpace.getU(0); // u = input, calculated by the input.
        // returns the voltage to apply (between -12 and 12)

        FireLog.log("speed", speed);
        driveMotor.set(ControlMode.PercentOutput, voltageToApply / Constants.NOMINAL_VOLTAGE);*/
        driveMotor.set(ControlMode.Velocity, driveUnitModel.toTicks100ms(speed));
    }

    /**
     * @return the angle of the wheel in radians
     */
    public double getAngle() {
        return Math.IEEEremainder(angleUnitModel.toUnits(angleMotor.getSelectedSensorPosition() - Constants.SwerveModule.ZERO_POSITION[wheel]), 2 * Math.PI);
    }

    /**
     * sets the angle of the wheel, in consideration of the shortest path to the target angle
     *
     * @param angle the target angle in radians
     */
    public void setAngle(double angle) {
        double targetAngle = Math.IEEEremainder(angle, 2 * Math.PI);
        double currentAngle = getAngle();
        double error = getTargetError(targetAngle, currentAngle);

        if (targetAngle != lastAngle) {
            angleStateSpace.reset(Matrix.mat(Nat.N2(), Nat.N1()).fill(-error, getAngleMotorVelocity()));
        }
        lastAngle = targetAngle;

        stateSpace.setNextR(0);
//        angleStateSpace.setNextR(0, 0);
        angleStateSpace.correct(VecBuilder.fill(-error)); // TODO: maybe need to be in ticks
        angleStateSpace.predict(currentTime - lastTime);
        double nextVoltage = angleStateSpace.getU(0);
        angleMotor.set(ControlMode.PercentOutput, nextVoltage / Constants.NOMINAL_VOLTAGE);
/*
        double targetAngle = Math.IEEEremainder(angle, 2 * Math.PI);
        double currentAngle = getAngle();
        double error = getTargetError(targetAngle, currentAngle);
        double power = anglePID.calculate(error, 0);
        angleMotor.set(ControlMode.PercentOutput, power);
*/
    }

    public double getAngleMotorVelocity() {
        return angleUnitModel.toVelocity(angleMotor.getSelectedSensorVelocity()) / (2 * Math.PI);
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


        // set PIDF - drive motor
        if (wheel == 0) {
            driveMotor.config_kP(1, Constants.SwerveModule.KP_BROKEN.get(), Constants.TALON_TIMEOUT);
            driveMotor.config_kI(1, Constants.SwerveModule.KI_BROKEN.get(), Constants.TALON_TIMEOUT);
            driveMotor.config_kD(1, Constants.SwerveModule.KD_BROKEN.get(), Constants.TALON_TIMEOUT);
            driveMotor.config_kF(1, Constants.SwerveModule.KF_BROKEN.get(), Constants.TALON_TIMEOUT);
        } else if (wheel != 1) {
            driveMotor.config_kP(1, Constants.SwerveModule.KP_DRIVE.get(), Constants.TALON_TIMEOUT);
            driveMotor.config_kI(1, Constants.SwerveModule.KI_DRIVE.get(), Constants.TALON_TIMEOUT);
            driveMotor.config_kD(1, Constants.SwerveModule.KD_DRIVE.get(), Constants.TALON_TIMEOUT);
            driveMotor.config_kF(1, Constants.SwerveModule.KF_DRIVE.get(), Constants.TALON_TIMEOUT);

        } else {
            driveMotor.config_kP(1, Constants.SwerveModule.KP_DRIVE_SLOW.get(), Constants.TALON_TIMEOUT);
            driveMotor.config_kI(1, Constants.SwerveModule.KI_DRIVE_SLOW.get(), Constants.TALON_TIMEOUT);
            driveMotor.config_kD(1, Constants.SwerveModule.KD_DRIVE_SLOW.get(), Constants.TALON_TIMEOUT);
            driveMotor.config_kF(1, Constants.SwerveModule.KF_DRIVE_SLOW.get(), Constants.TALON_TIMEOUT);
//            System.out.println("P" + Constants.SwerveModule.KP_DRIVE_SLOW.get() + "\nI: " + Constants.SwerveModule.KI_DRIVE_SLOW.get() + "\nD: " + Constants.SwerveModule.KD_DRIVE_SLOW.get() + "\nF: " + Constants.SwerveModule.KF_DRIVE_SLOW.get());
        }
    }

    public void initialize() {
        lastAngle = getAngle();
        lastProfiledReference = new TrapezoidProfile.State(lastAngle, 0);
    }

    @Override
    public void periodic() {
        timer.start();
        anglePID.setP(anglePIDF[0].get());
        anglePID.setI(anglePIDF[1].get());
        anglePID.setD(anglePIDF[2].get());
        lastTime = currentTime;
        currentTime = timer.get();
        this.angleStateSpace = constructAngleStateSpace();
        this.stateSpace = constructLinearSystem(Constants.SwerveModule.J.get());

        FireLog.log("speed" + wheel, getSpeed());
    }
}
