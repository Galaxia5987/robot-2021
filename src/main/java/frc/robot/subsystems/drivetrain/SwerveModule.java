package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.UnitModel;
import frc.robot.valuetuner.WebConstant;

/**
 * The Swerve Module Subsystem controls the individual wheel with the controls from the Swerve Drive Subsystem.
 */
public class SwerveModule extends SubsystemBase {
    private final TalonFX driveMotor;
    private final TalonSRX angleMotor;

    private final int wheel;

    private final UnitModel driveUnitModel = new UnitModel(Constants.SwerveDrive.TICKS_PER_METER);
    private final UnitModel angleUnitModel = new UnitModel(Constants.SwerveDrive.TICKS_PER_RAD);
    private final PIDController anglePID;
    private final WebConstant[] anglePIDF;
    private final double startAngle;

    public SwerveModule(int wheel, int driveMotorPort, int angleMotorPort, boolean[] inverted, WebConstant[] anglePIDF) {
        anglePID = new PIDController(anglePIDF[0].get(), anglePIDF[1].get(), anglePIDF[2].get());
        this.anglePIDF = anglePIDF;
        anglePID.setTolerance(Math.toRadians(0.5));
        driveMotor = new TalonFX(driveMotorPort);
        angleMotor = new TalonSRX(angleMotorPort);


        // configure feedback sensors
        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, Constants.TALON_TIMEOUT);

        setEncoderMode(true);
        startAngle = getAngle();
        setEncoderMode(false);

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
    }

    private static double getTargetError(double angle, double currentAngle) {
        double cwDistance = angle - currentAngle;
        double ccwDistance = 2 * Math.PI - (Math.abs(cwDistance));
        if (Math.abs(cwDistance) < ccwDistance) {
            return -cwDistance;
        } else if (cwDistance < 0) {
            return -ccwDistance;
        }
        return ccwDistance;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeed(), new Rotation2d(getAngle()));
    }

    public void setState(SwerveModuleState state) {
       /* double targetAngle = Math.IEEEremainder(state.angle.getRadians(), 2 * Math.PI);
        double currentAngle = getAngle();
        double error = getTargetError(targetAngle, currentAngle);
        double power = anglePID.calculate(error, 0);
        angleMotor.set(ControlMode.PercentOutput, power);
       */
        driveMotor.set(ControlMode.Velocity, driveUnitModel.toTicks100ms(state.speedMetersPerSecond));
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
        driveMotor.set(ControlMode.Velocity, driveUnitModel.toTicks100ms(speed));
    }

    /**
     * @return the angle of the wheel in radians
     */
    public double getAngle() {
        return Math.IEEEremainder(angleUnitModel.toUnits(angleMotor.getSelectedSensorPosition() - Constants.SwerveModule.ZERO_POSITION[wheel]) + startAngle, 2 * Math.PI);
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
        angleMotor.set(ControlMode.Position, angleMotor.getSelectedSensorPosition() - angleUnitModel.toTicks(error));
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
        if (wheel == 1) {
            driveMotor.config_kP(1, Constants.SwerveModule.KP_BROKEN, Constants.TALON_TIMEOUT);
            driveMotor.config_kI(1, Constants.SwerveModule.KI_BROKEN, Constants.TALON_TIMEOUT);
            driveMotor.config_kD(1, Constants.SwerveModule.KD_BROKEN, Constants.TALON_TIMEOUT);
            driveMotor.config_kF(1, Constants.SwerveModule.KF_BROKEN, Constants.TALON_TIMEOUT);
        } else if (wheel != 0) {
            driveMotor.config_kP(1, Constants.SwerveModule.KP_DRIVE, Constants.TALON_TIMEOUT);
            driveMotor.config_kI(1, Constants.SwerveModule.KI_DRIVE, Constants.TALON_TIMEOUT);
            driveMotor.config_kD(1, Constants.SwerveModule.KD_DRIVE, Constants.TALON_TIMEOUT);
            driveMotor.config_kF(1, Constants.SwerveModule.KF_DRIVE, Constants.TALON_TIMEOUT);

        } else {
            driveMotor.config_kP(1, Constants.SwerveModule.KP_DRIVE_SLOW, Constants.TALON_TIMEOUT);
            driveMotor.config_kI(1, Constants.SwerveModule.KI_DRIVE_SLOW, Constants.TALON_TIMEOUT);
            driveMotor.config_kD(1, Constants.SwerveModule.KD_DRIVE_SLOW, Constants.TALON_TIMEOUT);
            driveMotor.config_kF(1, Constants.SwerveModule.KF_DRIVE_SLOW, Constants.TALON_TIMEOUT);
//            System.out.println("P" + Constants.SwerveModule.KP_DRIVE_SLOW.get() + "\nI: " + Constants.SwerveModule.KI_DRIVE_SLOW.get() + "\nD: " + Constants.SwerveModule.KD_DRIVE_SLOW.get() + "\nF: " + Constants.SwerveModule.KF_DRIVE_SLOW.get());
        }
    }

    @Override
    public void periodic() {
        configPIDF();
        anglePID.setP(anglePIDF[0].get());
        anglePID.setI(anglePIDF[1].get());
        anglePID.setD(anglePIDF[2].get());
    }

    public void setEncoderMode(boolean absolute) {
        angleMotor.configFeedbackNotContinuous(absolute, Constants.TALON_TIMEOUT);
    }
}
