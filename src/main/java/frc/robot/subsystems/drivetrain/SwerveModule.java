package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.UnitModel;
import frc.robot.Utils;

public class SwerveModule extends SubsystemBase {
    private final TalonFX driveMotor;
    private final TalonSRX angleMotor;
    private final int wheel;

    private final UnitModel driveUnitModel = new UnitModel(Constants.SwerveDrive.TICKS_PER_METER);
    private final UnitModel angleUnitModel = new UnitModel(Constants.SwerveDrive.TICKS_PER_RAD);

    public SwerveModule(int wheel, int driveMotorPort, int angleMotorPort, boolean[] inverted) {

        driveMotor = new TalonFX(driveMotorPort);
        angleMotor = new TalonSRX(angleMotorPort);

        // configure feedback sensors
        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, Constants.TALON_TIMEOUT);
        angleMotor.configFeedbackNotContinuous(Ports.SwerveDrive.IS_NOT_CONTINUOUS_FEEDBACK, Constants.TALON_TIMEOUT);
        resetAngleEncoder();
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

        this.wheel = wheel;
    }


    /**
     * @return the speed of the wheel in [m/s]
     */
    public double getSpeed() {
        return driveUnitModel.toVelocity(driveMotor.getSelectedSensorVelocity(1));
    }

    /**
     * @return the angle of the wheel in radians
     */
    public double getAngle() {
        return angleUnitModel.toUnits(angleMotor.getSelectedSensorPosition() - Constants.SwerveModule.ZERO_POSITION[wheel]);
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
     * sets the angle of the wheel, in consideration of the shortest path to the target angle
     *
     * @param angle the target angle in radians
     */
    public void setAngle(double angle) {
        double targetAngle = getTargetAngle(angle, getAngle());
        int angleTicks = angleUnitModel.toTicks(targetAngle);
        angleMotor.set(ControlMode.Position, angleTicks);
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

        return targetAngle;
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
        angleMotor.setSelectedSensorPosition(Constants.SwerveModule.ZERO_POSITION[wheel]);
    }

    /**
     * config PIDF for the angle motor and drive motor
     */
    private void configPIDF() {
        // set PIDF - angle motor
        if (wheel != 2) {
            angleMotor.config_kP(0, Constants.SwerveModule.KP.get(), Constants.TALON_TIMEOUT);
            angleMotor.config_kI(0, Constants.SwerveModule.KI.get(), Constants.TALON_TIMEOUT);
            angleMotor.config_kD(0, Constants.SwerveModule.KD.get(), Constants.TALON_TIMEOUT);
            angleMotor.config_kF(0, Constants.SwerveModule.KF.get(), Constants.TALON_TIMEOUT);
        } else {
            angleMotor.config_kP(0, Constants.SwerveModule.KP_SICK.get(), Constants.TALON_TIMEOUT);
            angleMotor.config_kI(0, Constants.SwerveModule.KI_SICK.get(), Constants.TALON_TIMEOUT);
            angleMotor.config_kD(0, Constants.SwerveModule.KD_SICK.get(), Constants.TALON_TIMEOUT);
            angleMotor.config_kF(0, Constants.SwerveModule.KF_SICK.get(), Constants.TALON_TIMEOUT);
        }

        if (wheel != 1) {
            driveMotor.config_kP(1, Constants.SwerveModule.KP_DRIVE.get(), Constants.TALON_TIMEOUT);
            driveMotor.config_kI(1, Constants.SwerveModule.KI_DRIVE.get(), Constants.TALON_TIMEOUT);
            driveMotor.config_kD(1, Constants.SwerveModule.KD_DRIVE.get(), Constants.TALON_TIMEOUT);
            driveMotor.config_kF(1, Constants.SwerveModule.KF_DRIVE.get(), Constants.TALON_TIMEOUT);
        } else {
            driveMotor.config_kP(1, Constants.SwerveModule.KP_DRIVE_SLOW.get(), Constants.TALON_TIMEOUT);
            driveMotor.config_kI(1, Constants.SwerveModule.KI_DRIVE_SLOW.get(), Constants.TALON_TIMEOUT);
            driveMotor.config_kD(1, Constants.SwerveModule.KD_DRIVE_SLOW.get(), Constants.TALON_TIMEOUT);
            driveMotor.config_kF(1, Constants.SwerveModule.KF_DRIVE_SLOW.get(), Constants.TALON_TIMEOUT);
        }
    }
}
