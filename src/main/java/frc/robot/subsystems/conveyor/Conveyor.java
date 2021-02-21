package frc.robot.subsystems.conveyor;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.utils.DeadbandLaserSensor;

import static frc.robot.Constants.Conveyor.*;

public class Conveyor extends SubsystemBase {
    private static int balls = Constants.Conveyor.INITIAL_BALLS_AMOUNT;

    private final TalonFX motor = new TalonFX(Ports.Conveyor.MOTOR);
    private final DeadbandLaserSensor shooterSensor = new DeadbandLaserSensor(Ports.Conveyor.SHOOTER_LASER_SENSOR,
            SHOOTER_PROXIMITY_LOST_VOLTAGE, SHOOTER_PROXIMITY_SENSE_VOLTAGE);
    private final DeadbandLaserSensor funnelSensor = new DeadbandLaserSensor(Ports.Conveyor.FUNNEL_LASER_SENSOR,
            FUNNEL_PROXIMITY_LOST_VOLTAGE, FUNNEL_PROXIMITY_SENSE_VOLTAGE);

    public Conveyor() {
        motor.setInverted(Ports.Conveyor.IS_MOTOR_INVERTED);

        motor.configPeakOutputForward(FORWARD_PEAK, Constants.TALON_TIMEOUT);
        motor.configPeakOutputReverse(REVERSE_PEAK, Constants.TALON_TIMEOUT);

        motor.configSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT, Constants.TALON_TIMEOUT);
        motor.configStatorCurrentLimit(STATOR_CURRENT_LIMIT, Constants.TALON_TIMEOUT);

        motor.enableVoltageCompensation(true);
        motor.configVoltageCompSaturation(Constants.NOMINAL_VOLTAGE);
    }

    /**
     * Get the amount of balls sensed by the proximity.
     *
     * @return the amount of balls in the conveyor.
     */
    private static int getBallsAmount() {
        return balls;
    }

    public static boolean isConveyorFull() {
        return getBallsAmount() >= MAX_BALLS_AMOUNT;
    }

    public static boolean isConveyorEmpty() {
        return getBallsAmount() <= 0;
    }

    /**
     * Increment by one the amount of balls (only programmatically).
     */
    private static void addBall() {
        balls++;
    }

    /**
     * Decrement by one the amount of balls (only programmatically).
     */
    private static void removeBall() {
        balls--;
    }

    /**
     * Get whether the power applied by the motor is upward, i.e moving up.
     *
     * @return whether the power applied by the motor is upward.
     */
    private boolean isMovingUp() {
        return motor.getMotorOutputPercent() > 0;
    }

    /**
     * Set the power to apply by the motor of the conveyor.
     *
     * @param power the power to apply by the motor. [%]
     */
    public void setPower(double power) {
        motor.set(TalonFXControlMode.PercentOutput, power);
    }

    /**
     * Stop the motor.
     */
    public void stop() {
        setPower(0);
    }


    /**
     * Get whether the funnel sensed an object.
     *
     * @return whether the funnel sensed an object.
     */
    public boolean hasFunnelSensedObject() {
        return funnelSensor.hasObjectSensed();
    }

    @Override
    public void periodic() {
        shooterSensor.updateState();
        funnelSensor.updateState();

        // An object is being taken out either from the shooter or from the funnel.
        if (!shooterSensor.hasObjectSensed() && shooterSensor.hasStateChanged() && isMovingUp() ||
                !funnelSensor.hasObjectSensed() && funnelSensor.hasStateChanged() && !isMovingUp()) {
            Conveyor.removeBall();
        }

        // An object is being inserted into the conveyor.
        if (funnelSensor.hasObjectSensed() && funnelSensor.hasStateChanged() && isMovingUp()) {
            Conveyor.addBall();
        }
    }
}
