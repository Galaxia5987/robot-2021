package frc.robot.subsystems.hood;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.UnitModel;
import frc.robot.subsystems.shooter.LinearRegression;
import org.apache.commons.lang.math.DoubleRange;

import java.io.InputStream;
import java.io.InputStreamReader;

import static frc.robot.Constants.Shooter.*;

public class Hood extends SubsystemBase {
    private final UnitModel unitModel = new UnitModel(TICKS_PER_ROTATION);
    private final TalonSRX motor = new TalonSRX(Ports.Hood.MOTOR);
    private State state = State.CLOSED;

    public Hood() {
        motor.setInverted(Ports.Hood.IS_INVERTED);

        motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Constants.TALON_TIMEOUT);
        motor.setSensorPhase(Ports.Hood.IS_SENSOR_INVERTED);
        motor.config_kP(0, Constants.Hood.KP.get(), Constants.TALON_TIMEOUT);
        motor.config_kI(0, Constants.Hood.KI.get(), Constants.TALON_TIMEOUT);
        motor.config_kD(0, Constants.Hood.KD.get(), Constants.TALON_TIMEOUT);
        motor.config_kF(0, Constants.Hood.KF.get(), Constants.TALON_TIMEOUT);

        motor.configMotionCruiseVelocity(unitModel.toTicks100ms(Constants.Hood.CRUISE_VELOCITY), Constants.TALON_TIMEOUT);
        motor.configMotionAcceleration(unitModel.toTicks100ms(Constants.Hood.ACCELERATION), Constants.TALON_TIMEOUT);

        motor.enableVoltageCompensation(true);
        motor.configVoltageCompSaturation(Constants.NOMINAL_VOLTAGE);
    }

    /**
     * Estimate the velocity that the shooter should apply in order to reach the target.
     *
     * @param distance the distance from the target. [meters]
     * @return the velocity that should be applied by the shooter in order to reach the target.[RPS]
     */
    public double estimateVelocityFromDistance(double distance) {
        return state.velocityEstimator.estimateVelocityFromDistance(distance);
    }


    /**
     * Change the state of the hood to another desired state.
     *
     * @param newState the new state of the hood.
     */
    public void changeState(State newState) {
        if (newState.position < Constants.Hood.MAX_POSITION && newState.position > Constants.Hood.MIN_POSITION) {
            double gain = 0.02;
            if (Constants.Hood.STUCK_POSITION - getPosition() > 0) {
                gain *= -1;
            }
//yy            hoodMotor.set(ControlMode.MotionMagic, newState.position, DemandType.ArbitraryFeedForward,
//                    Constants.Hood.HOOD_ARBITRARY_KF.get() / ((STUCK_POSITION - getHoodPosition())/ MAX_HOOD_POSITION + gain));
            motor.set(ControlMode.MotionMagic, newState.position);
            this.state = newState;
            System.out.println(Constants.Hood.ARBITRARY_KF.get() / ((Constants.Hood.STUCK_POSITION - getPosition()) / Constants.Hood.MAX_POSITION + gain));
        }
    }

    public State getState() {
        return state;
    }

    public void updatePID() {
        motor.config_kP(0, Constants.Hood.KP.get(), Constants.TALON_TIMEOUT);
        motor.config_kI(0, Constants.Hood.KI.get(), Constants.TALON_TIMEOUT);
        motor.config_kD(0, Constants.Hood.KD.get(), Constants.TALON_TIMEOUT);
        motor.config_kF(0, Constants.Hood.KF.get(), Constants.TALON_TIMEOUT);
    }


    public double getVelocity() {
        return unitModel.toVelocity(motor.getSelectedSensorVelocity());
    }

    public double getPosition() {
        return motor.getSelectedSensorPosition();
    }

    public void stop() {
        motor.set(ControlMode.PercentOutput, 0);
    }

    public enum State {
        CLOSED(Constants.Hood.MIN_POSITION + 200, new DoubleRange(0, 0), "/Low.csv"),
        LOW(Constants.Hood.MIN_POSITION + 1500, new DoubleRange(0, 0), "/Low.csv"),
        MIDDLE(Constants.Hood.MIN_POSITION + 3500, new DoubleRange(0, 0), "/Middle.csv"),
        HIGH(Constants.Hood.MIN_POSITION + 4000, new DoubleRange(0, 0), "/High.csv"),
        OPEN(Constants.Hood.MAX_POSITION - 200, new DoubleRange(0, 0), "/High.csv");

        public final DoubleRange shootingRange; // [min, max] meters
        public final LinearRegression velocityEstimator;
        public final int position; //[ticks]

        State(int position, DoubleRange range, String pathToCsv) {
            this.position = position;
            this.shootingRange = range;
            this.velocityEstimator = new LinearRegression(readCSV(pathToCsv));
        }

        /**
         * Get the optimal state in order to shoot.
         *
         * @param distance the distance from the target. [m]
         * @return the optimal state.
         */
        public static State getOptimalState(double distance) {
            State current = CLOSED;
            double minVelocity = CLOSED.velocityEstimator.estimateVelocityFromDistance(distance);
            for (State state : State.values()) {
                double currentVelocity = state.velocityEstimator.estimateVelocityFromDistance(distance);
                if (currentVelocity < minVelocity) {
                    minVelocity = currentVelocity;
                    current = state;
                }
            }
            return current;
        }

        /**
         * Internal function to create an input stream reader, in order values from the shooting experiments.
         *
         * @return a reader to the CSV.
         */
        private InputStreamReader readCSV(String path) {
            InputStream is = getClass().getResourceAsStream(path);
            assert is != null : "Can't create input stream";

            return new InputStreamReader(is);
        }
    }
}
