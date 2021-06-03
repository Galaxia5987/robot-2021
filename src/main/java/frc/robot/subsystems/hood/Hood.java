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
import org.techfire225.webapp.FireLog;

import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.Arrays;
import java.util.Comparator;
import java.util.function.IntSupplier;

import static frc.robot.Constants.Shooter.TICKS_PER_ROTATION;

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
    public void changePosition(State newState) {
        if (newState.position.getAsInt() < Constants.Hood.MAX_POSITION && newState.position.getAsInt() > Constants.Hood.MIN_POSITION) {
            double gain = 0.02;
            if (Constants.Hood.STUCK_POSITION - getPosition() > 0) {
                gain *= -1;
            }
//yy            hoodMotor.set(ControlMode.MotionMagic, newState.position.getAsInt(), DemandType.ArbitraryFeedForward,
//                    Constants.Hood.HOOD_ARBITRARY_KF.get() / ((STUCK_POSITION - getHoodPosition())/ MAX_HOOD_POSITION + gain));
            motor.set(ControlMode.MotionMagic, newState.position.getAsInt());
            this.state = newState;
            System.out.println(Constants.Hood.ARBITRARY_KF.get() / ((Constants.Hood.STUCK_POSITION - getPosition()) / Constants.Hood.MAX_POSITION + gain));
        }
    }

    public void changePosition(int position) {
        if (position < Constants.Hood.MAX_POSITION && position > Constants.Hood.MIN_POSITION) {
            double gain = 0.02;
            if (Constants.Hood.STUCK_POSITION - getPosition() > 0) {
                gain *= -1;
            }
            motor.set(ControlMode.MotionMagic, position);
//            this.state = newState;
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

    public void resetPosition() {
//        Constants.Hood.MIN_POSITION = motor.getSelectedSensorPosition();
    }

    @Override
    public void periodic() {
        super.periodic();

        FireLog.log("position", getPosition());
    }

    public enum State {
        CLOSED(Constants.Hood.MIN_POSITION + 200, new DoubleRange(0, 0), "/Low.csv"),
        LOW(Constants.Hood.MIN_POSITION + 1500, new DoubleRange(0.75, 1.7), "/Low.csv"),
        MIDDLE_LOW(Constants.Hood.MIN_POSITION + 2500, new DoubleRange(1.7, 2.5), "/Middle_Low.csv"),
        MIDDLE(Constants.Hood.MIN_POSITION + 3000, new DoubleRange(2.5, 3.3), "/Middle.csv"),
        HIGH(Constants.Hood.MIN_POSITION + 3450, new DoubleRange(3.3, 4.5), "/High.csv"),
        ABOVE_HIGH(() -> Constants.Hood.MIN_POSITION + 3750, new DoubleRange(4.5, 6), "/Above_High.csv"),
        OPEN(Constants.Hood.MAX_POSITION - 200, new DoubleRange(0, 0), "/High.csv");

        public final DoubleRange shootingRange; // [min, max] meters
        public final LinearRegression velocityEstimator;
        public final IntSupplier position; //[ticks]

        State(int position, DoubleRange range, String pathToCsv) {
            this(() -> position, range, pathToCsv);
        }

        State(IntSupplier position, DoubleRange range, String pathToCsv) {
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
            var filtered = Arrays.stream(State.values())
                    .filter(state -> state.shootingRange.getMinimumDouble() < distance && state.shootingRange.getMaximumDouble() > distance)
                    .min(Comparator.comparingDouble(state -> state.velocityEstimator.estimateVelocityFromDistance(distance)));
            return filtered.orElse(State.CLOSED);
        /*    State current = CLOSED;
            double minVelocity = CLOSED.velocityEstimator.estimateVelocityFromDistance(distance);
            for (State state : State.values()) {
                double currentVelocity = state.velocityEstimator.estimateVelocityFromDistance(distance);
                if (currentVelocity < minVelocity) {
                    minVelocity = currentVelocity;
                    current = state;
                }
            }
            return current;
        */
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
