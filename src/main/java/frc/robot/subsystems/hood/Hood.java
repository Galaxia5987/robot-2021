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
    private TalonSRX hoodMotor = new TalonSRX(Ports.Shooter.HOOD);

    private State hoodState = State.MIDDLE;

    public Hood() {
        hoodMotor.setInverted(Ports.Shooter.IS_HOOD_INVERTED);

        hoodMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Constants.TALON_TIMEOUT);
        hoodMotor.setSensorPhase(Ports.Shooter.IS_HOOD_SENSOR_INVERTED);
        hoodMotor.config_kP(0, Constants.Shooter.HOOD_KP.get(), Constants.TALON_TIMEOUT);
        hoodMotor.config_kI(0, Constants.Shooter.HOOD_KI.get(), Constants.TALON_TIMEOUT);
        hoodMotor.config_kD(0, Constants.Shooter.HOOD_KD.get(), Constants.TALON_TIMEOUT);
        hoodMotor.config_kF(0, HOOD_KF.get(), Constants.TALON_TIMEOUT);

        hoodMotor.configMotionCruiseVelocity(unitModel.toTicks100ms(Constants.Shooter.CRUISE_VELOCITY), Constants.TALON_TIMEOUT);
        hoodMotor.configMotionAcceleration(unitModel.toTicks100ms(Constants.Shooter.ACCELERATION), Constants.TALON_TIMEOUT);

        hoodMotor.enableVoltageCompensation(true);
        hoodMotor.configVoltageCompSaturation(Constants.NOMINAL_VOLTAGE);
    }

    /**
     * Estimate the velocity that the shooter should apply in order to reach the target.
     *
     * @param distance the distance from the target. [meters]
     * @return the velocity that should be applied by the shooter in order to reach the target.[RPS]
     */
    public double estimateVelocityFromDistance(double distance) {
        return hoodState.velocityEstimator.estimateVelocityFromDistance(distance);
    }


    /**
     * Change the state of the hood to another desired state.
     *
     * @param newState the new state of the hood.
     */
    public void changeHoodState(State newState) {
        if (newState.position < MAX_HOOD_POSITION && newState.position > MIN_HOOD_POSITION) {
            double gain = 0.02;
            if (STUCK_POSITION - getHoodPosition() > 0) {
                gain *= -1;
            }
//            hoodMotor.set(ControlMode.MotionMagic, newState.position, DemandType.ArbitraryFeedForward,
//                    Constants.Shooter.HOOD_ARBITRARY_KF.get() / ((STUCK_POSITION - getHoodPosition())/ MAX_HOOD_POSITION + gain));
            hoodMotor.set(ControlMode.MotionMagic, newState.position);
            this.hoodState = newState;
            System.out.println(Constants.Shooter.HOOD_ARBITRARY_KF.get() / ((STUCK_POSITION - getHoodPosition())/ MAX_HOOD_POSITION + gain));
        }
    }

    public void setHoodPID() {
        hoodMotor.config_kP(0, Constants.Shooter.HOOD_KP.get(), Constants.TALON_TIMEOUT);
        hoodMotor.config_kI(0, Constants.Shooter.HOOD_KI.get(), Constants.TALON_TIMEOUT);
        hoodMotor.config_kD(0, Constants.Shooter.HOOD_KD.get(), Constants.TALON_TIMEOUT);
        hoodMotor.config_kF(0, HOOD_KF.get(), Constants.TALON_TIMEOUT);
    }


    public double getHoodVelocity() {
        return unitModel.toVelocity(hoodMotor.getSelectedSensorVelocity());
    }

    public double getHoodPosition() {
        return hoodMotor.getSelectedSensorPosition();
    }
    public void stopHood() {
        hoodMotor.set(ControlMode.PercentOutput, 0);
    }
    public enum State {
        LOW(MIN_HOOD_POSITION + 200, new DoubleRange(0, 0), "/Low.csv"),
        MIDDLE(MIN_HOOD_POSITION + 3500, new DoubleRange(0, 0), "/Middle.csv"),
        HIGH(MAX_HOOD_POSITION - 200, new DoubleRange(0, 0), "/High.csv");

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
            State current = LOW;
            double minVelocity = LOW.velocityEstimator.estimateVelocityFromDistance(distance);
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
