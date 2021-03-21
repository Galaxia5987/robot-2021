package frc.robot.subsystems.PTO;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

/**
 * This subsystem define the PTO.
 * The PTO is responsible for switching between the climber and the shooter, because they are using the same motors.
 */
public class PTO extends SubsystemBase {

    private final TalonFX master = new TalonFX(Ports.PTO.MASTER);
    private final TalonFX slave = new TalonFX(Ports.PTO.SLAVE);
    private final DoubleSolenoid piston = new DoubleSolenoid(Ports.PTO.PISTON_FORWARD, Ports.PTO.PISTON_REVERSE);
    private GearboxState state = GearboxState.SHOOTER;

    /**
     * This Configure the motors with the configuration required for the climber.
     */
    public void configureClimber() {
        slave.follow(master);

        master.setInverted(Ports.Climber.IS_MASTER_INVERTED);
        slave.setInverted(Ports.Climber.IS_SLAVE_INVERTED);

        master.setSensorPhase(Ports.Climber.IS_SENSOR_PHASE_INVERTED);

        master.configMotionCruiseVelocity(Constants.Climber.CRUISE_VELOCITY, Constants.TALON_TIMEOUT);
        master.configMotionAcceleration(Constants.Climber.ACCELERATION, Constants.TALON_TIMEOUT);

        master.config_kP(0, Constants.Climber.KP, Constants.TALON_TIMEOUT);
        master.config_kI(0, Constants.Climber.KI, Constants.TALON_TIMEOUT);
        master.config_kD(0, Constants.Climber.KD, Constants.TALON_TIMEOUT);
        master.config_kF(0, Constants.Climber.KF, Constants.TALON_TIMEOUT);

        master.setSelectedSensorPosition(0);
    }

    /**
     * This Configure the motors with the configuration required for the shooter.
     */
    public void configureShooter() {
        // Configure the motors
        master.setInverted(Ports.Shooter.MAIN_INVERTED);
        slave.setInverted(Ports.Shooter.AUX_INVERTED);
        master.setSensorPhase(Ports.Shooter.IS_SENSOR_INVERTED);

        master.setNeutralMode(NeutralMode.Coast);
        slave.setNeutralMode(NeutralMode.Coast);

        master.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
        master.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);

        master.enableVoltageCompensation(true);
        slave.enableVoltageCompensation(true);

        master.configVoltageCompSaturation(Constants.NOMINAL_VOLTAGE);
        slave.configVoltageCompSaturation(Constants.NOMINAL_VOLTAGE);

        slave.follow(master);
    }

    /**
     * This method changes the current state of the gearboxState.
     *
     * @param state the new state.
     */
    private void changeState(GearboxState state) {
        this.state = state;
    }

    /**
     * @return the current GearboxState of the subsystem.
     */
    public GearboxState getState() {
        return state;
    }

    /**
     * This method returns the motor controller for the climber and the shooter to use.
     *
     * @return the motor controller of the master.
     */
    public TalonFX getMaster() {
        return master;
    }

    /**
     * This method returns the motor controller for the climber and the shooter to use.
     *
     * @return the motor controller of the slave.
     */
    public TalonFX getSlave() {
        return slave;
    }

    /**
     * This method switches the piston to change the subsystem.
     * After that this method configure the motors according to the new state and change the current state.
     *
     * @param isClimber whether the new state is the climber subsystem.
     */
    public void changePiston(boolean isClimber) {
        if (isClimber) {
            piston.set(DoubleSolenoid.Value.kForward);
            changeState(GearboxState.CLIMBER);
            configureClimber();
        } else {
            piston.set(DoubleSolenoid.Value.kReverse);
            changeState(GearboxState.SHOOTER);
            configureShooter();
        }
    }

    /**
     * This enum contain the possible states of the PTO:
     * Climber and Shooter.
     */
    public enum GearboxState {
        SHOOTER, CLIMBER
    }
}