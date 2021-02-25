package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.UnitModel;

/**
 * Climber's configurations and commands.
 */
public class Climber extends SubsystemBase {
    private final TalonFX master = new TalonFX(Ports.Climber.MASTER);
    private final TalonFX slave = new TalonFX(Ports.Climber.SLAVE);
    private final UnitModel unitModel = new UnitModel(Constants.Climber.TICKS_PER_METER);
    private final DoubleSolenoid stopper = new DoubleSolenoid(Ports.Climber.STOPPER_FORWARD_CHANNEL, Ports.Climber.STOPPER_REVERSE_CHANNEL);
    private final DoubleSolenoid gearboxShifter = new DoubleSolenoid(Ports.Climber.GEARBOX_FORWARD_CHANNEL, Ports.Climber.GEARBOX_REVERSE_CHANNEL);

    public Climber() {
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
    }

    /**
     * Get the climber's elevation relative to the ground.
     *
     * @return the climber's height [m].
     */
    public double getHeight() {
        return unitModel.toUnits(master.getSelectedSensorPosition());
    }

    /**
     * Raise the climber to a given height.
     *
     * @param height requested height to climb [m].
     */
    public void setHeight(double height) {
        master.set(ControlMode.MotionMagic, unitModel.toTicks(height));
    }

    /**
     * Set the stopper mode.
     *
     * @param mode the stopper mode.
     */
    public void setStopperMode(PistonMode mode) {
        if (mode == PistonMode.OPEN)
            stopper.set(DoubleSolenoid.Value.kForward);
        else
            stopper.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * Set the gearbox shifter mode to a given mode.
     *
     * @param mode the wanted gearbox shifter mode.
     */
    public void setGearboxMode(PistonMode mode) {
        if (mode == PistonMode.OPEN)
            gearboxShifter.set(DoubleSolenoid.Value.kForward);
        else
            gearboxShifter.set(DoubleSolenoid.Value.kReverse);

    }

    /**
     * Toggle the piston mode of the piston responsible for the gearbox.
     */
    public void toggleGear() {
        if (gearboxShifter.get() == DoubleSolenoid.Value.kReverse)
            gearboxShifter.set(DoubleSolenoid.Value.kForward);
        else
            gearboxShifter.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * Toggle the piston mode of the piston responsible for the stopper.
     */
    public void toggleStopper() {
        if (stopper.get() == DoubleSolenoid.Value.kReverse)
            stopper.set(DoubleSolenoid.Value.kForward);
        else
            stopper.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * Enum for piston mode, OPEN is true, CLOSED is false.
     */
    public enum PistonMode {
        OPEN, CLOSED;
    }
}
