package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.UnitModel;
import frc.robot.subsystems.PTO.PTO;

/**
 * Climber's configurations and commands.
 */
public class Climber extends SubsystemBase {

    private final UnitModel unitModel = new UnitModel(Constants.Climber.TICKS_PER_METER);
    private final Solenoid stopper = new Solenoid(Ports.Climber.STOPPER_FORWARD_CHANNEL);
    private final Solenoid drum = new Solenoid(Ports.Climber.DRUM);
    private PTO pto;

    public Climber(PTO pto) {

        this.pto = pto;
    }

    /**
     * Get the climber's elevation relative to the ground.
     *
     * @return the climber's height [m].
     */
    public double getHeight() {
        if (pto.getState() == PTO.GearboxState.SHOOTER) {
            return 0;
        }

        return unitModel.toUnits(pto.getMaster().getSelectedSensorPosition());
    }

    /**
     * Raise the climber to a given height.
     *
     * @param height requested height to climb [m].
     */
    public void setHeight(double height) {
        if (pto.getState() == PTO.GearboxState.SHOOTER) {
            return;
        }

        pto.getMaster().set(ControlMode.MotionMagic, unitModel.toTicks(height));
    }

    public double getVelocity() {
        return unitModel.toVelocity(pto.getMaster().getSelectedSensorVelocity());
    }

    /**
     * Set the stopper shifter mode to a given mode.
     *
     * @param mode the wanted stopper shifter mode.
     */
    public void setStopperMode(PistonMode mode) {
        if (mode == PistonMode.OPEN)
            stopper.set(true);
        else
            stopper.set(false);
    }

    public void setDrumMode(PistonMode mode) {
        if (mode == PistonMode.OPEN)
            drum.set(true);
        else
            drum.set(false);
    }


    /**
     * Toggle the piston mode of the piston responsible for the stopper.
     */
    public void toggleStopper() {
        stopper.set(!stopper.get());
    }

    @Override
    public void periodic() {
        if (pto.getState() == PTO.GearboxState.SHOOTER)
            return;
        pto.getMaster().config_kP(0, Constants.Climber.KP, Constants.TALON_TIMEOUT);
        pto.getMaster().config_kI(0, Constants.Climber.KI, Constants.TALON_TIMEOUT);
        pto.getMaster().config_kD(0, Constants.Climber.KD, Constants.TALON_TIMEOUT);
        pto.getMaster().config_kF(0, Constants.Climber.KF, Constants.TALON_TIMEOUT);
//        FireLog.log("Height", getHeight());
//        FireLog.log("Velocity", getVelocity());
//        FireLog.log("Target Velocity", 0);
    }

    public void setPower(double v) {
        pto.getMaster().set(ControlMode.PercentOutput, v);
    }

    /**
     * Enum for piston mode, OPEN is true, CLOSED is false.
     */
    public enum PistonMode {
        OPEN, CLOSED;
    }


}
