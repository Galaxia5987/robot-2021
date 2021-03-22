package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PTO.PTO;
import frc.robot.subsystems.UnitModel;

/**
 * Climber's configurations and commands.
 */
public class Climber extends SubsystemBase {

    private final UnitModel unitModel = new UnitModel(Constants.Climber.TICKS_PER_METER);
    private final Solenoid stopper = new Solenoid(Ports.Climber.STOPPER_REVERSE_CHANNEL);
    private final Solenoid gearboxShifter = new Solenoid(Ports.Climber.GEARBOX_REVERSE_CHANNEL);

    /**
     * Get the climber's elevation relative to the ground.
     *
     * @return the climber's height [m].
     */
    public double getHeight() {
        if (RobotContainer.pto.getState() == PTO.GearboxState.SHOOTER) {
            return 0;
        }

        return unitModel.toUnits(RobotContainer.pto.getMaster().getSelectedSensorPosition());
    }

    /**
     * Raise the climber to a given height.
     *
     * @param height requested height to climb [m].
     */
    public void setHeight(double height) {
        if (RobotContainer.pto.getState() == PTO.GearboxState.SHOOTER) {
            return;
        }

        RobotContainer.pto.getMaster().set(ControlMode.MotionMagic, unitModel.toTicks(height));
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

    /**
     * Set the gearbox shifter mode to a given mode.
     *
     * @param mode the wanted gearbox shifter mode.
     */
    public void setGearboxMode(PistonMode mode) {
        if (mode == PistonMode.OPEN)
            gearboxShifter.set(true);
        else
            gearboxShifter.set(false);

    }

    /**
     * Toggle the piston mode of the piston responsible for the gearbox.
     */
    public void toggleGear() {
        if (gearboxShifter.get())
            gearboxShifter.set(true);
        else
            gearboxShifter.set(false);
    }

    /**
     * Toggle the piston mode of the piston responsible for the stopper.
     */
    public void toggleStopper() {
        if (!stopper.get())
            stopper.set(true);
        else
            stopper.set(false);
    }

    /**
     * Enum for piston mode, OPEN is true, CLOSED is false.
     */
    public enum PistonMode {
        OPEN, CLOSED;
    }
}
