package frc.robot.subsystems.PTO.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.PTO.PTO;

/**
 * This command switches between the shooter and the climber subsystems with the PTO.
 */
public class SwitchSubsystems extends CommandBase {
    private final PTO pto;
    private final boolean isClimber;
    private Timer timer = new Timer();

    /**
     * Here we define the desired state of the PTO.
     *
     * @param pto       the subsystem.
     * @param isClimber whether we want it to be the climber or not.
     */
    public SwitchSubsystems(PTO pto, boolean isClimber) {
        this.pto = pto;
        this.isClimber = isClimber;
        addRequirements(pto);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        pto.getMaster().set(ControlMode.PercentOutput, 0.1);
    }

    @Override
    public void execute() {
        if (timer.get() > 0.5)
            pto.changePiston(isClimber);
    }

    @Override
    public void end(boolean interrupted) {
        pto.getMaster().set(TalonFXControlMode.PercentOutput, 0);
    }

    @Override
    public boolean isFinished() {
        return timer.get() > 0.5;
    }
}
