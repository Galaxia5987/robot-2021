package frc.robot.subsystems.PTO.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.PTO.PTO;

/**
 * This command switches between the shooter and the climber subsystems with the PTO.
 */
public class SwitchSubsystems extends InstantCommand {
    private final PTO pto;
    private final boolean isClimber;

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
        pto.changePiston(isClimber);
    }

}
