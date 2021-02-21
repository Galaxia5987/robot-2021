package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.climber.Climber;

/**
 * Set the gearbox to be open or closed.
 */
public class ShiftGear extends InstantCommand {
    private final Climber climber;
    private Climber.PistonMode mode;

    public ShiftGear(Climber climber, Climber.PistonMode mode) {
        this.climber = climber;
        this.mode = mode;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.setGearboxMode(mode);
    }
}
