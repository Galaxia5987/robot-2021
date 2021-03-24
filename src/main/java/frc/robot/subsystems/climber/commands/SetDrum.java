package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.climber.Climber;

public class SetDrum extends InstantCommand {
    private Climber climber;
    private Climber.PistonMode mode;

    public SetDrum(Climber climber, Climber.PistonMode mode) {
        this.climber = climber;
        this.mode = mode;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.setDrumMode(mode);
    }

}
