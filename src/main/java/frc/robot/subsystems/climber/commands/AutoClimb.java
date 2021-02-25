package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climber.Climber;

public class AutoClimb extends SequentialCommandGroup {

    public AutoClimb(Climber climber, double height) {
        addCommands(new ShiftGear(climber, Climber.PistonMode.OPEN),
                new SetStopper(climber, Climber.PistonMode.OPEN),
                new ManageClimb(climber, height),
                new SetStopper(climber, Climber.PistonMode.CLOSED));
    }
}
