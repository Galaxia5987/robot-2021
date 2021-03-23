package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.PTO.PTO;
import frc.robot.subsystems.PTO.commands.SwitchSubsystems;
import frc.robot.subsystems.climber.Climber;

public class AutoClimb extends SequentialCommandGroup {

    public AutoClimb(PTO pto, Climber climber, double height) {
        addCommands(
                new SwitchSubsystems(pto, true),
                new SetStopper(climber, Climber.PistonMode.OPEN),
                new ManageClimb(climber, height),
                new SetStopper(climber, Climber.PistonMode.CLOSED));
    }
}
