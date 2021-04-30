package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.PTO.PTO;
import frc.robot.subsystems.PTO.commands.SwitchSubsystems;
import frc.robot.subsystems.climber.Climber;

import java.util.function.DoubleSupplier;

public class StartClimb extends SequentialCommandGroup {

    public StartClimb(PTO pto, Climber climber) {
        addCommands(
                new SwitchSubsystems(pto, true),
                new ParallelCommandGroup(new SetStopper(climber, Climber.PistonMode.OPEN),
                        new SetDrum(climber, Climber.PistonMode.OPEN)));
    }
}
