package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.funnel.Funnel;

public class Load extends SequentialCommandGroup {
    public Load(Conveyor conveyor, Funnel funnel) {
        addCommands(
                new InstantCommand(() -> funnel.setPower(0)),
                new StartConveyor(conveyor).withTimeout(0.5),
                new TransferBall(funnel)
        );
    }
}
