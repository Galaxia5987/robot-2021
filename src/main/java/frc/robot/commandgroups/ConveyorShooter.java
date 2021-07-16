package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.FeedShooter;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.funnel.commands.StartFunnel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.VisionModule;

public class ConveyorShooter extends ParallelCommandGroup {
    public ConveyorShooter(Shooter shooter, Hood hood, Conveyor conveyor, Funnel funnel, VisionModule vision, double power, boolean manual) {
        addCommands(
                new SequentialCommandGroup(
                        new WaitUntilCommand(() -> shooter.hasReachedSetpoint(hood.estimateVelocityFromDistance(vision.getTargetRawDistance().orElse(0)))),
                        new ParallelCommandGroup(
                                new FeedShooter(conveyor, power),
                                new StartFunnel(funnel, true)
                        )
                ),
                new ShootAndAdjust(shooter, vision, hood, manual)
        );
    }
}
