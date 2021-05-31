package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.FeedShooter;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.funnel.commands.StartFunnel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;

public class FunnelAndShoot extends ParallelCommandGroup {
    public FunnelAndShoot(Hood hood, Shooter shooter, Funnel funnel, Conveyor conveyor, double power) {
        addCommands(
                new FeedShooter(conveyor, power),
                new StartFunnel(funnel, true),
                new ShootAndAdjust(shooter, hood)
        );
    }
}
