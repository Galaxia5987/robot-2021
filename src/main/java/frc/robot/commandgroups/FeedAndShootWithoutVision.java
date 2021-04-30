package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.FeedShooter;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;
import frc.robot.subsystems.shooter.commands.ShootWithoutVision;

public class FeedAndShootWithoutVision extends ParallelCommandGroup {


    public FeedAndShootWithoutVision(Conveyor conveyor, Shooter shooter, double power) {
        addCommands(
                new FeedShooter(conveyor, power),
                new ShootWithoutVision(shooter)
        );
    }

}
