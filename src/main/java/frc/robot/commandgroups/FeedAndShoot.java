package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.FeedShooter;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;

import static frc.robot.RobotContainer.velocity;

public class FeedAndShoot extends ParallelCommandGroup {

    public FeedAndShoot(Conveyor conveyor, Shooter shooter, double power) {
        addCommands(
                new FeedShooter(conveyor, power),
                new Shoot(shooter, velocity::get)
        );
    }
}
