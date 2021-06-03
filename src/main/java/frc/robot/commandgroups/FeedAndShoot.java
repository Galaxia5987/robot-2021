package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.PTO.PTO;
import frc.robot.subsystems.PTO.commands.SwitchSubsystems;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.FeedShooter;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;

//import static frc.robot.RobotContainer.velocity;

public class FeedAndShoot extends ParallelCommandGroup {

    public FeedAndShoot(PTO pto, Conveyor conveyor, Shooter shooter, Hood hood, VisionModule vision, double power) {
        addCommands(
                new SwitchSubsystems(pto, false),
                new FeedShooter(conveyor, power),
                new Shoot(shooter, vision, hood, false)
        );
    }
}
