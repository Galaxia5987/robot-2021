package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.FeedShooter;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.MoveToPosition;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.funnel.commands.StartFunnel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.VisionModule;

public class ConveyorShooter extends ParallelCommandGroup {
    public ConveyorShooter(Shooter shooter, Hood hood, Conveyor conveyor, Funnel funnel, SwerveDrive swerve, VisionModule vision, double power) {
        addCommands(
                new SequentialCommandGroup(
                        new MoveToPosition(swerve, vision),
                        new ShootAndAdjust(shooter, vision, hood, false)
                ),
                new FeedShooter(conveyor, power),
                new StartFunnel(funnel, true)
        );
    }
}
