package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.FeedShooter;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.funnel.commands.StartFunnel;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.StartIntake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.ShootWithoutVision;

public class Outtake extends ParallelCommandGroup {
    public Outtake(Funnel funnel, Conveyor conveyor, Shooter shooter) {
        addCommands(
                new StartFunnel(funnel, false),
                new FeedShooter(conveyor, -Constants.Conveyor.CONVEYOR_MOTOR_POWER_SLOW),
                new ShootWithoutVision(shooter, -20)
        );
    }

}
