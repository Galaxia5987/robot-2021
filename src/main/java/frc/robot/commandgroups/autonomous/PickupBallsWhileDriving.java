package frc.robot.commandgroups.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commandgroups.PickupBalls;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.autonomous.MoveForward;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.intake.Intake;

public class PickupBallsWhileDriving extends ParallelCommandGroup {
    public PickupBallsWhileDriving(Intake intake, Funnel funnel, Conveyor conveyor, SwerveDrive swerveDrive) {
        addCommands(
                new PickupBalls(intake, funnel, conveyor, Constants.Intake.POWER::get, true),
                new MoveForward(swerveDrive).withTimeout(2)
        );
    }

}
