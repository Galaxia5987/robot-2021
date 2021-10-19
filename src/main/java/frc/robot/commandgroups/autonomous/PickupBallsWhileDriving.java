package frc.robot.commandgroups.autonomous;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commandgroups.PickupBalls;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.autonomous.FollowPath;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.intake.Intake;

import java.io.IOException;
import java.nio.file.Path;

public class PickupBallsWhileDriving extends ParallelCommandGroup {
    public PickupBallsWhileDriving(Intake intake, Funnel funnel, Conveyor conveyor, SwerveDrive swerveDrive) {
        Trajectory pickupBalls = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(Constants.Autonomous.PICKUP_BALLS_PATH);
            pickupBalls = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            System.out.println("Couldn't find the autonomous file...");
            ex.printStackTrace();
        }
        addCommands(
                new PickupBalls(intake, funnel, conveyor, () -> Constants.Intake.POWER, true),
                new FollowPath(swerveDrive, pickupBalls)
        );
    }

}
