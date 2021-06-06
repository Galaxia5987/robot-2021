package frc.robot.commandgroups.autonomous;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commandgroups.PickupBalls;
import frc.robot.subsystems.PTO.PTO;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.autonomous.FollowPath;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.ToggleIntake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.VisionModule;

import java.io.IOException;
import java.nio.file.Path;

public class SafeTrenchShot extends SequentialCommandGroup {
    public SafeTrenchShot(SwerveDrive swerve, Intake intake, Funnel funnel, Conveyor conveyor, Shooter shooter, VisionModule vision, PTO pto) {
        Trajectory initiationToSafeTrench = new Trajectory(), safeTrenchToShoot = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(Constants.Autonomous.INITIATION_TO_SAFE_TRENCH_PATH);
            initiationToSafeTrench = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            Path trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(Constants.Autonomous.SAFE_TRENCH_TO_SHOOT_PATH);
            safeTrenchToShoot = TrajectoryUtil.fromPathweaverJson(trajectoryPath2);
        } catch (IOException ex) {
            System.out.println("Couldn't find the autonomous file...");
            ex.printStackTrace();
        }
        addCommands(new FollowPath(swerve, initiationToSafeTrench), new PickupBalls(intake, funnel, conveyor, Constants.Intake.POWER::get, true).withTimeout(2), new ToggleIntake(intake));
//                new FollowPath(swerve, safeTrenchToShoot), new FeedAndShoot(pto, conveyor, shooter, hood, vision, Constants.Conveyor.CONVEYOR_MOTOR_POWER).withTimeout(4));
    }
}
