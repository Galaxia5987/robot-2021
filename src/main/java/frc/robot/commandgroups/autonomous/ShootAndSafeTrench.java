package frc.robot.commandgroups.autonomous;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.commandgroups.ShootAndAdjust;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.autonomous.FollowPath;
import frc.robot.subsystems.drivetrain.commands.MoveToPosition;
import frc.robot.subsystems.drivetrain.commands.RotateToAngle;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.commands.AdjustHood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.ToggleVisionPiston;
import frc.robot.utils.VisionModule;
import org.photonvision.LEDMode;

import java.io.IOException;
import java.nio.file.Path;


public class ShootAndSafeTrench extends SequentialCommandGroup {

    public ShootAndSafeTrench(SwerveDrive swerveDrive, VisionModule vision, Shooter shooter, Hood hood) {
        Trajectory trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(Constants.Autonomous.SHOOT_TO_SAFE_TRENCH_PATH);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            System.out.println("Couldn't find the autonomous file...");
            ex.printStackTrace();
        }
        addCommands(
                new RotateToAngle(swerveDrive, 90),
                new ParallelCommandGroup(
                        new InstantCommand(() -> vision.setLEDs(LEDMode.kOff)),
                        new ToggleVisionPiston(vision)
                ),
                new WaitCommand(1),
                new ParallelDeadlineGroup(new WaitCommand(2), new MoveToPosition(swerveDrive, vision), new AdjustHood(hood, () -> vision.getTargetRawDistance().orElse(0))
                ),
                new ShootAndAdjust(shooter, vision, hood, false).withTimeout(4),
                new FollowPath(swerveDrive, trajectory));
    }
}
