package frc.robot.commandgroups.autonomous;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commandgroups.ConveyorShooter;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.autonomous.FollowPath;
import frc.robot.subsystems.drivetrain.commands.MoveToPosition;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.VisionModule;
import org.photonvision.LEDMode;

import java.io.IOException;
import java.nio.file.Path;

public class ShootFromInitiation extends SequentialCommandGroup {
    public ShootFromInitiation(SwerveDrive swerveDrive, VisionModule vision, Funnel funnel, Conveyor conveyor, Shooter shooter, Hood hood) {
        Trajectory trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(Constants.Autonomous.INITIATION_TO_SHOOT_PATH);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            System.out.println("Couldn't find the autonomous file...");
            ex.printStackTrace();
        }
        addCommands(new FollowPath(swerveDrive, trajectory),
                new ParallelCommandGroup(
                        new InstantCommand(() -> vision.setLEDs(LEDMode.kOff)),
                        new InstantCommand(() -> vision.setPistonMode(true), vision)
                ),
                new WaitCommand(0.5),
                new MoveToPosition(swerveDrive, vision).withTimeout(2),
                new ConveyorShooter(shooter, hood, conveyor, funnel, vision, Constants.Conveyor.CONVEYOR_MOTOR_POWER, false)
        );
    }
}
