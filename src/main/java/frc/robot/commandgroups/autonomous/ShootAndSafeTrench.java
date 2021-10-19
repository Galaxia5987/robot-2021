package frc.robot.commandgroups.autonomous;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.commandgroups.ConveyorShooter;
import frc.robot.commandgroups.FunnelAndShoot;
import frc.robot.commandgroups.PickupBalls;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.autonomous.FollowPath;
import frc.robot.subsystems.drivetrain.commands.MoveToPosition;
import frc.robot.subsystems.drivetrain.commands.RotateToAngle;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.commands.AdjustHood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.ToggleVisionPiston;
import frc.robot.utils.VisionModule;
import org.photonvision.LEDMode;

import java.io.IOException;
import java.nio.file.Path;


public class ShootAndSafeTrench extends SequentialCommandGroup {

    public ShootAndSafeTrench(SwerveDrive swerveDrive, VisionModule vision, Shooter shooter, Hood hood, Intake intake, Funnel funnel, Conveyor conveyor) {
        Trajectory trajectory = new Trajectory();
        Trajectory pickupToInitiation = new Trajectory();

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(Constants.Autonomous.SHOOT_TO_SAFE_TRENCH_PATH);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            Path trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(Constants.Autonomous.PICKUP_TO_INITIATION_PATH);
            pickupToInitiation = TrajectoryUtil.fromPathweaverJson(trajectoryPath2);
        } catch (IOException ex) {
            System.out.println("Couldn't find the autonomous file...");
            ex.printStackTrace();
        }
        addCommands(
                new RotateToAngle(swerveDrive, 80),
                new ParallelCommandGroup(
                        new InstantCommand(() -> vision.setLEDs(LEDMode.kOff)),
                        new ToggleVisionPiston(vision)
                ),
                new WaitCommand(1),
                new ParallelDeadlineGroup(new WaitCommand(2),
                        new MoveToPosition(swerveDrive, vision),
                        new AdjustHood(hood, vision, () -> vision.getTargetRawDistance().orElse(0))
                ),
//                        new WaitUntilCommand(() -> shooter.hasReachedSetpoint(hood.estimateVelocityFromDistance(vision.getTargetRawDistance().orElse(0)))),
//                new FunnelAndShoot(hood, shooter, funnel, conveyor, vision, Constants.Conveyor.CONVEYOR_MOTOR_POWER, false).withTimeout(4),
                new ConveyorShooter(shooter, hood, conveyor, funnel, vision, Constants.Conveyor.CONVEYOR_MOTOR_POWER, false).withTimeout(4),
                new FollowPath(swerveDrive, trajectory),
//                new WaitCommand(.5),
                new PickupBallsWhileDriving(intake, funnel, conveyor, swerveDrive).withTimeout(4),
                new ParallelCommandGroup(new FollowPath(swerveDrive, pickupToInitiation), new PickupBalls(intake, funnel, conveyor, () -> Constants.Intake.POWER, true))
        );

    }
}
