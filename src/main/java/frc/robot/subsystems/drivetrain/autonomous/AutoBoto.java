package frc.robot.subsystems.drivetrain.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.imuglyandimproud.PathPlanner;
import frc.robot.imuglyandimproud.PathPlannerTrajectory;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class AutoBoto extends SequentialCommandGroup {
    public AutoBoto(SwerveDrive swerveDrive) {
//        PathPlannerTrajectory examplePath = PathPlanner.loadPath("New New New Path", 1, 0.5); // saar efes :| HAHAHAHAHAHAHAH
//        swerveDrive.resetOdometry(examplePath.getInitialPose());
//        Robot.navx.reset();
        addCommands(new DriveFollowPath("New New New Path", swerveDrive, 0.5, 0.25));
//        addCommands(new FollowPath2(swerveDrive, examplePath));
    }
}
