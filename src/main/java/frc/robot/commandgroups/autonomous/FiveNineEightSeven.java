package frc.robot.commandgroups.autonomous;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.autonomous.FollowPath;

import java.io.IOException;
import java.nio.file.Path;

public class FiveNineEightSeven extends SequentialCommandGroup {

    public FiveNineEightSeven(SwerveDrive swerveDrive) {
        Trajectory initiationToFive = new Trajectory();
        Trajectory five = new Trajectory();
        Trajectory fiveToNine = new Trajectory();
        Trajectory nine = new Trajectory();
        Trajectory nineToEight = new Trajectory();
        Trajectory eight = new Trajectory();
        Trajectory eightToSeven = new Trajectory();
        Trajectory seven = new Trajectory();
        try {

            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(Constants.Autonomous.INITIATION_TO_FIVE_PATH);
            initiationToFive = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            Path trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(Constants.Autonomous.FIVE_PATH);
            five = TrajectoryUtil.fromPathweaverJson(trajectoryPath2);
            Path trajectoryPath3 = Filesystem.getDeployDirectory().toPath().resolve(Constants.Autonomous.FIVE_TO_NINE_PATH);
            fiveToNine = TrajectoryUtil.fromPathweaverJson(trajectoryPath3);
            Path trajectoryPath4 = Filesystem.getDeployDirectory().toPath().resolve(Constants.Autonomous.NINE_PATH);
            nine = TrajectoryUtil.fromPathweaverJson(trajectoryPath4);
            Path trajectoryPath5 = Filesystem.getDeployDirectory().toPath().resolve(Constants.Autonomous.NINE_TO_EIGHT_PATH);
            nineToEight = TrajectoryUtil.fromPathweaverJson(trajectoryPath5);
            Path trajectoryPath6 = Filesystem.getDeployDirectory().toPath().resolve(Constants.Autonomous.EIGHT_PATH);
            eight = TrajectoryUtil.fromPathweaverJson(trajectoryPath6);
            Path trajectoryPath7 = Filesystem.getDeployDirectory().toPath().resolve(Constants.Autonomous.EIGHT_TO_SEVEN_PATH);
            eightToSeven = TrajectoryUtil.fromPathweaverJson(trajectoryPath7);
            Path trajectoryPath8 = Filesystem.getDeployDirectory().toPath().resolve(Constants.Autonomous.SEVEN_PATH);
            seven = TrajectoryUtil.fromPathweaverJson(trajectoryPath8);

        } catch (IOException ex) {
            System.out.println("Couldn't find the autonomous file...");
            ex.printStackTrace();
        }

        addCommands(
                new FollowPath(swerveDrive, initiationToFive),
                new WaitCommand(1),
                new FollowPath(swerveDrive, five),
                new WaitCommand(1),
                new FollowPath(swerveDrive, fiveToNine),
                new WaitCommand(1),
                new FollowPath(swerveDrive, nine),
                new WaitCommand(1),
                new FollowPath(swerveDrive, nineToEight),
                new WaitCommand(1),
                new FollowPath(swerveDrive, eight),
                new WaitCommand(1),
                new FollowPath(swerveDrive, eightToSeven),
                new WaitCommand(1),
                new FollowPath(swerveDrive, seven)
        );
    }

}
