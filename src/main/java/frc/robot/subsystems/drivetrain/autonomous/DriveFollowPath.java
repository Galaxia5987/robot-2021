// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.imuglyandimproud.PathPlanner;
import frc.robot.imuglyandimproud.PathPlannerTrajectory;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class DriveFollowPath extends CommandBase {
    Timer timer;
    SwervePath path;
    SwervePathController pathController;
    double lastTime;
    boolean ignoreHeading;
    private final SwerveDrive swerveDrive;

    public DriveFollowPath(String pathname, SwerveDrive swerveDrive, double maxVel, double acceleration) {
        addRequirements(swerveDrive);
        this.timer = new Timer();
        this.path = SwervePath.loadPath(pathname, maxVel, acceleration);
        this.swerveDrive = swerveDrive;

        PIDController posController = new PIDController(1, 0, 0);
        PIDController headingController = new PIDController(1, 0, 0);
        ProfiledPIDController rotationController = new ProfiledPIDController(1, 0, 0,
                new TrapezoidProfile.Constraints(0.1, 0.05));
        this.pathController = new SwervePathController(posController, headingController, rotationController);
        this.ignoreHeading = true;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        SwervePath.State initialState = path.getInitialState();
        swerveDrive.resetOdometry(new Pose2d(swerveDrive.getPoseMeters().getTranslation(), initialState.getRotation()));
        pathController.reset(swerveDrive.getPoseMeters());
        lastTime = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double time = timer.get();
        SwervePath.State desiredState = path.sample(time);
        Pose2d currentPose2d = new Pose2d();

        if (ignoreHeading) desiredState.rotation = new Rotation2d(0);

//        ChassisSpeeds targetSpeeds = pathController.calculate(swerveDrive.getPoseMeters(), desiredState, time - lastTime, !ignoreHeading);
        System.out.println(swerveDrive.getPoseMeters());
        ChassisSpeeds targetSpeeds = pathController.calculate(currentPose2d, desiredState, time - lastTime, !ignoreHeading);
//        System.out.println(targetSpeeds);
        swerveDrive.drive(targetSpeeds);

        lastTime = time;

        // Position Graph
        SmartDashboard.putNumber("PIDTarget", desiredState.getPos());
        SmartDashboard.putNumber("PIDActual", pathController.getTotalDistance());

        // Heading Graph
//        SmartDashboard.putNumber("PIDTarget", desiredState.getHeading().getDegrees());
//        SmartDashboard.putNumber("PIDActual", pathController.getCurrentHeading().getDegrees());

        // Rotation Graph
//        SmartDashboard.putNumber("PIDTarget", desiredState.getRotation().getDegrees());
//        SmartDashboard.putNumber("PIDActual", RobotContainer.drive.getPoseMeters().getRotation().getDegrees());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println(timer.get());
        timer.stop();
        swerveDrive.terminate();
//        swerveDrive.drive(0, 0, 0, true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(path.getRuntime());
    }
}