package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.guis.Kramer;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class Playground extends CommandBase {
    private final SwerveDrive swerve;
    private final Kramer kramer;
    private final HolonomicDriveController controller;

    public Playground(SwerveDrive swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
        kramer = new Kramer();
        controller = new HolonomicDriveController(
                new PIDController(Constants.Autonomous.kPXController, 0, 0),
                new PIDController(Constants.Autonomous.kPYController, 0, 0),
                new ProfiledPIDController(Constants.Autonomous.kPThetaController, 0, 0, Constants.Autonomous.kThetaControllerConstraints) {{
                    enableContinuousInput(-Math.PI, Math.PI);
                }});
    }

    @Override
    public void initialize() {
        super.initialize();
        swerve.resetOdometry();
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.getOdometryPose();
        kramer.updateRobotX(currentPose.getX());
        kramer.updateRobotY(currentPose.getY());
        kramer.updateRobotRotation(currentPose.getRotation().getDegrees());
        double desiredX = kramer.getDesiredX();
        double desiredY = kramer.getDesiredY();
        double desiredRotation = kramer.getDesiredRotation();
        Trajectory.State desiredState = new Trajectory.State(0, 0, 0, new Pose2d(desiredX, desiredY, Rotation2d.fromDegrees(desiredRotation)), 0);
        ChassisSpeeds targetChassisSpeeds = controller.calculate(currentPose, desiredState, Rotation2d.fromDegrees(desiredRotation));
        swerve.setChassisSpeeds(targetChassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
