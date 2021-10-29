package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.DriveUtils;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class FineTunedDrive extends CommandBase {
    private final SwerveDrive swerveDrive;
    private double referenceAngle;
    private boolean isAngleSet = false;
    public static final PIDController driveController = new PIDController(0.08, 0, 0);
    public static final PIDController rotationController = new PIDController(0.1, 0, 0);


    public FineTunedDrive(SwerveDrive swerveDrive) {
        rotationController.enableContinuousInput(-180, 180);
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        GenericHID.Hand right = GenericHID.Hand.kRight;
        GenericHID.Hand left = GenericHID.Hand.kLeft;
        double forward = -RobotContainer.Xbox.getY(left);
        double strafe = RobotContainer.Xbox.getX(left);
        double rotation = RobotContainer.Xbox.getX(right);
        double alpha = Math.atan2(forward, strafe);
        double vector = Math.hypot(forward, strafe);
        if (Math.abs(vector) < 0.1) vector = 0;
        if (Math.abs(rotation) < 0.1) rotation = 0;
        vector = vector < 0 ? -Math.pow(vector, 2) : Math.pow(vector, 2);
        double checking_vector = vector;
        vector *= Constants.SwerveDrive.SPEED_MULTIPLIER * 0.35;
        rotation *= Constants.SwerveDrive.ROTATION_MULTIPLIER * 0.75;
        forward = Math.sin(alpha) * vector;
        strafe = Math.cos(alpha) * vector;

        if (DriveUtils.isPaused(forward, strafe, rotation)) {
            swerveDrive.stop();
            isAngleSet = false;
            return;
        }

        if (DriveUtils.isProblematic(checking_vector, rotation)) {
            swerveDrive.holonomicDrive(forward, strafe, rotation);
            isAngleSet = false;
            return;
        }

        if (DriveUtils.isStraightLine(forward, strafe, rotation)) {
            if (!isAngleSet) {
                referenceAngle = Robot.navx.getYaw();
                isAngleSet = true;
            }
            swerveDrive.holonomicDrive(
                    DriveUtils.getForwardStraightLine(forward, swerveDrive.getRealChassisSpeeds().vxMetersPerSecond),
                    DriveUtils.getStrafeStraightLine(strafe, swerveDrive.getRealChassisSpeeds().vyMetersPerSecond),
                    DriveUtils.getRotationStraightLine(referenceAngle, Robot.navx.getYaw()));
            return;
        }
        if (DriveUtils.isRotationOnly(forward, strafe, rotation)) {
            swerveDrive.holonomicDrive(
                    DriveUtils.getForwardRotationOnly(swerveDrive.getRealChassisSpeeds().vxMetersPerSecond),
                    DriveUtils.getStrafeRotationOnly(swerveDrive.getRealChassisSpeeds().vyMetersPerSecond),
                    DriveUtils.getRotationRotationOnly(rotation));
            isAngleSet = false;
            return;
        }
        if (DriveUtils.isFlex(forward, strafe, rotation)) {
            swerveDrive.holonomicDrive(
                    DriveUtils.getForwardFlex(forward, swerveDrive.getRealChassisSpeeds().vxMetersPerSecond),
                    DriveUtils.getStrafeFlex(strafe, swerveDrive.getRealChassisSpeeds().vyMetersPerSecond),
                    DriveUtils.getRotationFlex(rotation));
            isAngleSet = false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.stop();
    }
}
