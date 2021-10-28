package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.DriveUtils;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class FineTunedDrive extends CommandBase {
    private final SwerveDrive swerveDrive;
    private double referenceAngle;
    private boolean isAngleSet = false;
    public static final PIDController driveController = new PIDController(1, 0, 0);
    public static final PIDController rotationController = new PIDController(1, 0, 0);


    public FineTunedDrive(SwerveDrive swerveDrive) {
        rotationController.enableContinuousInput(-180, 180);
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        double forward = DriveUtils.getForward();
        double strafe = DriveUtils.getStrafe();
        double rotation = DriveUtils.getRotation();

        if (DriveUtils.isPaused(forward, strafe, rotation)) {
            swerveDrive.stop();
            isAngleSet = false;
        }

        if (DriveUtils.isProblematic()) {
            swerveDrive.holonomicDrive(forward, strafe, rotation);
            isAngleSet = false;
        }

        if (DriveUtils.isStraightLine(forward, strafe, rotation)) {
            if (!isAngleSet) referenceAngle = Robot.navx.getYaw();
            swerveDrive.holonomicDrive(
                    DriveUtils.getForwardStraightLine(forward, swerveDrive.getRealChassisSpeeds().vxMetersPerSecond),
                    DriveUtils.getStrafeStraightLine(strafe, swerveDrive.getRealChassisSpeeds().vyMetersPerSecond),
                    DriveUtils.getRotationStraightLine(referenceAngle, Robot.navx.getYaw()));
            isAngleSet = true;
        }
        if (DriveUtils.isRotationOnly(forward, strafe, rotation)) {
            swerveDrive.holonomicDrive(
                    DriveUtils.getForwardRotationOnly(swerveDrive.getRealChassisSpeeds().vxMetersPerSecond),
                    DriveUtils.getStrafeRotationOnly(swerveDrive.getRealChassisSpeeds().vyMetersPerSecond),
                    DriveUtils.getRotationRotationOnly(rotation));
            isAngleSet = false;
        }
        if (DriveUtils.isFlex(forward, strafe, rotation)) {
            swerveDrive.holonomicDrive(
                    DriveUtils.getForwardStraightLine(forward, swerveDrive.getRealChassisSpeeds().vxMetersPerSecond),
                    DriveUtils.getStrafeStraightLine(strafe, swerveDrive.getRealChassisSpeeds().vyMetersPerSecond),
                    DriveUtils.getRotationStraightLine(referenceAngle, Robot.navx.getYaw()));
            isAngleSet = false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.stop();
    }
}
