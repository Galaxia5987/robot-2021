package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;

import static frc.robot.subsystems.drivetrain.commands.FineTunedDrive.driveController;
import static frc.robot.subsystems.drivetrain.commands.FineTunedDrive.rotationController;

public class DriveUtils {
    public static double JOYSTICK_DEADBAND = 0.1;
    public static double PROBLEMATIC_LOW_SPEEDS_DEADBAND = 0.25;
    public static double DRIVE_SPEED_MULTIPLIER = Constants.SwerveDrive.SPEED_MULTIPLIER;
    public static double ROTATION_SPEED_MULTIPLIER = Constants.SwerveDrive.ROTATION_MULTIPLIER;


    public static double deadband(double value) {
        return Math.abs(value) < JOYSTICK_DEADBAND ? value : 0;
    }

    public static double easeIn(double value) {
        return Math.pow(value, 3);
    }

    public static double easeOut(double value) {
        return Math.pow(value, 1.0 / 3.0);
    }

    public static boolean isProblematic() {
        double forward = -RobotContainer.Xbox.getY(GenericHID.Hand.kLeft);
        double strafe = RobotContainer.Xbox.getX(GenericHID.Hand.kLeft);
        double rotation = RobotContainer.Xbox.getX(GenericHID.Hand.kRight);
        forward = deadband(forward);
        strafe = deadband(strafe);
        rotation = deadband(rotation);
        double vector = Math.hypot(strafe, forward);
        vector = easeIn(vector);
        return vector <= PROBLEMATIC_LOW_SPEEDS_DEADBAND && rotation != 0;
    }

    public static double getForward() {
        double forward = -RobotContainer.Xbox.getY(GenericHID.Hand.kLeft);
        double strafe = RobotContainer.Xbox.getX(GenericHID.Hand.kLeft);
        forward = deadband(forward);
        strafe = deadband(strafe);
        double vector = Math.hypot(strafe, forward);
        double alpha = Math.atan2(forward, strafe);
        vector *= DRIVE_SPEED_MULTIPLIER;
        vector = easeIn(vector);
        forward = Math.sin(alpha) * vector;
        return forward;
    }

    public static double getStrafe() {
        double forward = -RobotContainer.Xbox.getY(GenericHID.Hand.kLeft);
        double strafe = RobotContainer.Xbox.getX(GenericHID.Hand.kLeft);
        forward = deadband(forward);
        strafe = deadband(strafe);
        double vector = Math.hypot(strafe, forward);
        double alpha = Math.atan2(forward, strafe);
        vector *= DRIVE_SPEED_MULTIPLIER;
        vector = easeIn(vector);
        strafe = Math.cos(alpha) * vector;
        return strafe;
    }

    public static double getRotation() {
        double rotation = RobotContainer.Xbox.getX(GenericHID.Hand.kRight);
        rotation = deadband(rotation);
        rotation = easeIn(rotation);
        rotation *= ROTATION_SPEED_MULTIPLIER;
        return rotation;
    }

    public static boolean isPaused(double forward, double strafe, double rotation) {
        return forward == 0 && strafe == 0 && rotation == 0;
    }

    public static boolean isStraightLine(double forward, double strafe, double rotation) {
        return rotation == 0 && (forward != 0 || strafe != 0);
    }

    public static boolean isRotationOnly(double forward, double strafe, double rotation) {
        return forward == 0 && strafe == 0 && rotation != 0;
    }

    public static boolean isFlex(double forward, double strafe, double rotation) {
        return rotation != 0 && (forward != 0 || strafe != 0);
    }


    public static double getForwardStraightLine(double desiredForward, double realForward) {
        return desiredForward + driveController.calculate(realForward, desiredForward);
    }

    public static double getStrafeStraightLine(double desiredStrafe, double realStrafe) {
        return desiredStrafe + driveController.calculate(realStrafe, desiredStrafe);
    }

    public static double getRotationStraightLine(double referenceAngle, double currentAngle) {
        return rotationController.calculate(currentAngle, referenceAngle);
    }

    public static double getForwardRotationOnly(double realForward) {
        return driveController.calculate(realForward, 0);
    }

    public static double getStrafeRotationOnly(double realStrafe) {
        return driveController.calculate(realStrafe, 0);
    }

    public static double getRotationRotationOnly(double desiredRotation) {
        return desiredRotation;
    }

    public static double getForwardFlex(double desiredForward, double realForward) {
        return desiredForward + driveController.calculate(realForward, desiredForward);
    }

    public static double getStrafeFlex(double desiredStrafe, double realStrafe) {
        return desiredStrafe + driveController.calculate(realStrafe, desiredStrafe);
    }

    public static double getRotationFlex(double desiredFlex) {
        return desiredFlex;
    }
}
