package frc.robot.utils;

import static frc.robot.subsystems.drivetrain.commands.FineTunedDrive.driveController;
import static frc.robot.subsystems.drivetrain.commands.FineTunedDrive.rotationController;

public class DriveUtils {
    public static double PROBLEMATIC_LOW_SPEEDS_DEADBAND = 0.25;

    public static boolean isProblematic(double vector, double rotation) {
        return Math.abs(vector) <= PROBLEMATIC_LOW_SPEEDS_DEADBAND && rotation == 0;
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
