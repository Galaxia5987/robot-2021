package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class RotateToAngle extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final double initAngle;
    private final double targetAngle;


    public RotateToAngle(SwerveDrive swerveDrive, double initAngle, double targetAngle) {
        this.swerveDrive = swerveDrive;
        this.initAngle = initAngle;
        this.targetAngle = targetAngle;
    }

    public double getOptimalPath() {
        double firstPath = targetAngle - initAngle;
        double secondPath = 360 - Math.abs(firstPath);
        double clockWisePath, antiClockWisePath;
        if (firstPath > 0) {
            clockWisePath = firstPath;
            antiClockWisePath = secondPath;
        } else {
            clockWisePath = secondPath;
            antiClockWisePath = Math.abs(firstPath);
        }
        if (clockWisePath < antiClockWisePath) {
            return clockWisePath;
        } else {
            return -antiClockWisePath;
        }
    }
}
