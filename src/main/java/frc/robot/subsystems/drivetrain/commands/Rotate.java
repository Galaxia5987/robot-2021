package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import org.techfire225.webapp.FireLog;

import java.util.function.DoubleSupplier;

public class Rotate extends CommandBase {

    private final SwerveDrive swerveDrive;
    private DoubleSupplier targetAngle;

    public Rotate(SwerveDrive swerveDrive, DoubleSupplier angle) {
        this.swerveDrive = swerveDrive;
        targetAngle = angle;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
//        double rotation = -RobotContainer.Xbox.getY();
//        rotation = Utils.joystickDeadband(rotation, Constants.SwerveDrive.JOYSTICK_THRESHOLD);

        swerveDrive.getModule(0).setAngle(Math.toRadians(targetAngle.getAsDouble()));
        swerveDrive.getModule(1).setAngle(Math.toRadians(targetAngle.getAsDouble()));
        swerveDrive.getModule(2).setAngle(Math.toRadians(targetAngle.getAsDouble()));
        swerveDrive.getModule(3).setAngle(Math.toRadians(targetAngle.getAsDouble()));

        SmartDashboard.putNumber("module FR", Math.toDegrees(swerveDrive.getModule(0).getAngle()));
        SmartDashboard.putNumber("module FL", Math.toDegrees(swerveDrive.getModule(1).getAngle()));
        SmartDashboard.putNumber("module RR", Math.toDegrees(swerveDrive.getModule(2).getAngle()));
        SmartDashboard.putNumber("module RL", Math.toDegrees(swerveDrive.getModule(3).getAngle()));
        FireLog.log("angle-setpoint", targetAngle.getAsDouble());
        FireLog.log("module FR", Math.toDegrees(swerveDrive.getModule(0).getAngle()));
        FireLog.log("module FL", Math.toDegrees(swerveDrive.getModule(1).getAngle()));
        FireLog.log("module RR", Math.toDegrees(swerveDrive.getModule(2).getAngle()));
        FireLog.log("module RL", Math.toDegrees(swerveDrive.getModule(3).getAngle()));

        FireLog.log("angle ", swerveDrive.getModule(2).getAngle());
        FireLog.log("swerve direction", Robot.navx.getYaw());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.terminate();
    }

}
