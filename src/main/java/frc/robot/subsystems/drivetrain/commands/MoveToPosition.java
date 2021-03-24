package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.utils.VisionModule;

public class MoveToPosition extends CommandBase {

    private SwerveDrive swerveDrive;
    private PIDController drivePID = new PIDController(Constants.SwerveDrive.KP_MOVE.get(),
            Constants.SwerveDrive.KI_MOVE.get(), Constants.SwerveDrive.KD_MOVE.get());
    private PIDController anglePID = new PIDController(Constants.SwerveDrive.KP_TURN.get(),
            Constants.SwerveDrive.KI_TURN.get(), Constants.SwerveDrive.KD_TURN.get());
    private double startAngle;
    private double angleTarget;
    private double horizontalTarget;

    public MoveToPosition(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    @Override
    public void initialize() {
        drivePID.setPID(0, 0 , 0);
        anglePID.setPID(Constants.SwerveDrive.KP_TURN.get(),
                Constants.SwerveDrive.KI_TURN.get(), Constants.SwerveDrive.KD_TURN.get());
        angleTarget = Robot.navx.getYaw() + VisionModule.getVisionAngle();
        startAngle = Robot.navx.getYaw();
        horizontalTarget = VisionModule.getTargetRawDistance(55) * Math.tan(Math.abs(Math.toRadians(Robot.navx.getYaw() - startAngle)));
    }

    @Override
    public void execute() {
        double forward = drivePID.calculate(VisionModule.getTargetRawDistance(55), Constants.SwerveDrive.DRIVE_SETPOINT);
        double strafe = drivePID.calculate(VisionModule.getTargetRawDistance(55) *
                Math.tan(Math.abs(Math.toRadians(Robot.navx.getYaw() - startAngle))), horizontalTarget);
        double rotation = anglePID.calculate(Robot.navx.getYaw(), angleTarget);
        swerveDrive.holonomicDrive(forward, strafe, rotation);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.lock();
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(angleTarget - Robot.navx.getYaw()) < Constants.SwerveDrive.TURN_TOLERANCE) &&
                (Math.abs(Constants.SwerveDrive.DRIVE_SETPOINT - VisionModule.getTargetRawDistance(55)) < Constants.SwerveDrive.DRIVE_TOLERANCE) &&
                (Math.abs(horizontalTarget - VisionModule.getTargetRawDistance(55) * Math.tan(Math.toRadians(Robot.navx.getYaw() - startAngle))) < Constants.SwerveDrive.DRIVE_TOLERANCE);
    }
}
