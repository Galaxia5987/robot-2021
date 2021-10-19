package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.utils.VisionModule;
import org.techfire225.webapp.FireLog;

public class MoveToPosition extends CommandBase {

    private final SwerveDrive swerveDrive;
    private final VisionModule vision;
    private final PIDController anglePID = new PIDController(Constants.SwerveDrive.KP_TURN,
            Constants.SwerveDrive.KI_TURN, Constants.SwerveDrive.KD_TURN);
    private double startAngle;
    private double angleTarget;
    private double horizontalTarget;

    public MoveToPosition(SwerveDrive swerveDrive, VisionModule vision) {
        this.swerveDrive = swerveDrive;
        this.vision = vision;

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
//        drivePID.setPID(0, 0, 0);
        var visionAngle = vision.getVisionYaw();
        if (visionAngle.isPresent()) {
            angleTarget = -swerveDrive.getPose().getRotation().getDegrees() + visionAngle.getAsDouble();
            anglePID.setSetpoint(0);
            anglePID.setTolerance(1);
            startAngle = -swerveDrive.getPose().getRotation().getDegrees();
        }
/*            var distance = vision.getTargetRawDistance();
            if (distance.isPresent()) {
                horizontalTarget = distance.getAsDouble() * Math.tan(Math.abs(Math.toRadians(Robot.navx.getYaw() - startAngle)));
            }*/
//        }
    }

    @Override
    public void execute() {
        /*
        var distance = vision.getTargetRawDistance();
        if (distance.isPresent()) {
        double forward = drivePID.calculate(distance.getAsDouble(), Constants.SwerveDrive.DRIVE_SETPOINT);
        double strafe = drivePID.calculate(distance.getAsDouble() *
                Math.tan(Math.abs(Math.toRadians(Robot.navx.getYaw() - startAngle))), horizontalTarget);
*/
        double rotation = anglePID.calculate(Math.IEEEremainder(-swerveDrive.getPose().getRotation().getDegrees() - angleTarget, 360));
        swerveDrive.holonomicDrive(0, 0, rotation);
        FireLog.log("swerve-set-point", angleTarget);
        FireLog.log("swerve-rotation", Math.IEEEremainder(-swerveDrive.getPose().getRotation().getDegrees(), 360));
        FireLog.log("swerve-error", Math.IEEEremainder(-swerveDrive.getPose().getRotation().getDegrees() - angleTarget, 360));


    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
/*        var distance = vision.getTargetRawDistance();
        return distance.isPresent() && (Math.abs(angleTarget - Robot.navx.getYaw()) < Constants.SwerveDrive.TURN_TOLERANCE) &&
                (Math.abs(Constants.SwerveDrive.DRIVE_SETPOINT - distance.getAsDouble()) < Constants.SwerveDrive.DRIVE_TOLERANCE) &&
                (Math.abs(horizontalTarget - distance.getAsDouble() * Math.tan(Math.toRadians(Robot.navx.getYaw() - startAngle))) < Constants.SwerveDrive.DRIVE_TOLERANCE);
        */
        return anglePID.atSetpoint();
    }
}
