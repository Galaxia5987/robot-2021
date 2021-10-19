package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import org.techfire225.webapp.FireLog;

public class RotateToAngle extends CommandBase {

    private final SwerveDrive swerveDrive;
    private final PIDController anglePID = new PIDController(Constants.SwerveDrive.KP_TURN,
            Constants.SwerveDrive.KI_TURN, Constants.SwerveDrive.KD_TURN);
    private double startAngle;
    private double angleTarget;
    private double angle;

    public RotateToAngle(SwerveDrive swerveDrive, double angle) {
        this.swerveDrive = swerveDrive;
        this.angle = angle;

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        angleTarget = -swerveDrive.getPose().getRotation().getDegrees() + angle;
        anglePID.setSetpoint(0);
        anglePID.setTolerance(1);
        startAngle = -swerveDrive.getPose().getRotation().getDegrees();
    }

    @Override
    public void execute() {
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
        return anglePID.atSetpoint();
    }
}
