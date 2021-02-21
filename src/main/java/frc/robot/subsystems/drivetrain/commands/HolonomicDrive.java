package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.*;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import org.techfire225.webapp.FireLog;

public class HolonomicDrive extends CommandBase {

    private final SwerveDrive swerveDrive;

    public HolonomicDrive(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        Robot.gyro.reset();
    }

    @Override
    public void execute() {
        GenericHID.Hand right = GenericHID.Hand.kRight;
        GenericHID.Hand left = GenericHID.Hand.kLeft;

        double forward = Utils.joystickDeadband(-OI.xbox.getY(right), Constants.SwerveDrive.JOYSTICK_THRESHOLD);
        double strafe = Utils.joystickDeadband(-OI.xbox.getX(right), Constants.SwerveDrive.JOYSTICK_THRESHOLD);
        double rotation = Utils.joystickDeadband(-OI.xbox.getX(left), Constants.SwerveDrive.JOYSTICK_THRESHOLD);
        if (forward != 0 || strafe != 0 || rotation != 0) {
            swerveDrive.holonomicDrive(forward, strafe, rotation);
        } else {
            for (int i = 0; i < 4; i++) {
                swerveDrive.swerveModules[i].setSpeed(0);
                swerveDrive.swerveModules[i].setAngle(swerveDrive.swerveModules[i].getAngle());
            }
        }

        FireLog.log("swerve angle by vectors", swerveDrive.getVelocity()[1]);
        FireLog.log("swerve direction", Robot.gyro.getAngle());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.stop();
    }


}
