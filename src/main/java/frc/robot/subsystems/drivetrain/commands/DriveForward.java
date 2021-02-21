package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.valuetuner.WebConstant;
import org.techfire225.webapp.FireLog;

public class DriveForward extends CommandBase {

    private SwerveDrive swerveDrive;
    private WebConstant target = new WebConstant("targetSpeed", 0);

    public DriveForward(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        double forward = Utils.joystickDeadband(-OI.getJoystickY(), Constants.SwerveDrive.JOYSTICK_THRESHOLD);

        for (int i = 0; i< 4; i++) {
            swerveDrive.getModule(i).setAngle(swerveDrive.getModule(i).getAngle());
            swerveDrive.getModule(i).setSpeed(target.get());
            FireLog.log("speed " + i, Math.abs(swerveDrive.getModule(i).getSpeed()));
        }

        FireLog.log("target speed", target.get());
        FireLog.log("swerve velocity", swerveDrive.getVelocity()[0]);
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