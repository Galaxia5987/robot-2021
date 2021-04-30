package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Utils;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.valuetuner.WebConstant;
import org.techfire225.webapp.FireLog;

public class DriveForward extends CommandBase {

    private final SwerveDrive swerveDrive;
    private WebConstant target = new WebConstant("targetSpeed", 0);

    public DriveForward(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        double forward = Utils.joystickDeadband(-RobotContainer.Xbox.getY(), Constants.SwerveDrive.JOYSTICK_THRESHOLD);

//        swerveDrive.stayAtAngle();
        for (int i = 0; i < 4; i++) {
            swerveDrive.getModule(i).setAngle(0);
            swerveDrive.getModule(i).setSpeed(target.get());
            FireLog.log("speed " + i, Math.abs(swerveDrive.getModule(i).getSpeed()));
            swerveDrive.getModule(i).configPIDF();
        }
//        swerveDrive.holonomicDrive(0, 0, forward);

        FireLog.log("target speed", target.get());
        FireLog.log("swerve velocity", swerveDrive.getVelocity()[0]);
        FireLog.log("swerve angle by vectors", swerveDrive.getVelocity()[1]);
        FireLog.log("swerve direction", Robot.navx.getYaw());
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