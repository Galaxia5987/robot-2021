package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Utils;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import org.techfire225.webapp.FireLog;

public class HolonomicDriveExperimental extends CommandBase {

    private final SwerveDrive swerveDrive;

    public HolonomicDriveExperimental(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }


    @Override
    public void execute() {
        GenericHID.Hand right = GenericHID.Hand.kRight;
        GenericHID.Hand left = GenericHID.Hand.kLeft;
//        double forward = smoothInput(Utils.joystickDeadband(RobotContainer.Xbox.getY(left), Constants.SwerveDrive.JOYSTICK_THRESHOLD));
//        double strafe = smoothInput(-Utils.joystickDeadband(RobotContainer.Xbox.getX(left), Constants.SwerveDrive.JOYSTICK_THRESHOLD));

        double forward = -RobotContainer.Xbox.getY(left);
        double strafe = RobotContainer.Xbox.getX(left);
        double alpha = Math.atan2(forward, strafe);
        double vector = Math.hypot(forward, strafe);
//        vector = smoothInput(vector);
        forward = Math.sin(alpha) * vector;
        strafe = Math.cos(alpha) * vector;

        double rotation = Utils.joystickDeadband(RobotContainer.Xbox.getX(right), Constants.SwerveDrive.JOYSTICK_THRESHOLD);
        if (vector < Constants.SwerveDrive.JOYSTICK_THRESHOLD) {
            forward = 0;
            strafe = 0;
        }

        // turns the joystick values into the heading of the robot
        forward *= Constants.SwerveDrive.SPEED_MULTIPLIER;
        strafe *= Constants.SwerveDrive.SPEED_MULTIPLIER;
        rotation *= Constants.SwerveDrive.ROTATION_MULTIPLIER;

        if (forward != 0 || strafe != 0 || rotation != 0) {
            if (rotation == 0) {
                double averageSpeed = 0;
                for (int i = 0; i < 4; i++) {
                    averageSpeed += Math.abs(swerveDrive.getModule(i).getSpeed());
                }
                averageSpeed /= 4;
                double[] filter = {0, 0, 0, 0};
                for (int i = 0; i < 4; i++) {
                    filter[i] = averageSpeed - Math.abs(swerveDrive.getModule(i).getSpeed());
                }
                swerveDrive.holonomicDriveExperimental(forward, strafe, rotation, filter);
            } else {
                swerveDrive.holonomicDrive(forward, strafe, rotation);
            }
        } else {
            swerveDrive.stop();
        }

        FireLog.log("swerve direction", Robot.navx.getYaw());
    }

    private double smoothInput(double input) {
        return Math.pow(input, 3) * 0.5;
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
