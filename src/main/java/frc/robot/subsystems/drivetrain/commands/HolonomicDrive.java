package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Utils;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import org.techfire225.webapp.FireLog;

public class HolonomicDrive extends CommandBase {

    private final SwerveDrive swerveDrive;
    private boolean bool;

    public HolonomicDrive(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        GenericHID.Hand right = GenericHID.Hand.kRight;
        GenericHID.Hand left = GenericHID.Hand.kLeft;
//        double forward = smoothInput(Utils.joystickDeadband(RobotContainer.Xbox.getY(left), Constants.SwerveDrive.JOYSTICK_THRESHOLD));
//        double strafe = smoothInput(-Utils.joystickDeadband(RobotContainer.Xbox.getX(left), Constants.SwerveDrive.JOYSTICK_THRESHOLD));
        double forward = smoothInput((RobotContainer.Xbox.getY(left)));
        double strafe = smoothInput(-(RobotContainer.Xbox.getX(left)));
        double rotation = smoothInput(-Utils.joystickDeadband(RobotContainer.Xbox.getX(right), Constants.SwerveDrive.JOYSTICK_THRESHOLD));
        if (Math.sqrt(Math.pow(RobotContainer.Xbox.getY(left), 2) + Math.pow(RobotContainer.Xbox.getX(left), 2)) < Constants.SwerveDrive.JOYSTICK_THRESHOLD) {
            forward = 0;
            strafe = 0;
        }


        // turns the joystick values into the heading of the robot
        forward *= Constants.SwerveDrive.SPEED_MULTIPLIER;
        strafe *= Constants.SwerveDrive.SPEED_MULTIPLIER;
        rotation *= Constants.SwerveDrive.ROTATION_MULTIPLIER;

        if (RobotContainer.Xbox.getRawButtonPressed(XboxController.Button.kStickLeft.value)) {
            bool = true;
        }
        if (RobotContainer.Xbox.getRawButtonReleased(XboxController.Button.kStickLeft.value)) {
            bool = false;
        }

        if (bool) {
            System.out.println("true");
//            forward *= 2 / 0.7;
            forward *= 8;
            strafe *= 8;
//            strafe *= 2 / 0.7;

        }

        if (forward != 0 || strafe != 0 || rotation != 0) {
            swerveDrive.holonomicDrive(forward, strafe, rotation);
        } else {
            swerveDrive.lockModulePositions();
        }

        FireLog.log("swerve angle by vectors", swerveDrive.getVelocity()[1]);
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
