package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class TankDrive extends CommandBase {
    private SwerveDrive swerveDrive;

    public TankDrive(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        for (int i = 0; i < 4; i++) {
            swerveDrive.getModule(i).setAngle(0);
        }
    }

    @Override
    public void execute() {
        double rightForward = RobotContainer.Xbox.getY(GenericHID.Hand.kRight);
        double leftForward = RobotContainer.Xbox.getY(GenericHID.Hand.kLeft);

        swerveDrive.getModule(0).setSpeed(rightForward);
        swerveDrive.getModule(2).setSpeed(rightForward);

        swerveDrive.getModule(1).setSpeed(leftForward);
        swerveDrive.getModule(3).setSpeed(leftForward);
    }
}