package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.SwerveModule;

public class TankDrive extends CommandBase {
    private SwerveDrive swerveDrive;

    public TankDrive(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        for (SwerveModule swerveModule : swerveDrive.swerveModules) {
            swerveModule.setAngle(0);
        }
    }

    @Override
    public void execute() {
        double rightForward = OI.xbox.getY(GenericHID.Hand.kRight);
        double leftForward = OI.xbox.getY(GenericHID.Hand.kLeft);

        swerveDrive.getModule(0).setSpeed(rightForward);
        swerveDrive.getModule(2).setSpeed(rightForward);

        swerveDrive.getModule(1).setSpeed(leftForward);
        swerveDrive.getModule(3).setSpeed(leftForward);
    }
}
