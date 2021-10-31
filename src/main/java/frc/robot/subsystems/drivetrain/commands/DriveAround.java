package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class DriveAround extends CommandBase {
    private final SwerveDrive swerve;
    public DriveAround(SwerveDrive swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        swerve.holonomicDrive(0,0,2);
        System.out.println(swerve.getModule(0).getSpeed() + " " + swerve.getModule(1).getSpeed()  + " " + swerve.getModule(2).getSpeed() + " " + swerve.getModule(3).getSpeed() );
//        System.out.println(Math.toDegrees(swerve.getModule(0).getAngle()) + " " + Math.toDegrees(swerve.getModule(1).getAngle())  + " " + Math.toDegrees(swerve.getModule(2).getAngle()) + " " + Math.toDegrees(swerve.getModule(3).getAngle() ));

    }

    @Override
    public void end(boolean interrupted) {
        swerve.terminate();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
