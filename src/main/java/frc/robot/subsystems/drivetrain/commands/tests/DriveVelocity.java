package frc.robot.subsystems.drivetrain.commands.tests;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class DriveVelocity extends CommandBase {
    private double output;
    private SwerveDrive swerveDrive;
    private Timer timer = new Timer();

    public DriveVelocity(double output, SwerveDrive swerveDrive) {
        this.output = output;
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        swerveDrive.holonomicDrive(output, 0, 0);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.terminate();
    }

    @Override
    public boolean isFinished() {
        return (timer.get() > 1);
    }
}
