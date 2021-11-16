package frc.robot.subsystems.drivetrain.commands.tests;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class SpinMotors extends CommandBase {
    private final SwerveDrive swerveDrive;
    private int count;
    private int currentCount;

    public SpinMotors(SwerveDrive swerveDrive, int count) {
        this.swerveDrive = swerveDrive;
        this.count = count;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        currentCount = 0;
        swerveDrive.setAngles(0);
    }

    @Override
    public void execute() {
        if(currentCount != count)
            swerveDrive.holonomicDrive(0, 0, 3);
        else{
            count--;
            swerveDrive.setAngles(0);
        }
        currentCount++;
    }

    @Override
    public boolean isFinished() {
        return currentCount > count;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.terminate();
    }
}
