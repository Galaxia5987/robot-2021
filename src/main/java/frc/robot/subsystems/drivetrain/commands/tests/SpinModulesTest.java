package frc.robot.subsystems.drivetrain.commands.tests;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import static frc.robot.Constants.LOOP_PERIOD;

public class SpinModulesTest extends CommandBase {
    private SwerveDrive swerveDrive;
    private double d_Angle = Math.toRadians(90);
    private double reqAngle = 0;
    private double cycles = 0;
    private Timer timer = new Timer();

    public SpinModulesTest(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        if (timer.advanceIfElapsed(10)) {
            reqAngle = reqAngle == 90 ? 0 : 90;
        }
        swerveDrive.setAngles(reqAngle);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.terminate();
    }

    @Override
    public boolean isFinished() {
        return (cycles == 20);
    }
}
