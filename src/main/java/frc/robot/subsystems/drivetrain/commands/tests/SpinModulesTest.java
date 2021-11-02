package frc.robot.subsystems.drivetrain.commands.tests;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import static frc.robot.Constants.LOOP_PERIOD;

public class SpinModulesTest extends CommandBase {
    private SwerveDrive swerveDrive;
    private double d_Angle = Math.toRadians(90);
    private double reqAngle = 90;
    private double cycles = 0;
    private Timer timer = new Timer();

    public SpinModulesTest(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(cycles + 1))
            swerveDrive.setAngles(reqAngle);

        if(swerveDrive.hasReachedAngles(reqAngle)){
            cycles += 1;
            reqAngle += d_Angle;
        }
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
