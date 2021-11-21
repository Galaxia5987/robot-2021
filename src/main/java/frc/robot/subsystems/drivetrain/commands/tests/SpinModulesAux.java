package frc.robot.subsystems.drivetrain.commands.tests;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.Rotate;

public class SpinModulesAux extends CommandBase {
    private double velocity;
    private SwerveDrive swerveDrive;
    private Timer timer = new Timer();

    public SpinModulesAux(SwerveDrive swerveDrive, double velocity) {
        this.swerveDrive = swerveDrive;
        this.velocity = velocity;

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        for (int i = 0; i < 4; i++) {
            swerveDrive.getModule(i).setSpeed(velocity);
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.terminate();

        rotate();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(10);
    }

    public void rotate() {
        new Rotate(swerveDrive, () -> 0);
    }
}
