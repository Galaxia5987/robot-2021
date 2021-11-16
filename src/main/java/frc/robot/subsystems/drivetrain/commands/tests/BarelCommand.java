package frc.robot.subsystems.drivetrain.commands.tests;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BarelCommand extends CommandBase {
    private final double seconds;
    private final Timer timer = new Timer();

    public BarelCommand(double seconds) {
        this.seconds = seconds;
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(seconds);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }
}
