package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;


/**
 * This command convey the balls to the shooter.
 */
public class FeedShooter extends CommandBase {
    private final Conveyor conveyor;
    private final double setpoint;
    private final Timer timer = new Timer();
    private double currentPower = 0;
    private double lastTime = 0;

    public FeedShooter(Conveyor conveyor, double power) {
        this.conveyor = conveyor;
        this.setpoint = power;

        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        currentPower = updatePower(currentPower, timer.get() - lastTime, 2);
        conveyor.setPower(currentPower);
        lastTime = timer.get();
    }

    private double updatePower(double current, double dt, double time) {
        if (timer.hasElapsed(time) || current > setpoint) return setpoint;
        double m = setpoint / time;

        return Math.min(setpoint, m * dt + current);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
        timer.stop();
    }
}
