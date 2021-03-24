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
        conveyor.setPower(moderatePower(timer.get(), 2));
    }

    private double moderatePower(double elapsedTime, double cycleTime) {
        if (elapsedTime > cycleTime) return setpoint;
        double m = setpoint / cycleTime;
        return Math.min(setpoint, m * elapsedTime);
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
