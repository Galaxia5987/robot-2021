package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;


/**
 * This command convey the balls to the shooter.
 */
public class FeedShooter extends CommandBase {
    private final Conveyor conveyor;
    private final double power;
    private final Timer timer = new Timer();

    public FeedShooter(Conveyor conveyor, double power) {
        this.conveyor = conveyor;
        this.power = power;

        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        conveyor.setPower(power);
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
