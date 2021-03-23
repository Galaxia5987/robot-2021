package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Shooter;

public class AdjustHood extends CommandBase {

    private final Shooter shooter;
    private final Shooter.State state;

    public AdjustHood(Shooter shooter, double distance) {
        this(shooter, Shooter.State.getOptimalState(distance));
    }

    public AdjustHood(Shooter shooter, Shooter.State state) {
        this.shooter = shooter;
        this.state = state;
    }

    @Override
    public void execute() {
        shooter.changeState(state);
    }
}
