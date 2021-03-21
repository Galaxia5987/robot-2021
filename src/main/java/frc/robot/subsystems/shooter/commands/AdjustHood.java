package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.shooter.Shooter;

public class AdjustHood extends InstantCommand {

    private final Shooter shooter;
    private final double distance;

    public AdjustHood(Shooter shooter, double distance) {
        this.shooter = shooter;
        this.distance = distance;
    }


    @Override
    public void initialize() {
        shooter.changeState(Shooter.State.getOptimalState(distance));
    }
}
