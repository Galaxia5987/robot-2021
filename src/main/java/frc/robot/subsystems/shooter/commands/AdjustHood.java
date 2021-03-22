package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.shooter.Shooter;

public class AdjustHood extends CommandBase {

    private final Shooter shooter;
    private final double distance;

    public AdjustHood(Shooter shooter, double distance) {
        this.shooter = shooter;
        this.distance = distance;
    }


    @Override
    public void execute() {
//        shooter.changeState(Shooter.State.getOptimalState(distance));
        shooter.hoodMotor.set(-0.2);
    }
}
