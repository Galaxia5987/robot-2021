package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.shooter.Shooter;

public class ToggleVisionPiston extends InstantCommand {
    private final Shooter shooter;

    public ToggleVisionPiston(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        super.initialize();
        shooter.togglePiston();
    }
}
