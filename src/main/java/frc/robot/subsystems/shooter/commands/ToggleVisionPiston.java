package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.utils.VisionModule;

public class ToggleVisionPiston extends InstantCommand {
    private final VisionModule vision;

    public ToggleVisionPiston(VisionModule vision) {
        this.vision = vision;
        addRequirements(this.vision);
    }

    @Override
    public void initialize() {
        super.initialize();
        vision.togglePiston();
    }
}
