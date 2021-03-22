package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ToggleLed extends InstantCommand {
    private static boolean on = false;

    @Override
    public void initialize() {
        super.initialize();
        ToggleLed.on = !ToggleLed.on;
        VisionModule.setLEDs(ToggleLed.on);
    }
}
