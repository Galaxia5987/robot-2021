package frc.robot.utils.vision_commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.utils.VisionModule;
import org.photonvision.LEDMode;

public class ChangeLEDs extends InstantCommand {

    private final boolean on;

    public ChangeLEDs(boolean on) {
        this.on = on;
    }

    @Override
    public void initialize() {
        VisionModule.setLEDs(on ? LEDMode.kOn : LEDMode.kOff);
    }
}
