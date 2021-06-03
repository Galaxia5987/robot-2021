package frc.robot.utils.vision_commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.utils.VisionModule;
import org.photonvision.LEDMode;

public class ChangeLEDs extends InstantCommand {

    private VisionModule vision;
    private final boolean on;

    public ChangeLEDs(VisionModule vision, boolean on) {
        this.vision = vision;
        this.on = on;
    }

    @Override
    public void initialize() {
        vision.setLEDs(on ? LEDMode.kOn : LEDMode.kOff);
    }
}
