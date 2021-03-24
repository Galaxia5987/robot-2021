package frc.robot.utils.vision_commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.utils.VisionModule;
import org.photonvision.LEDMode;

/**
 * Once upon a time, Frank (2021's robot) had a little poo poo, and in order to toggle the LEDs we needed first to blink and then to turn the LEDs on.
 */
public class ToggleLEDs extends SequentialCommandGroup {

    public ToggleLEDs() {
        addCommands(
                new InstantCommand(() -> VisionModule.setLEDs(LEDMode.kBlink)),
                new InstantCommand(() -> VisionModule.setLEDs(LEDMode.kOn))
        );
    }
}
