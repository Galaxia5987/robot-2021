package frc.robot.utils.vision_commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.utils.VisionModule;
import org.photonvision.LEDMode;

public class TurnOffLeds extends ParallelCommandGroup {

    public TurnOffLeds() {
        addCommands(
                new InstantCommand(() -> VisionModule.camera.setLED(LEDMode.kBlink)),
                new InstantCommand(() -> VisionModule.camera.setLED(LEDMode.kOn))
        );
    }
}
