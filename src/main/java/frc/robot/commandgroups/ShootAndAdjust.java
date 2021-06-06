package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.commands.AdjustHood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;
import frc.robot.utils.VisionModule;

public class ShootAndAdjust extends ParallelCommandGroup {

    public ShootAndAdjust(Shooter shooter, VisionModule vision, Hood hood, boolean manual) {
        System.out.println("Distance" + vision.getTargetRawDistance().orElse(0));
        addCommands(
                new Shoot(shooter, vision, hood, manual),
                new AdjustHood(hood, () -> vision.getTargetRawDistance().orElse(0))
        );
    }

}
