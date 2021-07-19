package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.commands.TrajectoryAdjustHood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.TrajectoryShoot;
import frc.robot.utils.VisionModule;

public class TrajectoryShootAndAdjust extends ParallelCommandGroup {

    public TrajectoryShootAndAdjust(Shooter shooter, VisionModule vision, Hood hood) {
        addCommands(
                new TrajectoryShoot(shooter, vision, hood),
                new TrajectoryAdjustHood(hood, shooter, () -> vision.getTargetRawDistance().orElse(0))
        );
    }

}
