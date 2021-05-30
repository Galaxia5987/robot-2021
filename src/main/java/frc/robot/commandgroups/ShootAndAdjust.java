package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.commands.AdjustHood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;

public class ShootAndAdjust extends ParallelCommandGroup {

    public ShootAndAdjust(Shooter shooter, Hood hood) {
        addCommands(
                new Shoot(shooter),
                new AdjustHood(hood, Hood.State.MIDDLE)
        );
    }

}
