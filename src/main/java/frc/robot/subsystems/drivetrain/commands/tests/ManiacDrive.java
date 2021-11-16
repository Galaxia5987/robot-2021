package frc.robot.subsystems.drivetrain.commands.tests;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.Rotate;

public class ManiacDrive extends SequentialCommandGroup {

    public ManiacDrive(SwerveDrive swerveDrive, double velocity) {
        addCommands(
                new Rotate(swerveDrive, () -> 0).withTimeout(1),
                new DriveVelocity(velocity, swerveDrive), new TurnAround(swerveDrive),
                new DriveVelocity(velocity, swerveDrive), new TurnAround(swerveDrive),
                new DriveVelocity(velocity, swerveDrive), new TurnAround(swerveDrive),
                new DriveVelocity(velocity, swerveDrive), new TurnAround(swerveDrive),
                new DriveVelocity(velocity, swerveDrive), new TurnAround(swerveDrive),
                new DriveVelocity(velocity, swerveDrive), new TurnAround(swerveDrive),
                new DriveVelocity(velocity, swerveDrive), new TurnAround(swerveDrive),
                new DriveVelocity(velocity, swerveDrive), new TurnAround(swerveDrive),
                new DriveVelocity(velocity, swerveDrive), new TurnAround(swerveDrive),
                new DriveVelocity(velocity, swerveDrive), new TurnAround(swerveDrive),
                new Rotate(swerveDrive, () -> 0).withTimeout(1)
        );
    }
}
