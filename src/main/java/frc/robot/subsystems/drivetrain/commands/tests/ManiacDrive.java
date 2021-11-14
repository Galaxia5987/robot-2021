package frc.robot.subsystems.drivetrain.commands.tests;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class ManiacDrive extends SequentialCommandGroup {
    private SequentialCommandGroup INSTANCE;
    private SwerveDrive swerveDrive;

    public ManiacDrive(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        addCommands(
                new DriveVelocity(0.5, swerveDrive), new TurnAround(swerveDrive),
                new DriveVelocity(0.5, swerveDrive), new TurnAround(swerveDrive),
                new DriveVelocity(0.5, swerveDrive), new TurnAround(swerveDrive),
                new DriveVelocity(0.5, swerveDrive), new TurnAround(swerveDrive),
                new DriveVelocity(0.5, swerveDrive), new TurnAround(swerveDrive),
                new DriveVelocity(0.5, swerveDrive), new TurnAround(swerveDrive),
                new DriveVelocity(0.5, swerveDrive), new TurnAround(swerveDrive),
                new DriveVelocity(0.5, swerveDrive), new TurnAround(swerveDrive),
                new DriveVelocity(0.5, swerveDrive), new TurnAround(swerveDrive),
                new DriveVelocity(0.5, swerveDrive), new TurnAround(swerveDrive)
        );
    }
}
