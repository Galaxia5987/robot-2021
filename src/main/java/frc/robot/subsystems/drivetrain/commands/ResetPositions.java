package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class ResetPositions extends InstantCommand {

    private final SwerveDrive swerveDrive;

    public ResetPositions(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        swerveDrive.resetAllEncoders();
    }

}
