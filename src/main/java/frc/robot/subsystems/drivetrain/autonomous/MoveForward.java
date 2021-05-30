package frc.robot.subsystems.drivetrain.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class MoveForward extends CommandBase {

    private SwerveDrive swerveDrive;

    public MoveForward(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    @Override
    public void initialize() {
        for (int i = 0; i < 4; i++) {
            swerveDrive.getModule(i).setAngle(0);
            swerveDrive.getModule(i).setSpeed(0.2);
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.stop();
    }

}
