package frc.robot.subsystems.drivetrain.autonomous;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class MoveForward extends CommandBase {

    private final SwerveDrive swerveDrive;
    private final double velocity;

    public MoveForward(SwerveDrive swerveDrive, double velocity) {
        this.swerveDrive = swerveDrive;
        this.velocity = velocity;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        for (int i = 0; i < 4; i++) {
            swerveDrive.getModule(i).setState(new SwerveModuleState(velocity, new Rotation2d(0)));
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.stop();
    }

}
