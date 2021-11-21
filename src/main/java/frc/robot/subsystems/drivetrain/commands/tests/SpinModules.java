package frc.robot.subsystems.drivetrain.commands.tests;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.Rotate;

public class SpinModules extends SequentialCommandGroup {
    private int iterations;
    private double velocity;
    private SwerveDrive swerveDrive;

    public SpinModules(int iterations, double velocity, SwerveDrive swerveDrive) {
        this.iterations = iterations;
        this.velocity = velocity;
        this.swerveDrive = swerveDrive;

        boolean mode = true;

        for (int i = 0; i < iterations; i++) {
            spinModules(mode);
            mode = !mode;

            rotate();
        }
    }

    public void spinModules(boolean mode) {
        addCommands(
                new SpinModulesAux(swerveDrive, mode ? velocity : -velocity)
        );
    }

    public void rotate() {
        addCommands(
                new Rotate(swerveDrive, () -> 0)
        );
    }
}
