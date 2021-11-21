package frc.robot.subsystems.drivetrain.commands.tests;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.Rotate;

public class ManiacDrive extends SequentialCommandGroup {
    SwerveDrive swerveDrive;
    double velocity;
    private int iterations;

    public ManiacDrive(SwerveDrive swerveDrive, double velocity, int iterations) {
        this.swerveDrive = swerveDrive;
        this.velocity = velocity;
        this.iterations = iterations;

        double currAngle, angle_error;
        double[] angle_error_list = new double[iterations];

        rotate();

        for (int i = 0; i < iterations; i++) {
            driveAndTurn();

            currAngle = Robot.navx.getYaw();
            angle_error = currAngle - Robot.navx.getYaw();
            System.out.println("Error of angle is: " + angle_error);
            angle_error_list[i] = angle_error;
        }

        System.out.println("The average angle error is: " + calcMean(angle_error_list, iterations));
        System.out.println("The standard deviation of angle error is: "
                + standardDeviation(angle_error_list));

        rotate();
    }

    public double calcMean(double[] error_list, int iterations) {
        double sum = 0;
        for (Double error : error_list) {
            sum += error;
        }
        return sum / iterations;
    }

    public double standardDeviation(double[] error_list) {
        double sum = 0, mean = calcMean(error_list, iterations);
        for (Double error : error_list) {
            sum += Math.pow(error - mean, 2);
        }
        return sum / (iterations - 1);
    }

    public void driveAndTurn() {
        addCommands(
                new DriveVelocity(velocity, swerveDrive), new TurnAround(swerveDrive)
        );
    }

    public void rotate() {
        addCommands(
                new Rotate(swerveDrive, () -> 0)
        );
    }
}
