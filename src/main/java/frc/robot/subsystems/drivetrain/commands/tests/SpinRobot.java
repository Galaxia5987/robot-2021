package frc.robot.subsystems.drivetrain.commands.tests;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import java.util.ArrayList;

public class SpinRobot extends SequentialCommandGroup {
    SwerveDrive swerveDrive;

    public SpinRobot(SwerveDrive swerveDrive, int iterations) {
        this.swerveDrive = swerveDrive;

        double currAngle, lastAngle = Robot.navx.getYaw(), error;
        ArrayList<Double> error_list = new ArrayList<>();

        for (int i = 0; i < iterations; i++) {
            currAngle = Robot.navx.getYaw();

            turnAround();
            error = currAngle - lastAngle;

            System.out.println("The angle error after half a turn is: " +
                    error);
            lastAngle = currAngle;
            error_list.add(error);
        }

        System.out.println("The average angle error for every half turn is: " +
                calcMean(error_list));
        System.out.println("The standard deviation of angle errors for every half turn is: " +
                standardDeviation(error_list));
    }

    public void turnAround() {
        addCommands(
                new TurnAround(swerveDrive)
        );
    }

    public double calcMean(ArrayList<Double> error_list) {
        double sum = 0;
        for (Double error : error_list) {
            sum += error;
        }
        return sum / error_list.size();
    }

    public double standardDeviation(ArrayList<Double> error_list) {
        double sum = 0, mean = calcMean(error_list);
        for (Double error : error_list) {
            sum += Math.pow(error - mean, 2);
        }
        return sum / (error_list.size() - 1);
    }
}
