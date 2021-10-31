package frc.robot.subsystems.drivetrain.commands.tests;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import static frc.robot.Constants.*;

public class VoltageTest extends CommandBase {
    private SwerveDrive swerveDrive = new SwerveDrive(false);
    private double cycles = 0;
    private double initVoltage;
    private double finalVoltage;

    @Override
    public void initialize() {
        initVoltage = RobotController.getBatteryVoltage();
    }

    @Override
    public void execute() {
        swerveDrive.setModuleOutputMax(0);
        swerveDrive.setModuleOutputMax(1);
        swerveDrive.setModuleOutputMax(2);
        swerveDrive.setModuleOutputMax(3);
        cycles += LOOP_PERIOD;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.terminate();
        finalVoltage = RobotController.getBatteryVoltage();
        System.out.println(initVoltage - finalVoltage);
    }

    @Override
    public boolean isFinished() {
        return (cycles >= 20);
    }
}
