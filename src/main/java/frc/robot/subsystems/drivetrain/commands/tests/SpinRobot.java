package frc.robot.subsystems.drivetrain.commands.tests;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import static frc.robot.Constants.LOOP_PERIOD;

public class SpinRobot extends CommandBase {
    private PIDController pidController = new PIDController(0.1, 0, 0);
    private final int reqRotations = 6;
    private double lastAngle = 0;
    private SwerveDrive swerveDrive;
    private int rotations = 0;
    private boolean inDeadzone = false;

    public SpinRobot(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    private boolean inDeadzone(double angle){
        return (Math.abs(angle) < 2);
    }

    @Override
    public void initialize() {
        Robot.navx.reset();
    }

    @Override
    public void execute() {
        if(!inDeadzone(Robot.navx.getYaw()) && inDeadzone)
            inDeadzone = false;

        if(rotations < reqRotations - 1)
            swerveDrive.holonomicDrive(0, 0, 3);
        else
            swerveDrive.holonomicDrive(0,0, swerveDrive.doRotation(Robot.navx.getYaw(), pidController) / 360);

        if (inDeadzone(Robot.navx.getYaw()) && !inDeadzone) {
            inDeadzone = true;
            rotations++;
        }
        lastAngle = Robot.navx.getYaw();
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.terminate();
        System.out.println(Robot.navx.getYaw());
    }

    @Override
    public boolean isFinished() {
        return rotations == reqRotations;
    }
}
