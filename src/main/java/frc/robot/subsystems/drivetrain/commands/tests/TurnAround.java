package frc.robot.subsystems.drivetrain.commands.tests;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class TurnAround extends CommandBase {
    private double reqAngle;
    private PIDController pidController = new PIDController(0.2, 0, 0.02);
    private SwerveDrive swerveDrive;

    private boolean inDeadBand(){
        double currAngle = Robot.navx.getYaw();
        return Math.abs(currAngle - reqAngle) < 8 / 4.0;
    }

    public TurnAround(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        reqAngle = Robot.navx.getYaw() - ((Robot.navx.getYaw() < 0) ? (-180) : 180);
        System.out.println("reqAngle1: " + reqAngle + ", currentAngle1: " + Robot.navx.getYaw());
    }

    @Override
    public void execute() {
        swerveDrive.holonomicDrive(0, 0, Math.PI / 3);
        System.out.println("reqAngle: " + reqAngle + ", currentAngle: " + Robot.navx.getYaw());
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.terminate();
    }

    @Override
    public boolean isFinished() {
        return inDeadBand();
    }
}
