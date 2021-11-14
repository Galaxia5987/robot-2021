package frc.robot.subsystems.drivetrain.commands.tests;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class TurnAround extends CommandBase {
    private double reqAngle = Robot.navx.getAngle() - ((Robot.navx.getAngle() < 0) ? (-180) : 180);
    private PIDController pidController = new PIDController(0.2, 0, 0.02);
    private SwerveDrive swerveDrive;

    private boolean inDeadBand(){
        double currAngle = Robot.navx.getAngle();
        return currAngle - reqAngle < 4;
    }

    public TurnAround(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        swerveDrive.holonomicDrive(0, 0, pidController.calculate(1));
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
