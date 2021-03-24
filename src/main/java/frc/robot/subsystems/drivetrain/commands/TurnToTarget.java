package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import java.util.function.DoubleSupplier;

public class TurnToTarget extends CommandBase {
    private SwerveDrive swerveDrive;
    private DoubleSupplier visionOutput;
    private PIDController pid = new PIDController(Constants.SwerveDrive.KP_TURN.get(), Constants.SwerveDrive.KI_TURN.get(), Constants.SwerveDrive.KD_TURN.get());
    private double target;

    public TurnToTarget(SwerveDrive swerveDrive, DoubleSupplier visionOutput) {
        this.swerveDrive = swerveDrive;
        this.visionOutput = visionOutput;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        target = Robot.navx.getYaw() + visionOutput.getAsDouble();
        pid.setSetpoint(target);
        pid.setP(Constants.SwerveDrive.KP_TURN.get());
        pid.setI(Constants.SwerveDrive.KI_TURN.get());
        pid.setD(Constants.SwerveDrive.KD_TURN.get());
    }

    @Override
    public void execute() {
        double power = pid.calculate(Robot.navx.getYaw());
        swerveDrive.holonomicDrive(0, 0, power);
        SmartDashboard.putNumber("gyro", Robot.navx.getYaw());
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return Math.abs(target - Robot.navx.getYaw()) < Constants.SwerveDrive.TURN_TOLERANCE;
    }
}
