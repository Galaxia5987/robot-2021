package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.Intake;

import java.util.function.DoubleSupplier;

public class StartIntake extends CommandBase {
    private final Intake intake;
    private final boolean direction;
    private final DoubleSupplier power;

    public StartIntake(Intake i, DoubleSupplier power, boolean direction) {
        intake = i;
        this.direction = direction;
        this.power = power;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        super.initialize();
        if (!intake.isOpen())
            intake.togglePiston();
    }

    @Override
    public void execute() {
        intake.setVelocity(direction ? power.getAsDouble() : -power.getAsDouble());// field --> funnel
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setVelocity(0);
    }
}
