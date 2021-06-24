package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;

import java.util.function.DoubleSupplier;

public class PercentClimb extends CommandBase {

    private final Climber climber;
    private final DoubleSupplier power;

    public PercentClimb(Climber climber, DoubleSupplier power) {
        this.climber = climber;
        this.power = power;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.setPower(power.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        climber.setStopperMode(Climber.PistonMode.CLOSED);
    }
}
