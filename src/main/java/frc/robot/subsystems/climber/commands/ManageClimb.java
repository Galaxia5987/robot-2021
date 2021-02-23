package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.subsystems.climber.Climber;

/**
 * The command will raise the climber to a given height.
 */
public class ManageClimb extends CommandBase {

    private final Climber climber;
    private final double height;

    public ManageClimb(Climber climber, double height) {
        this.climber = climber;
        this.height = height;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.setHeight(height);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(climber.getHeight() - height) <= Constants.Climber.HEIGHT_TOLERANCE;
    }
}
