package frc.robot.subsystems.funnel.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.funnel.Funnel;

/**
 * this command activates Funnel's motor
 */
public class StartFunnel extends CommandBase {

    private Funnel funnel;
    private boolean direction;

    public StartFunnel(Funnel funnel, boolean direction) {
        this.funnel = funnel;
        this.direction = direction;
        addRequirements(funnel);
    }

    @Override
    public void initialize() {
        if (direction)
            funnel.setVelocity(Constants.Funnel.POWER);// in
        else
            funnel.setVelocity(-1 * Constants.Funnel.POWER);// out
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        funnel.setVelocity(0);
    }
}