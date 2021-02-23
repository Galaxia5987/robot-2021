package frc.robot.subsystems.funnel.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.funnel.Funnel;

/**
 * this command activates Funnel's motor
 */
public class StartFunnel extends CommandBase {

    private Funnel funnel;
    private boolean isMovingUp;

    public StartFunnel(Funnel funnel, boolean isMovingUp) {
        this.funnel = funnel;
        this.isMovingUp = isMovingUp;
        addRequirements(funnel);
    }

    @Override
    public void initialize() {
        if (isMovingUp)
            funnel.setPower(Constants.Funnel.POWER);// in
        else
            funnel.setPower(-1 * Constants.Funnel.POWER);// out
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        funnel.setPower(0);
    }
}