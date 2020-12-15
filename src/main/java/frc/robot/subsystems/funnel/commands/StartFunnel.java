package frc.robot.subsystems.funnel.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.funnel.Funnel;

public class StartFunnel extends CommandBase {

    private Funnel funnel;

    public StartFunnel(Funnel f) {
        funnel = f;
        addRequirements(funnel);
    }

    @Override
    public void initialize() {
        funnel.setVelocity(Constants.Funnel.VELOCITY);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        funnel.setVelocity(0.0);
    }
}