package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

public class StartIntake extends CommandBase {
    private Intake intake;
    public StartIntake(Intake i) {
        intake = i;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setVelocity(Constants.Intake.VELOCITY);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setVelocity(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
