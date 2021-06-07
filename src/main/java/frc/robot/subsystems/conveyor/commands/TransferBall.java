package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.funnel.Funnel;

public class TransferBall extends CommandBase {
    private final Funnel funnel;

    public TransferBall(Funnel funnel) {
        this.funnel = funnel;
        addRequirements(funnel);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        funnel.setPower(Constants.Funnel.POWER);
    }

    @Override
    public void end(boolean interrupted) {
        funnel.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return !Conveyor.hasFunnelSensedObject();
    }
}
