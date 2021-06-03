package frc.robot.subsystems.hood.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.hood.Hood;
import org.techfire225.webapp.FireLog;

import java.util.function.IntSupplier;

public class AdjustHoodDebug extends CommandBase {

    private final Hood hood;
    private final IntSupplier position;

    public AdjustHoodDebug(Hood hood, IntSupplier position) {
        this.hood = hood;
        this.position = position;

        addRequirements(hood);
    }

    @Override
    public void execute() {
        hood.updatePID();
        hood.changePosition(position.getAsInt());
        FireLog.log("setpoint", position.getAsInt());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        hood.stop();
    }
}
