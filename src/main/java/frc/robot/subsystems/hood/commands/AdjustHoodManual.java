package frc.robot.subsystems.hood.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.hood.Hood;
import org.techfire225.webapp.FireLog;

import java.util.function.Supplier;

public class AdjustHoodManual extends CommandBase {

    private final Hood hood;
    private final Supplier<Hood.State> state;

    public AdjustHoodManual(Hood hood, double distance) {
        this(hood, Hood.State.getOptimalState(distance));
    }

    public AdjustHoodManual(Hood hood, Hood.State state) {
        this(hood, ()-> state);
    }

    public AdjustHoodManual(Hood hood, Supplier<Hood.State> state) {
        this.hood = hood;
        this.state = state;

        addRequirements(hood);
    }

    @Override
    public void execute() {
        hood.updatePID();
        hood.changePosition(state.get());
        FireLog.log("setpoint", state.get().position.getAsInt());
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
