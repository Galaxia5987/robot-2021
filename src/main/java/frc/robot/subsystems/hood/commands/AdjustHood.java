package frc.robot.subsystems.hood.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.utils.VisionModule;
import org.techfire225.webapp.FireLog;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AdjustHood extends CommandBase {

    private final Hood hood;
    private final Supplier<Hood.State> state;

    public AdjustHood(Hood hood, VisionModule vision, DoubleSupplier distance) {
        this(hood, () -> Hood.State.getOptimalState(vision, distance.getAsDouble()));
    }

    public AdjustHood(Hood hood, Hood.State state) {
        this(hood, () -> state);
    }

    public AdjustHood(Hood hood, Supplier<Hood.State> state) {
        this.hood = hood;
        this.state = state;

        addRequirements(hood);
    }

    @Override
    public void execute() {
        hood.updatePID();
        hood.changePosition(state.get());
        FireLog.log("setpoint", state.get().position.getAsInt());
        SmartDashboard.putString("hood-state", state.get().toString());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(hood.getPosition() - state.get().position.getAsInt()) < Constants.Hood.POSITION_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        hood.stop();
    }
}
