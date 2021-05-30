package frc.robot.subsystems.hood.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.hood.Hood;
import org.techfire225.webapp.FireLog;

public class AdjustHood extends CommandBase {

    private final Hood hood;
    private final Hood.State state;

    public AdjustHood(Hood hood, double distance) {
        this(hood, Hood.State.getOptimalState(distance));
    }

    public AdjustHood(Hood hood, Hood.State state) {
        this.hood = hood;
        this.state = state;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        hood.setHoodPID();
        hood.changeHoodState(state);
        FireLog.log("setpoint", state.position);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(hood.getHoodPosition() - state.position) < Constants.Shooter.POSITION_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        hood.stopHood();
    }
}
