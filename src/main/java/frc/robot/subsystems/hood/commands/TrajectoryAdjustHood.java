package frc.robot.subsystems.hood.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.VisionModule;
import org.techfire225.webapp.FireLog;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TrajectoryAdjustHood extends CommandBase {

    private final Hood hood;
    private final Shooter shooter;
    private final DoubleSupplier distance;

    public TrajectoryAdjustHood(Hood hood, Shooter shooter, DoubleSupplier distance) {
        this.hood = hood;
        this.shooter = shooter;
        this.distance = distance;
        addRequirements(hood);
    }

    @Override
    public void execute() {
        hood.updatePID();
        hood.changePosition(shooter.calculateAngle(distance.getAsDouble() + Shooter.hoodDistanceAddition.get()));
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
