package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.Shooter;
import org.ejml.ops.ConvertDMatrixStruct;
import webapp.FireLog;

import java.util.function.Supplier;

/**
 * This command keeps the shooter in the specified velocity.
 * This command should <b>NOT</b> be used in order to change the velocity applied by the shooter.
 * The command should come in conjunction with a button, and not to start/stop it after a specified time.
 */
public class Shoot extends CommandBase {
    private final Shooter shooter;
    private final Supplier<Double> velocity;
    private final Timer shootingTimer = new Timer();
    private double lastTime = 0;

    public Shoot(Shooter shooter, Supplier<Double> velocity) {
        this.shooter = shooter;
        this.velocity = velocity;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shootingTimer.start();
        shooter.setVelocity(velocity.get());
        shooter.setVelocityUp(velocity.get() / 2.0);
    }

    @Override
    public void execute() {
        final double currentTime = shootingTimer.get();
        shooter.setVelocity(velocity.get(), currentTime - lastTime);
        shooter.setVelocityUp(velocity.get() / 2.0, currentTime - lastTime);
        lastTime = currentTime;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        shootingTimer.stop();
        shootingTimer.reset();
    }
}
