package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.VisionModule;
import frc.robot.valuetuner.WebConstant;

/**
 * This command keeps the shooter in the specified velocity.
 * This command should <b>NOT</b> be used in order to change the velocity applied by the shooter.
 * The command should come in conjunction with a button, and not to start/stop it after a specified time.
 */
public class Shoot extends CommandBase {
    private final Shooter shooter;
    private final VisionModule vision;
    private final Hood hood;
    private final Timer shootingTimer = new Timer();
    private final boolean manual;
    public WebConstant vel = new WebConstant("velocity", false ? 0 : 20);
    private double lastTime = 0;

    public Shoot(Shooter shooter, VisionModule vision, Hood hood, boolean manual) {
        this.shooter = shooter;
        this.vision = vision;
        this.hood = hood;
        this.manual = manual;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shootingTimer.start();
    }

    @Override
    public void execute() {
        final double currentTime = shootingTimer.get();
        var optionalDistance = vision.getTargetRawDistance();
        if (optionalDistance.isPresent()) {
            double distance = optionalDistance.getAsDouble();
            if (distance > 0) {
                SmartDashboard.putNumber("vision-distance", distance);
                double velocity = hood.estimateVelocityFromDistance(distance);
                if (manual) {
                    velocity = vel.get();
                }
                SmartDashboard.putNumber("hood-velocity", velocity);
                shooter.setVelocity(velocity + 5, currentTime - lastTime);
            }
        }
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
