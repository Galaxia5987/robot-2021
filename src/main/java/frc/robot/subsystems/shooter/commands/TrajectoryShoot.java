
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
public class TrajectoryShoot extends CommandBase {
    private final Shooter shooter;
    private final VisionModule vision;
    private final Hood hood;
    private final Timer shootingTimer = new Timer();
    private double lastTime = 0;

    public TrajectoryShoot(Shooter shooter, VisionModule vision, Hood hood) {
        this.shooter = shooter;
        this.vision = vision;
        this.hood = hood;
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
                double velocity = shooter.calculateVelocity(distance + Shooter.flyWheelDistanceAddition.get());
                shooter.setVelocity(velocity, currentTime - lastTime);
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
