package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.VisionModule;
import frc.robot.valuetuner.WebConstant;
import webapp.FireLog;

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
    }

    @Override
    public void execute() {
        shooter.setVelocity(vel.get());
        FireLog.log("vel setpoint", vel.get());
        FireLog.log("current velocity", shooter.getVelocity());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
