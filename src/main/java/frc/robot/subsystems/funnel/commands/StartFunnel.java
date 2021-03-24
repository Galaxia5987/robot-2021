package frc.robot.subsystems.funnel.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.funnel.Funnel;

/**
 * this command activates Funnel's motor
 */
public class StartFunnel extends CommandBase {

    private Funnel funnel;
    private boolean isMovingUp;
    private Timer timmy = new Timer();
    private double last = 0;

    public StartFunnel(Funnel funnel, boolean isMovingUp) {
        this.funnel = funnel;
        this.isMovingUp = isMovingUp;
        addRequirements(funnel);
    }

    @Override
    public void initialize() {
        timmy.reset();
        timmy.start();
        if (isMovingUp)
            funnel.setPower(Constants.Funnel.POWER);// in
        else
            funnel.setPower(-1 * Constants.Funnel.POWER);// out
    }

    @Override
    public void execute() {
        if (timmy.get() - last > 0.5) {
            funnel.toggle();
            last = timmy.get();
//            funnel.setPower(0);
        } else {
            if (isMovingUp)
                funnel.setPower(Constants.Funnel.POWER);// in
            else
                funnel.setPower(-1 * Constants.Funnel.POWER);// out
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        funnel.setPower(0);
        timmy.stop();
        last = 0;
    }
}