package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.funnel.Funnel;


/**
 * This command load the conveyor, as long as the conveyor isn't "out of space".
 */
public class LoadConveyor extends CommandBase {
    private final Conveyor conveyor;
    private final double power;
    private final Timer timer = new Timer();
    private final Funnel funnel;

    public  LoadConveyor(Conveyor conveyor, double power, Funnel funnel) {
        this.conveyor = conveyor;
        this.power = power;
        this.funnel = funnel;

        addRequirements(conveyor);
    }

    @Override
    public void execute() {
       /* if (Conveyor.hasFunnelSensedObject() && Conveyor.hasNavxFunnelSensedObject()) {
            funnel.setPower(0);
            conveyor.setPower(Constants.Conveyor.CONVEYOR_MOTOR_POWER);
        } else if (Conveyor.hasNavxFunnelSensedObject()) {
            funnel.setPower(Constants.Funnel.POWER);
            conveyor.setPower(0);
        } else if (Conveyor.hasFunnelSensedObject()) {
            funnel.setPower(0);
            conveyor.setPower(Constants.Conveyor.CONVEYOR_MOTOR_POWER);
        } else {
            funnel.setPower(Constants.Funnel.POWER);
            conveyor.setPower(0);
        }
*/

        if (Conveyor.hasFunnelSensedObject() && !timer.hasElapsed(2)) {
            timer.start();
            conveyor.setPower(moderatePower(timer.get(), 2));
        } else {
            conveyor.setPower(0);
            timer.stop();
            timer.reset();
        }

        // TODO: turn on LEDs to notify that the conveyor is full
        // TODO: Add override option in case that the sensor is "broken"

    }


    private double moderatePower(double elapsedTime, double cycleTime) {
        if (elapsedTime > cycleTime) return power;
        double m = power / cycleTime;
        return Math.min(power, m * elapsedTime);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
//        timer.stop();
        funnel.setPower(0);
    }
}
