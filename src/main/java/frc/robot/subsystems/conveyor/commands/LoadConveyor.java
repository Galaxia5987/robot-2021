package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj.Timer;
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
    private boolean last;

    public LoadConveyor(Conveyor conveyor, double power, Funnel funnel) {
        this.conveyor = conveyor;
        this.power = power;
        this.funnel = funnel;

        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        last = Conveyor.hasFunnelSensedObject();
       /* timer.reset();
        timer.start();
        powerConveyor = false;*/
    }

    @Override
    public void execute() {
       /* if (Conveyor.hasFunnelSensedObject() && !Conveyor.readLimitSwitch()) {
            funnel.setPower(0);
            conveyor.setPower(Constants.Conveyor.CONVEYOR_MOTOR_POWER);
        } else if (Conveyor.hasFunnelSensedObject()) {
            funnel.setPower(Constants.Funnel.POWER);
            conveyor.setPower(0);
        } else if (!Conveyor.readLimitSwitch()) {
            funnel.setPower(0);
            conveyor.setPower(Constants.Conveyor.CONVEYOR_MOTOR_POWER);
        } else {
            funnel.setPower(Constants.Funnel.POWER);
            conveyor.setPower(0);
        }*/
        //ADAMCODE
        /*if (timer.hasElapsed(0.4) || first) {
            first = false;
            if (Conveyor.hasFunnelSensedObject() && powerConveyor) {
                conveyor.setPower(Constants.Conveyor.CONVEYOR_MOTOR_POWER_SLOW);
                funnel.setPower(Constants.Funnel.POWER_SLOW);
            }

            if (Conveyor.hasFunnelSensedObject()) {
                conveyor.setPower(Constants.Conveyor.CONVEYOR_MOTOR_POWER_SLOW);
                timer.reset();
                timer.start();
            } else {
                funnel.setPower(Constants.Funnel.POWER_SLOW);
                conveyor.setPower(0);
                powerConveyor = false;
            }
        }*/
        if (Conveyor.hasFunnelSensedObject()) {
            conveyor.setPower(Constants.Conveyor.CONVEYOR_MOTOR_POWER_LOAD);
        }
        else {
            conveyor.setPower(0);
        }

/*        if (Conveyor.hasFunnelSensedObject()) {
            funnel.setPower(0);
            timer.reset(); timer.start();
            while (!timer.hasElapsed(0.5))
                conveyor.setPower(Constants.Conveyor.CONVEYOR_MOTOR_POWER_SLOW);
            conveyor.setPower(0);
            while (Conveyor.hasFunnelSensedObject())
                funnel.setPower(Constants.Funnel.POWER);
            funnel.setPower(0);
        } else {
            funnel.setPower(Constants.Funnel.POWER);
        }*/
/*
        if (Conveyor.hasFunnelSensedObject() && !timer.hasElapsed(2)) {
            timer.start();
            conveyor.setPower(moderatePower(timer.get(), 2));
        } else {
            conveyor.setPower(0);
            timer.stop();
            timer.reset();
        }*/

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
