package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;

/**
 * This command load the conveyor, as long as the conveyor isn't "out of space".
 */
public class LoadConveyor extends CommandBase {
    private final Conveyor conveyor;
    private final double power;

    public LoadConveyor(Conveyor conveyor, double power) {
        this.conveyor = conveyor;
        this.power = power;

        addRequirements(conveyor);
    }

    @Override
    public void execute() {
        if (!Conveyor.isConveyorFull()) {
            if (conveyor.hasFunnelSensedObject())
               conveyor.setPower(power);
            else
                conveyor.setPower(0);
        } else {
            // TODO: turn on LEDs to notify that the conveyor is full
            conveyor.setPower(0);
        }
        // TODO: Add override option in case that the sensor is "broken"

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
    }
}
