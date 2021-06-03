package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;


/**
 * Move all of the Power Cells from the mechanical stopper to the intake proximity, to prevent gaps between them
 */
public class MinimizeConveyor extends CommandBase {
    private final Conveyor conveyor;

    public MinimizeConveyor(Conveyor conveyor) {
        this.conveyor = conveyor;
        addRequirements(conveyor);
    }

    @Override
    public void execute() {
        conveyor.setPower(Constants.Conveyor.CONVEYOR_MOTOR_RETURN_POWER);
    }

    @Override
    public boolean isFinished() {
        return Conveyor.hasFunnelSensedObject();
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
    }
}
