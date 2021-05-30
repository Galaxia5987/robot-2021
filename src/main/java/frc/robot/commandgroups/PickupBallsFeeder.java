package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.LoadConveyor;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.funnel.commands.StartFunnel;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.StartIntake;

import java.util.function.DoubleSupplier;

public class PickupBallsFeeder extends ParallelCommandGroup {

    public PickupBallsFeeder(Intake intake, Funnel funnel, Conveyor conveyor, DoubleSupplier power, boolean up) {
        addCommands(
                new StartIntake(intake, power, up),
                new StartFunnel(funnel, true)
//                new LoadConveyor(conveyor, Constants.Conveyor.CONVEYOR_MOTOR_POWER)
        );
    }
}
