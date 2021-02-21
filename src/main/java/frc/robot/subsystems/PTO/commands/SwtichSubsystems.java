package frc.robot.subsystems.PTO.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.PTO.PTO;

public class SwtichSubsystems extends InstantCommand {
    private PTO pto;
    private boolean isClimber;

    public SwtichSubsystems(PTO pto, boolean isClimber){
        this.pto = pto;
        this.isClimber = isClimber;
        addRequirements(pto);
    }

    @Override
    public void initialize(){
        pto.changePiston(isClimber);
        if (isClimber){
            pto.changeState(PTO.gearboxState.climber);
        }else {
            pto.changeState(PTO.gearboxState.shooter);
        }

    }

}
