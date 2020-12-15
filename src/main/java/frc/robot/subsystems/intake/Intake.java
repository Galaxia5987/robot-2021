package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.UnitModel;

public class Intake extends SubsystemBase {
    private TalonSRX motor = new TalonSRX(Ports.Intake.MOTOR);
    private Solenoid solenoidR = new Solenoid(Ports.Intake.SOLENOID_RIGHT);
    private Solenoid solenoidL = new Solenoid(Ports.Intake.SOLENOID_LEFT);
    private State position = State.CLOSE;

    /**
     * sets the motor inverted
     */
    public Intake() {
        motor.setInverted(Ports.Intake.IS_INVERTED);
    }

    /**
     * @return the state of the solenoids
     */
    public boolean isOpen(){return position == State.OPEN;}

    /**
     * sets the output of intake's motor by present output (%)
     */
    public void setPower(double power){
        motor.set(ControlMode.PercentOutput,power);
    }

    /**
     * toggles piston's state (opened --> closed || closed -- > opened)
     */
    public void togglePiston(){
        if(position == State.OPEN){
            position = State.CLOSE;
        }
        else{
            position = State.OPEN;
        }
        
        solenoidL.set(!isOpen());
        solenoidR.set(!isOpen());

    }

    public enum State{
        OPEN,
        CLOSE
    }
}
