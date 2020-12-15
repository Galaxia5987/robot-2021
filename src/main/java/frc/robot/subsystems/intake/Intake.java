package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.UnitModel;

/**
 * this subsystem takes the balls from the field to the funnel.
 */
public class Intake extends SubsystemBase {
    private PWMSparkMax motor = new PWMSparkMax(Ports.Intake.MOTOR);
    private Solenoid solenoidR = new Solenoid(Ports.Intake.SOLENOID_RIGHT);
    private Solenoid solenoidL = new Solenoid(Ports.Intake.SOLENOID_LEFT);
    private State position;

    public Intake() {
        motor.setInverted(Ports.Intake.IS_INVERTED);

        if (isOpen())
            position = State.OPEN;
        else
            position = State.CLOSE;
    }

    /**
     * @return the state of the solenoids
     */
    public boolean isOpen() {
        return position == State.OPEN;
    }

    /**
     * this function sets the motor's velocity
     * @param speed - the target velocity (m/s)
     */
    public void setVelocity(double speed) {
        motor.set(speed);
    }

    /**
     * toggles piston's state (opened --> closed || closed -- > opened)
     */
    public void togglePiston() {
        if (position == State.OPEN) {
            position = State.CLOSE;
        } else {
            position = State.OPEN;
        }

        solenoidL.set(!isOpen());
        solenoidR.set(!isOpen());

    }

    public enum State {
        OPEN,
        CLOSE
    }
}
