package frc.robot.subsystems.PTO;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

/**
 *
 */
public class PTO extends SubsystemBase {
    private final TalonFX motor1 = new TalonFX(Ports.PTO.MOTOR_1);
    private final TalonFX motor2 = new TalonFX(Ports.PTO.MOTOR_2);
    private final DoubleSolenoid piston = new DoubleSolenoid(Ports.PTO.PISTON_FORWARD, Ports.PTO.PISTON_REVERSE);
    private GearboxState state = GearboxState.SHOOTER;

    public void configureClimber() {
        //Todo: take the configuration from the CLIMBER PR.
    }

    public void configureShooter() {
        //Todo: take the configuration from the SHOOTER PR.
    }

    public void changeState(GearboxState state) {
        this.state = state;
    }

    public GearboxState getState() {
        return state;
    }

    public TalonFX getMotor1() {
        return motor1;
    }

    public TalonFX getMotor2() {
        return motor2;
    }


    public void changePiston(boolean isClimber) {
        if (isClimber) {
            piston.set(DoubleSolenoid.Value.kForward);
            changeState(GearboxState.CLIMBER);
            configureClimber();
        } else {
            piston.set(DoubleSolenoid.Value.kReverse);
            changeState(GearboxState.CLIMBER);
            configureShooter();
        }
    }


    public enum GearboxState {
        SHOOTER, CLIMBER
    }
}
