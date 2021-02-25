package frc.robot.subsystems.PTO;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

/**
 * This subsystem define the PTO.
 * The PTO is responsible for switching between the climber and the shooter, because they are using the same motors.
 */
public class PTO extends SubsystemBase {
    private final TalonFX motor1 = new TalonFX(Ports.PTO.MOTOR_1);
    private final TalonFX motor2 = new TalonFX(Ports.PTO.MOTOR_2);
    private final DoubleSolenoid piston = new DoubleSolenoid(Ports.PTO.PISTON_FORWARD, Ports.PTO.PISTON_REVERSE);
    private GearboxState state = GearboxState.SHOOTER;

    /**
     * This Configure the motors with the configuration required for the climber.
     */
    public void configureClimber() {
        //Todo: take the configuration from the CLIMBER PR.
    }

    /**
     * This Configure the motors with the configuration required for the shooter.
     */
    public void configureShooter() {
        //Todo: take the configuration from the SHOOTER PR.
    }

    /**
     * This method changes the current state of the gearboxState.
     * @param state the new state.
     */
    private void changeState(GearboxState state) {
        this.state = state;
    }

    /**
     * @return the current GearboxState of the subsystem.
     */
    public GearboxState getState() {
        return state;
    }

    /**
     * This method returns the motor controller for the climber and the shooter to use.
     * @return the motor controller of motor 1.
     */
    public TalonFX getMotor1() {
        return motor1;
    }

    /**
     * This method returns the motor controller for the climber and the shooter to use.
     * @return the motor controller of motor 1.
     */
    public TalonFX getMotor2() {
        return motor2;
    }

    /**
     * This method switches the piston to change the subsystem.
     * After that this method configure the motors according to the new state and change the current state.
     * @param isClimber whether the new state is the climber subsystem.
     */
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

    /**
     * This enum contain the possible states of the PTO:
     * Climber and Shooter.
     */
    public enum GearboxState {
        SHOOTER, CLIMBER
    }
}
