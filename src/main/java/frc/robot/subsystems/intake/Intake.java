package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

/**
 * this subsystem takes the balls from the field to the funnel.
 */
public class Intake extends SubsystemBase {
    private final TalonFX motor = new TalonFX(Ports.Intake.MOTOR);
    private final Solenoid solenoid = new Solenoid(Ports.Intake.SOLENOID);

    public Intake() {
        motor.setInverted(Ports.Intake.IS_MOTOR_INVERTED);
    }

    /**
     * @return the state of the solenoids
     */
    public boolean isOpen() {
        return solenoid.get();
    }

    /**
     * this function sets the motor's velocity
     *
     * @param speed - the target velocity (m/s)
     */
    public void setVelocity(double speed) {
        motor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * toggles piston's state (opened to closed || closed to opened)
     */
    public void togglePiston() {
        solenoid.toggle();
    }
}
