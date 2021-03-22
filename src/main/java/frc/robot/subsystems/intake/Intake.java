package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
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
    private CANSparkMax motor = new CANSparkMax(Ports.Intake.MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private Solenoid solenoid = new Solenoid(Ports.Intake.SOLENOID);
    private boolean position;

    public Intake() {
        motor.setInverted(Ports.Intake.IS_MOTOR_INVERTED);
        position = false;
    }

    /**
     * @return the state of the solenoids
     */
    public boolean isOpen() {
        return position;
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
        solenoid.set(!position);
        position = !position;
    }
}
