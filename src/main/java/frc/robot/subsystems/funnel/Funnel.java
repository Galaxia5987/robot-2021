package frc.robot.subsystems.funnel;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

/**
 * this subsystem transfers balls from intake to conveyor
 */
public class Funnel extends SubsystemBase {
    private VictorSPX motor = new VictorSPX(Ports.Funnel.MOTOR);

    public Funnel() {
        motor.setInverted(Ports.Funnel.IS_INVERTED);
    }

    /**
     * this function sets the motor's velocity
     * @param power - the target output (%)
     */
    public void setPower(double power) {
        motor.set(ControlMode.PercentOutput , power);
    }
}
