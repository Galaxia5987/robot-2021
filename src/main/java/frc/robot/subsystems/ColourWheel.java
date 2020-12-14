package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

public class ColourWheel extends SubsystemBase {

    private final TalonSRX motor = new TalonSRX(Ports.ColourWheel.MOTOR);

    private final I2C.Port port = I2C.Port.kOnboard;

    private final ColorSensorV3 colorSensorV3 = new ColorSensorV3(port);

    private final ColorMatch colorMatch = new ColorMatch();

    private String colorString = " ";

    private final Color RedTarget = ColorMatch.makeColor(Constants.ColourWheel.RED[0], Constants.ColourWheel.RED[1], Constants.ColourWheel.RED[2]);
    private final Color GreenTarget = ColorMatch.makeColor(Constants.ColourWheel.GREEN[0], Constants.ColourWheel.GREEN[1], Constants.ColourWheel.GREEN[2]);
    private final Color BlueTarget = ColorMatch.makeColor(Constants.ColourWheel.BLUE[0], Constants.ColourWheel.BLUE[1], Constants.ColourWheel.BLUE[2]);
    private final Color YellowTarget = ColorMatch.makeColor(Constants.ColourWheel.YELLOW[0], Constants.ColourWheel.YELLOW[1], Constants.ColourWheel.YELLOW[2]);

    public ColourWheel() {
        motor.setInverted(Ports.ColourWheel.MOTOR_INVERTED);
        motor.setSensorPhase(Ports.ColourWheel.MOTOR_SENSOR_PHASE_INVERTED);
        motor.config_kP(0, Constants.ColourWheel.kP);
        motor.config_kI(0, Constants.ColourWheel.kI);
        motor.config_kD(0, Constants.ColourWheel.kD);
    }


}
