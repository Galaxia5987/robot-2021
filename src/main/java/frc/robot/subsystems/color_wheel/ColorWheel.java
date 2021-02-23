package frc.robot.subsystems.color_wheel;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

/**
 * Color wheel values and functions.
 */
public class ColorWheel extends SubsystemBase {

    private final TalonSRX motor = new TalonSRX(Ports.ColorWheel.MOTOR);

    private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    private final ColorMatch colorMatch = new ColorMatch();

    private String colorString = "";

    private ColorMatchResult matchResult = new ColorMatchResult(Color.kBlack, 0);

    public ColorWheel() {
        motor.setInverted(Ports.ColorWheel.MOTOR_INVERTED);
        motor.setSensorPhase(Ports.ColorWheel.MOTOR_SENSOR_PHASE_INVERTED);

        motor.configVoltageCompSaturation(Constants.VOLTAGE);
        motor.enableVoltageCompensation(true);

        colorMatch.addColorMatch(Constants.ColorWheel.RED_TARGET);
        colorMatch.addColorMatch(Constants.ColorWheel.GREEN_TARGET);
        colorMatch.addColorMatch(Constants.ColorWheel.BLUE_TARGET);
        colorMatch.addColorMatch(Constants.ColorWheel.YELLOW_TARGET);
    }

    /**
     * Set power for the Color Wheel.
     *
     * @param percent designated percentages for the Color Wheel's power.
     */
    public void setPower(double percent) {
        motor.set(ControlMode.PercentOutput, percent);
    }

    /**
     * Convert the color seen by the color sensor to String.
     *
     * @return the color seen by the color sensor in String.
     */
    private String colorToString() {
        if (Constants.ColorWheel.RED_TARGET.equals(matchResult.color)) {
            return "RED";
        } else if (Constants.ColorWheel.GREEN_TARGET.equals(matchResult.color)) {
            return "GREEN";
        } else if (Constants.ColorWheel.BLUE_TARGET.equals(matchResult.color)) {
            return "BLUE";
        } else if (Constants.ColorWheel.YELLOW_TARGET.equals(matchResult.color)) {
            return "YELLOW";
        }
        return "UNKNOWN";
    }

    /**
     * Match the color seen by the color sensor to a color from the list and set private String colorString to the String of the color.
     */
    public void updateSensor() {
        matchResult = colorMatch.matchClosestColor(colorSensor.getColor());
        colorString = colorToString();
    }

    /**
     * Get the color of the closest color in String.
     *
     * @return the color of the closest color in String.
     */
    public String getColorString() {
        return colorString;
    }
}

