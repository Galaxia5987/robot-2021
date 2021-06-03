package frc.robot.subsystems.conveyor;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

import static frc.robot.Constants.Conveyor.*;

public class Conveyor extends SubsystemBase {
    private static final ColorMatch colorMatcher = new ColorMatch();
    private static final ColorMatch colorMatcher2 = new ColorMatch();
    private static final Color BlueTarget = ColorMatch.makeColor(Constants.Conveyor.BLUE_RGB[0], Constants.Conveyor.BLUE_RGB[1], Constants.Conveyor.BLUE_RGB[2]);
    private static final Color GreenTarget = ColorMatch.makeColor(Constants.Conveyor.GREEN_RGB[0], Constants.Conveyor.GREEN_RGB[1], Constants.Conveyor.GREEN_RGB[2]);
    private static final Color RedTarget = ColorMatch.makeColor(Constants.Conveyor.RED_RGB[0], Constants.Conveyor.RED_RGB[1], Constants.Conveyor.RED_RGB[2]);
    private static final Color YellowTarget = ColorMatch.makeColor(Constants.Conveyor.YELLOW_RGB[0], Constants.Conveyor.YELLOW_RGB[1], Constants.Conveyor.YELLOW_RGB[2]);
    private static int balls = Constants.Conveyor.INITIAL_BALLS_AMOUNT;
    private static String colorString = " ";
    private static String colorStringNavx = "";
    public final I2C.Port i2cPort = I2C.Port.kOnboard;
    public final I2C.Port i2cPort2 = I2C.Port.kMXP;
    public final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
    public final ColorSensorV3 navxColorSensor = new ColorSensorV3(i2cPort2);
    private final TalonFX motor = new TalonFX(Ports.Conveyor.MOTOR);

    public Conveyor() {
        motor.setInverted(Ports.Conveyor.IS_MOTOR_INVERTED);
        colorMatcher.addColorMatch(BlueTarget);
        colorMatcher.addColorMatch(GreenTarget);
        colorMatcher.addColorMatch(RedTarget);
        colorMatcher.addColorMatch(YellowTarget);

        colorMatcher2.addColorMatch(BlueTarget);
        colorMatcher2.addColorMatch(GreenTarget);
        colorMatcher2.addColorMatch(RedTarget);
        colorMatcher2.addColorMatch(YellowTarget);

        motor.configPeakOutputForward(FORWARD_PEAK, Constants.TALON_TIMEOUT);
        motor.configPeakOutputReverse(REVERSE_PEAK, Constants.TALON_TIMEOUT);

        motor.configSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT, Constants.TALON_TIMEOUT);
        motor.configStatorCurrentLimit(STATOR_CURRENT_LIMIT, Constants.TALON_TIMEOUT);

        motor.enableVoltageCompensation(true);
        motor.configVoltageCompSaturation(Constants.NOMINAL_VOLTAGE);

    }

    /**
     * Get the amount of balls sensed by the proximity.
     *
     * @return the amount of balls in the conveyor.
     */
    private static int getBallsAmount() {
        return balls;
    }

    public static boolean isConveyorFull() {
        return getBallsAmount() >= MAX_BALLS_AMOUNT;
    }

    public static boolean isConveyorEmpty() {
        return getBallsAmount() <= 0;
    }

    /**
     * Increment by one the amount of balls (only programmatically).
     */
    private static void addBall() {
        balls++;
    }

    /**
     * Decrement by one the amount of balls (only programmatically).
     */
    private static void removeBall() {
        balls--;
    }

    /**
     * Get whether the funnel sensed an object.
     *
     * @return whether the funnel sensed an object.
     */
    public static boolean hasFunnelSensedObject() {
        return colorString.equals("Yellow");
    }

    public static boolean hasNavxFunnelSensedObject() {
        return colorStringNavx.equals("Yellow");
    }

    /**
     * Get whether the power applied by the motor is upward, i.e moving up.
     *
     * @return whether the power applied by the motor is upward.
     */
    private boolean isMovingUp() {
        return motor.getMotorOutputPercent() > 0;
    }

    /**
     * Set the power to apply by the motor of the conveyor.
     *
     * @param power the power to apply by the motor. [%]
     */
    public void setPower(double power) {
        motor.set(TalonFXControlMode.PercentOutput, power);
    }

    /**
     * Stop the motor.
     */
    public void stop() {
        setPower(0);
    }

    /*
    Returns the currently detected color as a string
     */
    private String colorToString(ColorMatchResult match) {
        String colorInString;
        if (match.color == BlueTarget) {
            colorInString = "Blue";
        } else if (match.color == RedTarget) {
            colorInString = "Red";
        } else if (match.color == GreenTarget) {
            colorInString = "Green";
        } else if (match.color == YellowTarget) {
            colorInString = "Yellow";
        } else {
            colorInString = "Unknown";
        }
        return colorInString;
    }

    @Override
    public void periodic() {

        Color detectedColor = colorSensor.getColor();
        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
        colorString = colorToString(match);
        SmartDashboard.putString("color", colorString);

        Color detectedColorNavx = navxColorSensor.getColor();
        ColorMatchResult matchNavx = colorMatcher2.matchClosestColor(detectedColorNavx);
        colorStringNavx = colorToString(matchNavx);
        SmartDashboard.putString("color-navx", colorStringNavx);
        SmartDashboard.putBoolean("robotrio sensor", Conveyor.hasFunnelSensedObject());
        SmartDashboard.putBoolean("navx sensor", Conveyor.hasNavxFunnelSensedObject());
    }
}
