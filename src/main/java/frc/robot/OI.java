package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class OI {

    private static final Joystick rightJoystick = new Joystick(0);
    public static XboxController xbox = new XboxController(2);
    public static Button b = new JoystickButton(xbox, 1);
    public static JoystickButton a = new JoystickButton(xbox, XboxController.Button.kA.value);
    public static JoystickButton c = new JoystickButton(rightJoystick, 3);


    /**
     * @return the X value of the joystick
     */
    public static double getJoystickX() {
        return rightJoystick.getX();
    }

    /**
     * @return the Y value of the joystick
     */
    public static double getJoystickY() {
        return rightJoystick.getY();
    }

    /**
     * @return the Z value of the joystick
     */
    public static double getJoystickZ() {
        return rightJoystick.getZ();
    }
}
