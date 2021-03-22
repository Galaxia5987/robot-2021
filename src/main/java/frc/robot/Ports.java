package frc.robot;

/**
 * A class holding all of the ports on the robot.
 * Place mechanism-specific ports inside their own sub-class.
 * When accessing a mechanism-specific port, call Ports.[MECHANISM].[PORT_NAME]
 */
public class Ports {
    public static class ExampleSubsystem1 {
        //public static int TALON_PORT = 1;
    }

    public static class SwerveDrive {
        // front right
        public static final int DRIVE_MOTOR_FRONT_RIGHT = 27;
        public static final int ANGLE_MOTOR_FRONT_RIGHT = 28;
        public static final boolean[] FRONT_RIGHT_INVERTED = {false, false, false, false};

        // front left
        public static final int DRIVE_MOTOR_FRONT_LEFT = 23;
        public static final int ANGLE_MOTOR_FRONT_LEFT = 24;
        public static final boolean[] FRONT_LEFT_INVERTED = {true, false, true, false};

        // back right
        public static final int DRIVE_MOTOR_BACK_RIGHT = 25;
        public static final int ANGLE_MOTOR_BACK_RIGHT = 26;
        public static final boolean[] BACK_RIGHT_INVERTED = {false, false, false, false};

        // back left
        public static final int DRIVE_MOTOR_BACK_LEFT = 21;
        public static final int ANGLE_MOTOR_BACK_LEFT = 22;
        public static final boolean[] BACK_LEFT_INVERTED = {false, false, false, true};

        // encoder not continuous feedback
        public static final boolean IS_NOT_CONTINUOUS_FEEDBACK = false;
    }

}
