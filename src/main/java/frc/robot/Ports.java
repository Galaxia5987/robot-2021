package frc.robot;

public class Ports {

    public static class PTO {
        public static final int MASTER = 32;
        public static final int SLAVE = 33;
        public static final int PISTON_REVERSE = 0;
    }

    public static class Climber {
        public static final int STOPPER_FORWARD_CHANNEL = 3;
        public static final int DRUM = 1;

        public static final boolean IS_SENSOR_PHASE_INVERTED = false;
        public static final boolean IS_MASTER_INVERTED = true;
        public static final boolean IS_SLAVE_INVERTED = true;
    }

    public static final class Shooter {
        public static final int UP = 31;
        public static final boolean MAIN_INVERTED = true;
        public static final boolean AUX_INVERTED = true;
        public static final boolean UP_INVERTED = false;
        public static final boolean IS_SENSOR_INVERTED = false;
    }

    public static final class Vision {
        public static final int SOLENOID = 4;
    }

    public static final class Hood {
        public static final int MOTOR = 34;
        public static final boolean IS_INVERTED = true;
        public static final boolean IS_SENSOR_INVERTED = true;
    }

    public static final class Intake {
        public static final int MOTOR = 18;
        public static final boolean IS_MOTOR_INVERTED = true;
        public static final int SOLENOID = 5;
    }

    public static final class Funnel {
        public static final int MOTOR = 11;
        public static final boolean IS_INVERTED = false;
        public static final int BLOP = 6;
    }

    public static final class Conveyor {
        public static final int MOTOR = 41;
        public static final boolean IS_MOTOR_INVERTED = true;

        public static final int LIMIT_SWITCH = 0;
    }

    public static class SwerveDrive {
        // front right
        public static final int DRIVE_MOTOR_FRONT_RIGHT = 23;
        public static final int ANGLE_MOTOR_FRONT_RIGHT = 24;
        public static final boolean[] FRONT_RIGHT_INVERTED = {true, false, true, false};

        // front left
        public static final int DRIVE_MOTOR_FRONT_LEFT = 27;
        public static final int ANGLE_MOTOR_FRONT_LEFT = 28;
        public static final boolean[] FRONT_LEFT_INVERTED = {false, false, false, false};

        // back right
        public static final int DRIVE_MOTOR_BACK_RIGHT = 21;
        public static final int ANGLE_MOTOR_BACK_RIGHT = 22;
        public static final boolean[] BACK_RIGHT_INVERTED = {false, false, false, true};

        // back left
        public static final int DRIVE_MOTOR_BACK_LEFT = 25;
        public static final int ANGLE_MOTOR_BACK_LEFT = 26;
        public static final boolean[] BACK_LEFT_INVERTED = {false, false, false, false};
    }

}
