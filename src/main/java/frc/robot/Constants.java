package frc.robot;

import frc.robot.valuetuner.WebConstant;

/**
 * A class holding all of the constants of every mechanism on the robot.
 * Place global constants in this class, and mechanism-specific constants inside their respective mechanism subclass.
 * When accessing a mechanism-specific port, call Constants.[MECHANISM].[CONSTANT]
 */
public class Constants {
    public static final int TALON_TIMEOUT = 10; // in ms
    public static final boolean ENABLE_CURRENT_LIMIT = true;

    public static class SwerveDrive {

        public static final double TICKS_PER_METER = 2048 / (4 * 0.0254 * Math.PI) * 7.5;
        public static final int TICKS_IN_ENCODER = 1024;
        public static final double TICKS_PER_RAD = TICKS_IN_ENCODER / (2 * Math.PI);

        public static final int MAX_CURRENT = 35; // in ampere

        public static final double ROBOT_LENGTH = 0.75; // in meters
        public static final double ROBOT_WIDTH = 0.75; // in meters

        // the speed of the robot, this constant multiplies the speed outputs from the joysticks
        public static final double SPEED_MULTIPLIER = 4 / Math.sqrt(2);

        // the rotational speed of the robot, this constant multiplies the rotation output of the joystick
        public static final double ROTATION_MULTIPLIER = 1.5 * Math.PI;

        public static final double JOYSTICK_THRESHOLD = 0.1;

        public static final double TURN_TOLERANCE = 2; // degrees

        public static final WebConstant KP_TURN = new WebConstant("KP_TURN", 0);
        public static final WebConstant KI_TURN = new WebConstant("KP_TURN", 0);
        public static final WebConstant KD_TURN = new WebConstant("KP_TURN", 0);
    }

    public static class SwerveModule {
        public static final WebConstant KP = new WebConstant("KP", 6);
        public static final WebConstant KI = new WebConstant("KI", 0);
        public static final WebConstant KD = new WebConstant("KD", 10);
        public static final WebConstant KF = new WebConstant("KF", 0);
        public static final WebConstant[] ANGLE_PIDF = new WebConstant[]{KP, KI, KD, KF};

        public static final WebConstant KP_DRIVE = new WebConstant("KP_Drive", 0.15);
        public static final WebConstant KI_DRIVE = new WebConstant("KI_Drive", 0);
        public static final WebConstant KD_DRIVE = new WebConstant("KD_Drive", 2);
        public static final WebConstant KF_DRIVE = new WebConstant("KF_Drive", 0.05);
        public static final WebConstant[] DRIVE_PIDF = new WebConstant[]{KP_DRIVE, KI_DRIVE, KD_DRIVE, KF_DRIVE};

        // slow man
        // the module that is slower than the rest
        public static final WebConstant KP_DRIVE_SLOW = new WebConstant("KP_Drive_Slow", 0.046);
        public static final WebConstant KI_DRIVE_SLOW = new WebConstant("KI_Drive_Slow", 0);
        public static final WebConstant KD_DRIVE_SLOW = new WebConstant("KD_Drive_Slow", 2);
        public static final WebConstant KF_DRIVE_SLOW = new WebConstant("KF_Drive_Slow", 0);
        public static final WebConstant[] SLOW_DRIVE_PIDF = new WebConstant[]{KP_DRIVE_SLOW, KI_DRIVE_SLOW, KD_DRIVE_SLOW, KF_DRIVE_SLOW};


        public static final WebConstant KP_BROKEN = new WebConstant("KP_BROKEN", 0.065);
        public static final WebConstant KI_BROKEN = new WebConstant("KI_BROKEN", 0);
        public static final WebConstant KD_BROKEN = new WebConstant("KD_BROKEN", 2);
        public static final WebConstant KF_BROKEN = new WebConstant("KF_BROKEN", 0.046);

        public static final int[] ZERO_POSITION = {-903, 339, 102, 124};

        // sick man
        // the module that has more friction in the rotating mechanism
        public static final WebConstant KP_SICK = new WebConstant("KP_SICK", 6);
        public static final WebConstant KI_SICK = new WebConstant("KI_SICK", 0);
        public static final WebConstant KD_SICK = new WebConstant("KD_SICK", 8);
        public static final WebConstant KF_SICK = new WebConstant("KF_SICK", 0);
        public static final WebConstant[] SICK_ANGLE_PIDF = new WebConstant[]{KP_SICK, KI_SICK, KD_SICK, KF_SICK};

        public static final int TRIGGER_THRESHOLD_CURRENT = 5; // ampere
        public static final double TRIGGER_THRESHOLD_TIME = 0.02; // seconds
        public static final boolean ENABLE_VOLTAGE_COMPENSATION = true;
        public static final int VOLTAGE_SATURATION = 12; // volts

    }

}
