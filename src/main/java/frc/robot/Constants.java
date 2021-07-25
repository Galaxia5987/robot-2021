package frc.robot;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.valuetuner.WebConstant;
import frc.robot.valuetuner.WebConstantPIDTalon;

/**
 * A class holding all of the constants of every mechanism on the robot.
 * Place global constants in this class, and mechanism-specific constants inside their respective mechanism subclass.
 * When accessing a mechanism-specific port, call Constants.[MECHANISM].[CONSTANT]
 */
public final class Constants {

    public static final double LOOP_PERIOD = 0.02; // [s]
    public static final double NOMINAL_VOLTAGE = 12; // [volts]
    public static final int TALON_TIMEOUT = 10; // Waiting period for configurations [ms].
    public static final boolean ENABLE_CURRENT_LIMIT = true;

    public static class Climber {
        public static final double TICKS_PER_METER = 2048 * 0.03 * Math.PI * 100;

        public static final WebConstant KP = new WebConstant("kP", 0);
        public static final WebConstant KI = new WebConstant("kI", 0);
        public static final WebConstant KD = new WebConstant("kD", 0);
        public static final WebConstant KF = new WebConstant("kF", 0);

        public static final int CRUISE_VELOCITY = 0; //sensor units per 100ms.
        public static final int ACCELERATION = 0; //sensor units per 100ms^2.

        public static final double HEIGHT_TOLERANCE = 0.1; //Error tolerance for the height [m].
    }

    //TODO: change to real values
    public static final class Shooter {
        public static final int TICKS_PER_ROTATION = 2048;

        // NOTE: these are the only constants you need to change.
        // TODO: Calibrate
        public static final double VELOCITY_TOLERANCE = 10; // [RPS]
        public static final double MODEL_TOLERANCE = 2;
        public static final double ENCODER_TOLERANCE = 1; // [ticks]
        public static final WebConstant J = new WebConstant("J", 0.000429); // moment of inertia [kg * m^2]
        public static final double ARBITRARY_FEED_FORWARD = 0; // [%] https://docs.ctre-phoenix.com/en/latest/ch16_ClosedLoop.html#do-i-need-to-use-arbitrary-feed-forward


        // TODO: Note that we might need to change the values here using the frc-characterizations if the model won't satisfy our needs.
        public static final double STALL_CURRENT = 257; // [amps]
        public static final double STALL_TORQUE = 4.69; // [N*meters]

        public static final double FREE_CURRENT = 1.5; // [amps]
        public static final double FREE_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(6380); // [rad/sec]

        public static final double GEAR_RATIO = 0.987654321; //TODO: Choose real value.
        public static final double kT = STALL_TORQUE / STALL_CURRENT;// took from FRC examples.
        public static final double OMEGA = NOMINAL_VOLTAGE / STALL_CURRENT; // [Ohm]
        public static final double kV = FREE_SPEED / (NOMINAL_VOLTAGE - OMEGA * FREE_CURRENT);// took from FRC examples.

        public static final double ALLOWED_ERROR = 0.03;// [m]
        public static final double UP_MOTOR_J = 0.00055;
        public static final double ARBITRARY_FEED_FORWARD_UP = 0.2;
    }

    public static final class Hood {
        public static final WebConstant KP = new WebConstant("HOOD_KP", 4.5);
        public static final WebConstant KI = new WebConstant("HOOD_KI", 0.01);
        public static final WebConstant KD = new WebConstant("HOOD_KD", 0);
        public static final WebConstant KF = new WebConstant("HOOD_KF", 0.8);
        public static final double ACCELERATION = 2; // [m/sec^2]
        public static final double CRUISE_VELOCITY = 1; // [m/sec]
        public static final int POSITION_TOLERANCE = 5;
        public static final WebConstant ARBITRARY_KF = new WebConstant("ARBITRARY_KF", 4);
        public static final int MIN_POSITION = 0;
        public static final WebConstant POSITION = new WebConstant("Hood_position", MIN_POSITION);
        public static final int MAX_POSITION = MIN_POSITION + (82 + 5552);
        public static final int STUCK_POSITION = MIN_POSITION + (5552 - 932);
    }

    public static final class Intake {
        public static final WebConstant POWER = new WebConstant("INTAKE_POWER", 1); // the target power of intake's motor(%)
    }

    public static final class Funnel {
        public static final double POWER = 0.5; //the default output of Funnel's motor (%)
        public static final double POWER_SLOW = 0.4;
    }

    // TODO: Change the values
    public static final class Conveyor {
        public static final double FORWARD_PEAK = 1; // [%]
        public static final double REVERSE_PEAK = -1; // [%]
        public static final SupplyCurrentLimitConfiguration SUPPLY_CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(false, 40, 2, 0); //prevent breakers from tripping.
        public static final StatorCurrentLimitConfiguration STATOR_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(false, 40, 0, 0); // control the acceleration

        public static final int INITIAL_BALLS_AMOUNT = 3;
        public static final int MAX_BALLS_AMOUNT = 5;

        public static final double SHOOTER_PROXIMITY_LOST_VOLTAGE = 0; // [volts]
        public static final double SHOOTER_PROXIMITY_SENSE_VOLTAGE = 0; // [volts]
        public static final double FUNNEL_PROXIMITY_LOST_VOLTAGE = 0; // [volts]
        public static final double FUNNEL_PROXIMITY_SENSE_VOLTAGE = 0; // [volts]
        //        public static final double[] YELLOW_RGB = {0.317, 0.552, 0.127};
        public static final double[] YELLOW_RGB = {0.35, 0.55, 0.09};
        public static final double[] GREEN_RGB = {0.16, 0.571, 0.269};
        public static final double[] RED_RGB = {0.492, 0.348, 0.145};
        public static final double[] BLUE_RGB = {0.132, 0.427, 0.442};
        public static final double CONVEYOR_MOTOR_RETURN_POWER = 0; // [%]
        public static final double CONVEYOR_MOTOR_POWER = 0.9;
        public static final double CONVEYOR_MOTOR_POWER_SLOW = 0.3;
        public static final double CONVEYOR_MOTOR_POWER_LOAD = 0.6;
    }

    public static final class Vision {
        public static final double VISION_MODULE_HOOD_DISTANCE = 0;
        public static final double VISION_ROTATION_RADIUS = 0.2646;
        public static final double ROBOT_TO_TURRET_CENTER = 0.3079;
        public static final double HEIGHT = 0.66;
        public static final double TARGET_HEIGHT = 2.5;
        public static final double VISION_TO_CENTER = 0;

        public static final Pose2d RED_INNER_POWER_PORT_LOCATION = new Pose2d();
        public static final Pose2d RED_OUTER_POWER_PORT_LOCATION = new Pose2d();

        public static final int LOW_ANGLE = 34;
        public static final int HIGH_ANGLE = 56;
    }

    public static class SwerveDrive {

        public static final double THIS_SHOULD_BE_1_ASK_AMIR = 1;
        public static final double TICKS_PER_METER = 2048 / (4 * 0.0254 * Math.PI) * 7.5 * THIS_SHOULD_BE_1_ASK_AMIR;
        public static final int TICKS_IN_ENCODER = 1024;
        public static final int ALLOWABLE_ANGLE_ERROR = 30; // [ticks]
        public static final double ANGLE_GEAR_RATIO = 1;
        public static final double TICKS_PER_RAD = TICKS_IN_ENCODER / (2 * Math.PI) * ANGLE_GEAR_RATIO;

        public static final int MAX_CURRENT = 35; // in ampere

        public static final double ROBOT_LENGTH = 0.58; // in meters
        public static final double ROBOT_WIDTH = 0.58; // in meters

        // the speed of the robot, this constant multiplies the speed outputs from the joysticks
        public static final double SPEED_MULTIPLIER = 2 / Math.sqrt(2);

        // the rotational speed of the robot, this constant multiplies the rotation output of the joystick
        public static final double ROTATION_MULTIPLIER = 2 * Math.PI;

        public static final double JOYSTICK_THRESHOLD = 0.05;

        public static final double TURN_TOLERANCE = 2; // degrees

        public static final double KP_TURN = 0.05;
        public static final WebConstant KI_TURN = new WebConstant("KI_TURN", 0.02);
        public static final double KD_TURN = 0;

        public static final WebConstant KP_MOVE = new WebConstant("KP_MOVE", 0);
        public static final WebConstant KI_MOVE = new WebConstant("KI_MOVE", 0);
        public static final WebConstant KD_MOVE = new WebConstant("KD_MOVE", 0);

        public static final double DRIVE_SETPOINT = 0.965; // [m]
        public static final double DRIVE_TOLERANCE = 0.05; // [m]

        public static final double STALL_CURRENT = 134; // [amps]
        public static final double STALL_TORQUE = 0.71; // [N*meters]

        public static final double FREE_CURRENT = 0.7; // [amps]
        public static final double FREE_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(18730); // [rad/sec]

        public static final double kT_775PRO = STALL_TORQUE / STALL_CURRENT;// took from FRC examples.
        public static final double OMEGA_775PRO = NOMINAL_VOLTAGE / STALL_CURRENT; // [Ohm]
        public static final double kV_775PRO = FREE_SPEED / (NOMINAL_VOLTAGE - OMEGA_775PRO * FREE_CURRENT);// took from FRC examples.
        public static final double RADIUS = 0.05; // [m]
        // TODO: choose real values.
        public static final WebConstant MASS = new WebConstant("mass", 4.5); // [kg]
        public static final WebConstant kPERCENT = new WebConstant("kPercentAngle", 1); // [kg]

    }

    public static class SwerveModule {

        public static final double STD_DEVS_STATES = Math.sqrt(10e-6);
        public static final double STD_DEVS_OUTPUTS = Math.sqrt(10e-6);

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

        // sick man
        // the module that has more friction in the rotating mechanism
        public static final WebConstant KP_SICK = new WebConstant("KP_SICK", 6);
        public static final WebConstant KI_SICK = new WebConstant("KI_SICK", 0);
        public static final WebConstant KD_SICK = new WebConstant("KD_SICK", 8);
        public static final WebConstant KF_SICK = new WebConstant("KF_SICK", 0);
        public static final WebConstant[] SICK_ANGLE_PIDF = new WebConstant[]{KP_SICK, KI_SICK, KD_SICK, KF_SICK};


        public static final WebConstant KP_ANGLE_FR = new WebConstant("KP_FR", 0.193);
        public static final WebConstant KI_ANGLE_FR = new WebConstant("KI_FR", 0.0);
        public static final WebConstant KD_ANGLE_FR = new WebConstant("KD_FR", 0.0);
        public static final WebConstant KF_ANGLE_FR = new WebConstant("KF_FR", 0);
        public static final WebConstant[] PIDF_ANGLE_FR = new WebConstant[]{KP_ANGLE_FR, KI_ANGLE_FR, KD_ANGLE_FR, KF_ANGLE_FR};

        public static final WebConstant KP_ANGLE_FL = new WebConstant("KP_FL", 0.21);
        public static final WebConstant KI_ANGLE_FL = new WebConstant("KI_FL", 0.0);
        public static final WebConstant KD_ANGLE_FL = new WebConstant("KD_FL", 0.0);
        public static final WebConstant KF_ANGLE_FL = new WebConstant("KF_FL", 0);
        public static final WebConstant[] PIDF_ANGLE_FL = new WebConstant[]{KP_ANGLE_FL, KI_ANGLE_FL, KD_ANGLE_FL, KF_ANGLE_FL};

        public static final WebConstant KP_ANGLE_RR = new WebConstant("KP_RR", 0.237);
        public static final WebConstant KI_ANGLE_RR = new WebConstant("KI_RR", 0);
        public static final WebConstant KD_ANGLE_RR = new WebConstant("KD_RR", 0);
        public static final WebConstant KF_ANGLE_RR = new WebConstant("KF_RR", 0);
        public static final WebConstant[] PIDF_ANGLE_RR = new WebConstant[]{KP_ANGLE_RR, KI_ANGLE_RR, KD_ANGLE_RR, KF_ANGLE_RR};

        public static final WebConstant KP_ANGLE_RL = new WebConstant("KP_RL", 0.237);
        public static final WebConstant KI_ANGLE_RL = new WebConstant("KI_RL", 0);
        public static final WebConstant KD_ANGLE_RL = new WebConstant("KD_RL", 0);
        public static final WebConstant KF_ANGLE_RL = new WebConstant("KF_RL", 0);
        public static final WebConstant[] PIDF_ANGLE_RL = new WebConstant[]{KP_ANGLE_RL, KI_ANGLE_RL, KD_ANGLE_RL, KF_ANGLE_RL};

        public static final int[] ZERO_POSITION = {1000, 816, 268, 270};

        public static final int TRIGGER_THRESHOLD_CURRENT = 5; // ampere
        public static final double TRIGGER_THRESHOLD_TIME = 0.02; // seconds
        public static final boolean ENABLE_VOLTAGE_COMPENSATION = true;
        public static final int VOLTAGE_SATURATION = 12; // volts

        public static final WebConstant J = new WebConstant("J_Swerve", 0.105);
        public static final WebConstant J_ANGLE = new WebConstant("J_Angle", 0.0005255999 * 4);

        public static final double GEAR_RATIO_MOTOR = 1 / 43.17;

        public static final double STALL_CURRENT = 134; // [amps]
        public static final double STALL_TORQUE = 0.71; // [N*meters]

        public static final double FREE_CURRENT = 0.7; // [amps]
        public static final double FREE_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(18730); // [rad/sec]

        public static final double kT = STALL_TORQUE / STALL_CURRENT;// took from FRC examples.
        public static final double OMEGA = NOMINAL_VOLTAGE / STALL_CURRENT; // [Ohm]
        public static final double kV = FREE_SPEED / (NOMINAL_VOLTAGE - OMEGA * FREE_CURRENT);// took from FRC examples.
    }

    public static class Autonomous {
        public static final double MAX_ACCELERATION = 1;
        public static final double MAX_VELOCITY = 3;
        public static final double kPThetaController = 2;
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints();
        public static final double kPXController = 1.2;
        public static final double kPYController = 1.2;
        public static final double MAX_CENTRIPETAL_ACCELERATION = 1.2;

        public static final String TEST_PATH = "paths/Test.wpilib.json";
        public static final String INITIATION_TO_TRENCH_PATH = "paths/InitiationToTrench.wpilib.json";
        public static final String SHOOT_TO_SAFE_TRENCH_PATH = "paths/ShootToSafeTrench.wpilib.json";
        public static final String TRENCH_TO_SHOOT_PATH = "paths/TrenchToShoot.wpilib.json";
        public static final String INITIATION_TO_SAFE_TRENCH_PATH = "paths/InitiationToSafeTrench.wpilib.json";
        public static final String SAFE_TRENCH_TO_SHOOT_PATH = "paths/SafeTrenchToShoot.wpilib.json";
        public static final String FORWARD_PATH = "paths/Forward.wpilib.json";
        public static final String FORWARD_Y_PATH = "paths/ForwardY.wpilib.json";
        public static final String LEFT_TURN_PATH = "paths/LeftTurn.wpilib.json";
        public static final String TEST_ITST_PATH = "paths/TestITST.wpilib.json";
        public static final String INITIATION_TO_SHOOT_PATH = "paths/InitiationToShoot.wpilib.json";
        public static final String INITIATION_TO_FIVE_PATH = "paths/InitiationToFive.wpilib.json";
        public static final String FIVE_PATH = "paths/Five.wpilib.json";
        public static final String FIVE_TO_NINE_PATH = "paths/FiveToNine.wpilib.json";
        public static final String NINE_PATH = "paths/Nine.wpilib.json";
        public static final String NINE_TO_EIGHT_PATH = "paths/NineToEight.wpilib.json";
        public static final String EIGHT_PATH = "paths/Eight.wpilib.json";
        public static final String EIGHT_TO_SEVEN_PATH = "paths/EightToSeven.wpilib.json";
        public static final String SEVEN_PATH = "paths/Seven.wpilib.json";
        public static final String PICKUP_BALLS_PATH = "paths/ForwardPickup.wpilib.json";
        public static final String PICKUP_TO_INITIATION_PATH = "paths/PickupToInitiation.wpilib.json";
    }
}
