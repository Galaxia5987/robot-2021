package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Utils;
import org.techfire225.webapp.FireLog;

import static frc.robot.Ports.SwerveDrive.*;

public class SwerveDrive extends SubsystemBase {

    static double[][] Dynamics = new double[8][3];
    public SwerveModule[] swerveModules = new SwerveModule[4];
    // calculates the distance from the center of the robot to the wheels
    static double Rx = Constants.SwerveDrive.ROBOT_WIDTH / 2;
    static double Ry = Constants.SwerveDrive.ROBOT_LENGTH / 2;

    // the sign vectors of Rx and Ry
    static double[] signX = {1, 1, -1, -1};
    static double[] signY = {-1, 1, -1, 1};

    // creates an inverse matrix of all the mathematical operations needed to calculate the wheel velocities
    // see https://file.tavsys.net/control/controls-engineering-in-frc.pdf pg.144
    private static boolean isFieldOriented;

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(signX[0] * Rx, signY[0] * Ry),
            new Translation2d(signX[1] * Rx, signY[1] * Ry),
            new Translation2d(signX[2] * Rx, signY[2] * Ry),
            new Translation2d(signX[3] * Rx, signY[3] * Ry)
    );

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(Robot.gyro.getAngle()), new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(Robot.gyro.getAngle()))); // TODO: Check pose2d and angle might be ccw.

    public SwerveDrive(boolean isFieldOriented) {

        for (int i = 0; i < 8; i++) {
            if (i % 2 == 0) {
                Dynamics[i][0] = 1;
                Dynamics[i][1] = 0;
                Dynamics[i][2] = Rx * signX[i / 2];
            } else {
                Dynamics[i][0] = 0;
                Dynamics[i][1] = 1;
                Dynamics[i][2] = Ry * signY[i / 2];
            }
        }
        Robot.gyro.reset();

        swerveModules[0] = new SwerveModule(0, new TalonFX(DRIVE_MOTOR_1), new TalonSRX(ANGLE_MOTOR_1), FRONT_RIGHT_INVERTED);
        swerveModules[1] = new SwerveModule(1, new TalonFX(DRIVE_MOTOR_2), new TalonSRX(ANGLE_MOTOR_2), FRONT_LEFT_INVERTED);
        swerveModules[2] = new SwerveModule(2, new TalonFX(DRIVE_MOTOR_3), new TalonSRX(ANGLE_MOTOR_3), BACK_RIGHT_INVERTED);
        swerveModules[3] = new SwerveModule(3, new TalonFX(DRIVE_MOTOR_4), new TalonSRX(ANGLE_MOTOR_4), BACK_LEFT_INVERTED);

        this.isFieldOriented = isFieldOriented;
    }


    /**
     * sets the wheels of the robot to the calculated angle and speed
     *
     * @param forward  the Y value of the joystick
     * @param strafe   the X value of the joystick
     * @param rotation the rotation Z of the joystick
     */
    public void holonomicDrive(double forward, double strafe, double rotation) {

        double[] robotHeading = getRobotHeading(strafe, forward, rotation, -Math.toRadians(Robot.gyro.getAngle()));

        double[] velocities = calculateWheelVelocities(robotHeading);
        double[] polar;
        double[][] controls = new double[4][2];

        // converts the cartesian velocities to polar and transfers them to a control matrix
        for (int i = 0; i < 4; i++) {
            polar = Utils.cartesianToPolar(velocities[2 * i + 1], velocities[2 * i]);
            controls[i][0] = polar[0];
            controls[i][1] = polar[1];
        }

        // feeds the corresponding control to each wheel
        for (int k = 0; k < 4; k++) {
            swerveModules[k].setSpeed(controls[k][0]);
            swerveModules[k].setAngle(controls[k][1]);
        }

        double sumx = 0;
        double sumy = 0;
        for (int j = 0; j < 4; j++) {
            sumx += velocities[j * 2];
            sumy += velocities[j * 2 + 1];
        }
        double[] target = Utils.cartesianToPolar(sumx, sumy);
        FireLog.log("target velocity", target[0]);
        FireLog.log("target angle", target[1]);
    }

    /**
     * turns the joystick inputs into the robot heading
     *
     * @param forward    the Y value of the joystick
     * @param strafe     the X value of the joystick
     * @param rotation   the rotation Z of the joystick
     * @param robotAngle the current angle of the robot in radians
     * @return an array of the robot heading
     */
    public static double[] getRobotHeading(double forward, double strafe, double rotation, double robotAngle) {
        // turns the joystick values into the heading of the robot
        forward *= Constants.SwerveDrive.SPEED_MULTIPLIER;
        strafe *= Constants.SwerveDrive.SPEED_MULTIPLIER;
        rotation *= Constants.SwerveDrive.ROTATION_MULTIPLIER;
        // multiplies the 2D rotation matrix by the robot heading, there by rotating the coordinate system
        // see https://en.wikipedia.org/wiki/Rotation_matrix
        double[][] rotationMat = {{Math.cos(robotAngle), -Math.sin(robotAngle)},
                {Math.sin(robotAngle), Math.cos(robotAngle)}};
        double[] speeds = Utils.matrixVectorMult(rotationMat, new double[]{forward, strafe});
        System.out.println("forward" + forward);
        System.out.println("strafe" + strafe);

        // if the drive style is field oriented, changes the forward and strafe to be according to the field coordinate system
        if (isFieldOriented) {
            forward = speeds[0];
            strafe = speeds[1];
        }
        System.out.println("relative forward" + speeds[0]);
        System.out.println("relative strafe" + speeds[1]);

        double[] robotHeading = {forward, strafe, rotation};

        return robotHeading;
    }

    /**
     * calculates the velocity vector of each wheel
     *
     * @param robotHeading the three joystick outputs:
     *                     forward the heading of the robot in the Y direction
     *                     strafe the heading of the robot in the X direction
     *                     rotation the rotation of the robot
     * @return an array of length 8 in which each pair is the X and Y velocities of each wheel
     */
    public static double[] calculateWheelVelocities(double[] robotHeading) {
        // multiplies M by the robotHeading to obtain the wheel velocities
        double[] wheelVelocities = Utils.matrixVectorMult(Dynamics, robotHeading);
        return wheelVelocities;
    }

    /**
     * set the angle of the wheels on the robot to lock the robot in place
     */
    public void lock() {
        // calculates the lock angles of the wheels
        double[] lockAngles = calculateLockAngles();

        for (int i = 0; i < 4; i++) {
            swerveModules[i].setSpeed(0);
            if (i == 2) {
                swerveModules[i].setAngle(lockAngles[i + 1]);
            }
            if (i == 3) {
                swerveModules[i].setAngle(lockAngles[i - 1]);
            }
        }
    }

    /**
     * calculates the angles for which the wheels will lock in place
     */
    public double[] calculateLockAngles() {
        double[] lockAngles = new double[4];

        for (int i = 0; i < 4; i++) {
            lockAngles[i] = Math.PI / 2 - Math.atan(Constants.SwerveDrive.ROBOT_WIDTH / Constants.SwerveDrive.ROBOT_LENGTH) + i * Math.PI / 2;
        }

        return lockAngles;
    }


    /**
     * stops all the wheels
     */
    public void stop() {
        for (SwerveModule swerveModule : swerveModules) {
            swerveModule.setSpeed(0);
            swerveModule.stopAngleMotor();
        }

    }

    public double[][] getXYVelocities() {
        double[][] velocities = new double[4][2];
        for (int i = 0; i < 4; i++) {
            double[] cart = Utils.polarToCartesian(swerveModules[i].getSpeed(), swerveModules[i].getAngle());
            velocities[i][0] = cart[0];
            velocities[i][1] = cart[1];
        }

        return velocities;
    }

    public double[] getVelocity() {
        double[][] velocities = getXYVelocities();
        double sumx = 0;
        double sumy = 0;
        for (int i = 0; i < 4; i++) {
            sumx += velocities[i][0];
            sumy += velocities[i][1];
        }
        return Utils.cartesianToPolar(sumx / 4, sumy / 4);
    }

    public void resetAll() {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].resetAngle();
        }
    }

    public void setPose(Pose2d pose) {
        odometry.resetPosition(pose, Rotation2d.fromDegrees(Robot.gyro.getAngle())); // TODO: change, check CCW
    }

    public Pose2d getPose() {
        Pose2d pose = odometry.getPoseMeters();
        double x = pose.getTranslation().getX();
        double y = pose.getTranslation().getY();
        double angle = pose.getRotation().getRadians();
        SmartDashboard.putNumber("x", x);
        SmartDashboard.putNumber("y", y);
        SmartDashboard.putNumber("angle", angle);
        return pose;
    }

}
