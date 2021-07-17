package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Utils;
import org.techfire225.webapp.FireLog;

import static frc.robot.Ports.SwerveDrive.*;

/**
 * The Swerve Subsystem calculates all the mathematical operations and controls for the Swerve Modules.
 */
public class SwerveDrive extends SubsystemBase {

    private static final double[][] dynamics = new double[8][3];
    // calculates the distance from the center of the robot to the wheels
    private static final double Rx = Constants.SwerveDrive.ROBOT_WIDTH / 2;
    private static final double Ry = Constants.SwerveDrive.ROBOT_LENGTH / 2;
    // the sign vectors of Rx and Ry
    private static final double[] signX = {1, 1, -1, -1};
    private static final double[] signY = {-1, 1, -1, 1};

    private static boolean isFieldOriented;
    public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(signX[0] * Rx, signY[0] * Ry),
            new Translation2d(signX[1] * Rx, signY[1] * Ry),
            new Translation2d(signX[2] * Rx, signY[2] * Ry),
            new Translation2d(signX[3] * Rx, signY[3] * Ry)
    );
    private final Timer timer = new Timer();
    private final SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(
            new Rotation2d(Math.toRadians(-Robot.navx.getYaw())),
            new Pose2d(new Translation2d(), new Rotation2d(0)),
            kinematics,
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(Units.degreesToRadians(1000)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
    );

    //    private final SwerveDriveOdometry swerveOdometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(-Robot.startAngle), new Pose2d());
    private final SwerveModule[] swerveModules = new SwerveModule[4];
    private final TrajectoryConfig config = new TrajectoryConfig(Constants.Autonomous.MAX_VELOCITY,
            Constants.Autonomous.MAX_ACCELERATION)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(kinematics);

    public SwerveDrive(boolean isFieldOriented) {
        this(isFieldOriented, false);
    }

    public SwerveDrive(boolean isFieldOriented, boolean testMode) {
        createInverseMatrix();

        if (!testMode) {
            swerveModules[0] = new SwerveModule(0, DRIVE_MOTOR_FRONT_RIGHT, ANGLE_MOTOR_FRONT_RIGHT, FRONT_RIGHT_INVERTED,
                    Constants.SwerveModule.PIDF_ANGLE_FR);

            swerveModules[1] = new SwerveModule(1, DRIVE_MOTOR_FRONT_LEFT, ANGLE_MOTOR_FRONT_LEFT, FRONT_LEFT_INVERTED,
                    Constants.SwerveModule.PIDF_ANGLE_FL);

            swerveModules[2] = new SwerveModule(2, DRIVE_MOTOR_BACK_RIGHT, ANGLE_MOTOR_BACK_RIGHT, BACK_RIGHT_INVERTED,
                    Constants.SwerveModule.PIDF_ANGLE_RR);

            swerveModules[3] = new SwerveModule(3, DRIVE_MOTOR_BACK_LEFT, ANGLE_MOTOR_BACK_LEFT, BACK_LEFT_INVERTED,
                    Constants.SwerveModule.PIDF_ANGLE_RL);
        }

        SwerveDrive.isFieldOriented = isFieldOriented;

        odometry.resetPosition(convertTrajectoryToOdometry(new Pose2d(3.159, 5.86, new Rotation2d())), new Rotation2d(Math.toRadians(-Robot.navx.getYaw())));
        timer.reset();
        timer.start();
    }

    /**
     * Turns the joystick inputs into the robot heading.
     *
     * @param forward    the Y value of the joystick
     * @param strafe     the X value of the joystick
     * @param rotation   the rotation Z of the joystick
     * @param robotAngle the current angle of the robot in radians
     * @return an array of the robot heading
     */
    public static double[] getRobotHeading(double forward, double strafe, double rotation, double robotAngle) {
        // multiplies the 2D rotation matrix by the robot heading, there by rotating the coordinate system
        // see https://en.wikipedia.org/wiki/Rotation_matrix
        SmartDashboard.putNumber("final angle", robotAngle);
        double[][] rotationMat = {{Math.cos(robotAngle), -Math.sin(robotAngle)},
                {Math.sin(robotAngle), Math.cos(robotAngle)}};
        double[] speeds = Utils.matrixVectorMult(rotationMat, new double[]{forward, strafe});
        // if the drive style is field oriented, changes the forward and strafe to be according to the field coordinate system
        if (isFieldOriented) {
            forward = speeds[0];
            strafe = speeds[1];
        }
        return new double[]{forward, strafe, rotation};
    }

    /**
     * Calculates the velocity vector of each wheel.
     *
     * @param robotHeading the three joystick outputs:
     *                     forward the heading of the robot in the Y direction
     *                     strafe the heading of the robot in the X direction
     *                     rotation the rotation of the robot
     * @return an array of length 8 in which each pair is the X and Y velocities of each wheel
     */
    public static double[] calculateWheelVelocities(double[] robotHeading) {
        // multiplies M by the robotHeading to obtain the wheel velocities
        return Utils.matrixVectorMult(dynamics, robotHeading);
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }


    /**
     * Sets the wheels of the robot to the calculated angle and speed.
     *
     * @param forward  the Y value of the joystick
     * @param strafe   the X value of the joystick
     * @param rotation the rotation Z of the joystick
     */
    public void holonomicDrive(double forward, double strafe, double rotation) {
        // -Math.toRadians(180 - Robot.navx.getYaw())
        double[] robotHeading = getRobotHeading(strafe, forward, rotation, -Math.toRadians(180 - (Robot.navx.getYaw() - Robot.startAngle)));

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
            swerveModules[k].setState(new SwerveModuleState(controls[k][0], new Rotation2d(controls[k][1])));
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
     * Set the angle of the wheels on the robot to lock the robot in place.
     * Lock angles are used for when there is defence on the robot,
     * the wheels point outwards and it is nearly impossible to move the robot.
     */
    public void lock() {
        double[] lockAngles = calculateLockAngles();

        for (int i = 0; i < 4; i++) {
            swerveModules[i].setSpeed(0);
            // switches the lock angles between the back left and back right wheels
            // so that they will match the signs of the dynamics matrix
            if (i == 2) {
                swerveModules[i].setAngle(lockAngles[i + 1]);
            }
            if (i == 3) {
                swerveModules[i].setAngle(lockAngles[i - 1]);
            }
        }
    }

    /**
     * Calculates the angles for which the wheels will lock in place.
     *
     * @return an array.
     */
    public double[] calculateLockAngles() {
        double[] lockAngles = new double[4];
        double firstLockAngle = Math.atan(Constants.SwerveDrive.ROBOT_LENGTH / Constants.SwerveDrive.ROBOT_WIDTH);

        for (int i = 0; i < 4; i++) {
            lockAngles[i] = firstLockAngle + i * Math.PI / 2;
        }

        return lockAngles;
    }

    /**
     * Stops all the wheels.
     */
    public void stop() {
        for (SwerveModule swerveModule : swerveModules) {

            swerveModule.setSpeed(0);
            swerveModule.stopAngleMotor();
        }

    }

    /**
     * @return the x and y velocities of each module
     */
    public double[][] getXYVelocities() {
        double[][] velocities = new double[4][2];
        for (int i = 0; i < 4; i++) {
            double[] cart = Utils.polarToCartesian(swerveModules[i].getSpeed(), swerveModules[i].getAngle());
            velocities[i][0] = cart[0];
            velocities[i][1] = cart[1];
        }

        return velocities;
    }

    /**
     * @return the velocity of the robot (length, angle)
     */
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

    /**
     * Resets all the module encoder values to 0.
     */
    public void resetAllEncoders() {
        for (SwerveModule swerveModule : swerveModules) {
            swerveModule.resetAngleEncoder();
        }
    }

    /**
     * @param i the index of the module
     * @return the corresponding module
     */
    public SwerveModule getModule(int i) {
        return swerveModules[i];
    }

    /**
     * Creates an inverse matrix of all the mathematical operations needed to calculate the wheel velocities.
     * see https://file.tavsys.net/control/controls-engineering-in-frc.pdf pg.140
     */
    public void createInverseMatrix() {

        for (int i = 0; i < 8; i++) {
            if (i % 2 == 0) {
                dynamics[i][0] = 1;
                dynamics[i][1] = 0;
                dynamics[i][2] = Rx * signX[i / 2];
            } else {
                dynamics[i][0] = 0;
                dynamics[i][1] = 1;
                dynamics[i][2] = Ry * signY[i / 2];
            }
        }
    }

    /**
     * Locks all modules in their current position.
     */
    public void lockModulePositions() {
        for (SwerveModule swerveModule : swerveModules) {
            swerveModule.setSpeed(0);
        }
        stayAtAngle();
    }

    /**
     * Makes the modules stay at its current angle.
     */
    public void stayAtAngle() {
        for (SwerveModule swerveModule : swerveModules) {
            swerveModule.setAngle(swerveModule.getAngle());
        }
    }

    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    public Pose2d getPoseForTrajectory() {
        Pose2d pose = odometry.getEstimatedPosition();
        return convertOdometryToTrajectory(pose);
    }

    public Pose2d convertOdometryToTrajectory(Pose2d pose) {
        return new Pose2d(pose.getY(), -pose.getX(), pose.getRotation());
    }

    public Pose2d convertTrajectoryToOdometry(Pose2d pose) {
        return new Pose2d(-pose.getY(), pose.getX(), pose.getRotation());
    }


    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(pose, new Rotation2d(Robot.navx.getYaw()));
    }

    public TrajectoryConfig getConfig() {
        return config;
    }

    public double getOptimalAngle(double initAngle, double targetAngle) {
        double option1 = targetAngle - initAngle;
        double clockwise, counterClockwise;
        if (option1 > 0) {
            clockwise = option1;
            counterClockwise = 360 - option1;
        } else {
            clockwise = 360 - Math.abs(option1);
            counterClockwise = Math.abs(option1);
        }
        if (clockwise < counterClockwise) {
            return clockwise;
        }
        return -counterClockwise;
    }

    public double getJoystickAngle() {
        {
            double x = RobotContainer.XboxDriver.getRawAxis(XboxController.Axis.kRightX.value);
            double y = RobotContainer.XboxDriver.getRawAxis(XboxController.Axis.kRightY.value);
            return Math.toDegrees(Math.atan2(y, x));
        }
    }

    public void putThisSomewhereAdam(double initAngle) {
        double rotateBy = getOptimalAngle(initAngle, getJoystickAngle());
    }


    @Override
    public void periodic() {
        FireLog.log("module FR", Math.toDegrees(getModule(0).getAngle()));
        FireLog.log("module FL", Math.toDegrees(getModule(1).getAngle()));
        FireLog.log("module RR", Math.toDegrees(getModule(2).getAngle()));
        FireLog.log("module RL", Math.toDegrees(getModule(3).getAngle()));

        SwerveModuleState[] swerveModuleState = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            swerveModuleState[i] = new SwerveModuleState(swerveModules[i].getSpeed(), new Rotation2d(Math.toRadians(90) - swerveModules[i].getAngle()));
            SmartDashboard.putNumber("angle " + i, swerveModuleState[i].angle.getDegrees());
            SmartDashboard.putNumber("vel" + i, swerveModuleState[i].speedMetersPerSecond);
            SmartDashboard.putNumber("correct angle " + i, swerveModules[i].getAngle());

        }

        odometry.updateWithTime(timer.get(),
                new Rotation2d(Math.toRadians(-Robot.navx.getYaw())),
                swerveModuleState
        );

        Pose2d pose = getPose();
        SmartDashboard.putNumber("x-pose", pose.getX());
        SmartDashboard.putNumber("y-pose", pose.getY());
        SmartDashboard.putNumber("theta-pose", pose.getRotation().getDegrees());
        SmartDashboard.putNumber("navx angle", Robot.navx.getYaw());
    }


}
