package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Utils;
import org.techfire225.webapp.FireLog;

import static frc.robot.Ports.SwerveDrive.*;

public class SwerveDrive extends SubsystemBase {

    static double[][] dynamics = new double[8][3];
    public SwerveModule[] swerveModules = new SwerveModule[4];
    // calculates the distance from the center of the robot to the wheels
    static double Rx = Constants.SwerveDrive.ROBOT_WIDTH / 2;
    static double Ry = Constants.SwerveDrive.ROBOT_LENGTH / 2;

    // the sign vectors of Rx and Ry
    static double[] signX = {1, 1, -1, -1};
    static double[] signY = {-1, 1, -1, 1};

    private static boolean isFieldOriented;

    public SwerveDrive(boolean isFieldOriented, boolean testMode) {
        createInverseMatrix();

        if (!testMode) {
            swerveModules[0] = new SwerveModule(0, DRIVE_MOTOR_0, ANGLE_MOTOR_0, FRONT_RIGHT_INVERTED);
            swerveModules[1] = new SwerveModule(1, DRIVE_MOTOR_1, ANGLE_MOTOR_1, FRONT_LEFT_INVERTED);
            swerveModules[2] = new SwerveModule(2, DRIVE_MOTOR_2, ANGLE_MOTOR_2, BACK_RIGHT_INVERTED);
            swerveModules[3] = new SwerveModule(3, DRIVE_MOTOR_3, ANGLE_MOTOR_3, BACK_LEFT_INVERTED);
        }

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

        return new double[]{forward, strafe, rotation};
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
        return Utils.matrixVectorMult(dynamics, robotHeading);
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
     * calculates the angles for which the wheels will lock in place
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
     * stops all the wheels
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
     * resets all the module encoder values to 0
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
     * creates an inverse matrix of all the mathematical operations needed to calculate the wheel velocities
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
     * locks all modules in their current position.
     */
    public void lockModulesPositions() {
        for (SwerveModule swerveModule : swerveModules) {
            swerveModule.setSpeed(0);
            swerveModule.setAngle(swerveModule.getAngle());
        }
    }

}
