package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.Constants;
import frc.robot.Robot;
import org.techfire225.webapp.FireLog;

import java.util.function.DoubleSupplier;

import static frc.robot.Ports.SwerveDrive.*;

/**
 * The Swerve Subsystem calculates all the mathematical operations and controls for the Swerve Modules.
 */
public class SwerveDrive extends SubsystemBase {
    // calculates the distance from the center of the robot to the wheels
    private static final double Rx = Constants.SwerveDrive.ROBOT_WIDTH / 2;
    private static final double Ry = Constants.SwerveDrive.ROBOT_LENGTH / 2;
    // the sign vectors of Rx and Ry
//    private static final double[] signX = {1, 1, -1, -1};
//    private static final double[] signY = {-1, 1, -1, 1};
    private static final double[] signX = {1, 1, -1, -1};
    private static final double[] signY = {1, -1, 1, -1};

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
    private final SwerveModule[] swerveModules = new SwerveModule[4];
    private final TrajectoryConfig config = new TrajectoryConfig(Constants.Autonomous.MAX_VELOCITY,
            Constants.Autonomous.MAX_ACCELERATION)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(kinematics);
    private final DoubleSupplier angleSupplier = Robot.navx::getYaw;

    public SwerveDrive(boolean isFieldOriented) {
        this(isFieldOriented, false);
    }

    public SwerveDrive(boolean isFieldOriented, boolean testMode) {

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
     * Sets the wheels of the robot to the calculated angle and speed.
     *
     * @param forward  the Y value of the joystick
     * @param strafe   the X value of the joystick
     * @param rotation the rotation Z of the joystick
     */
    public void holonomicDrive(double forward, double strafe, double rotation) {
        ChassisSpeeds speeds = isFieldOriented ?
                ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, Rotation2d.fromDegrees(angleSupplier.getAsDouble())) :
                new ChassisSpeeds(forward, strafe, rotation);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        setStates(states);
    }

    public void holonomicDriveExperimental(double forward, double strafe, double rotation, double[] filter) {
        ChassisSpeeds speeds = isFieldOriented ?
                ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, Rotation2d.fromDegrees(angleSupplier.getAsDouble())) :
                new ChassisSpeeds(forward, strafe, rotation);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        setStatesExperimental(states, filter);
    }

    public void setStates(SwerveModuleState[] states) {
        for (int i = 0; i < states.length; i++) {
            states[i] = SwerveModuleState.optimize(states[i], new Rotation2d(getModule(i).getAngle()));
            getModule(i).setState(states[i]);
        }
    }

    public void setStatesExperimental(SwerveModuleState[] states, double[] filter) {
        for (int i = 0; i < states.length; i++) {
            states[i] = SwerveModuleState.optimize(states[i], new Rotation2d(getModule(i).getAngle()));
            getModule(i).setStateExperimental(states[i], filter[i]);
        }
    }

    public void setModuleOutputMax(int module) {
        getModule(module).setSpeed(100000);
        getModule(module).setAngle(100000);
    }

    public void setAngles(double angle) {
        for (int i = 0; i < 4; i++) {
            getModule(i).setAngle(angle);
        }
    }

    public boolean hasReachedAngles(double angle){
        for (int i = 0; i < 4; i++) {
            getModule(i).hasReachedAngle(angle);
        }
        return true;
    }

    /**
     * Stops all the wheels.
     */
    public void terminate() {
        for (SwerveModule swerveModule : swerveModules) {
            swerveModule.setSpeed(0);
            swerveModule.stopAngleMotor();
        }

    }

    /**
     * @param i the index of the module
     * @return the corresponding module
     */
    public SwerveModule getModule(int i) {
        return swerveModules[i];
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
        SmartDashboard.putNumber("navx angle", angleSupplier.getAsDouble());
        if (Robot.navx.isConnected())
            FireLog.log("navx angle", angleSupplier.getAsDouble());
        else
            FireLog.log("navx angle", 1);

    }

    public double getForward() {
        return Math.cos(Math.toRadians(angleSupplier.getAsDouble())) * Robot.navx.getVelocityX() + Math.cos(Math.toRadians(angleSupplier.getAsDouble())) * Robot.navx.getVelocityY();
    }

    public double getStrafe() {
        return Math.sin(Math.toRadians(angleSupplier.getAsDouble())) * Robot.navx.getVelocityX() + Math.sin(Math.toRadians(angleSupplier.getAsDouble())) * Robot.navx.getVelocityY();
    }

    public double getRotation() {
        return Math.toRadians(Robot.navx.getRate());
    }

    public ChassisSpeeds getRealChassisSpeeds() {
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++)
            swerveModuleStates[i] = new SwerveModuleState(getModule(i).getSpeed(), new Rotation2d(getModule(i).getAngle()));
        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(swerveModuleStates);
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond, Rotation2d.fromDegrees(angleSupplier.getAsDouble()));
        return chassisSpeeds;
    }
}
