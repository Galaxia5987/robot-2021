package frc.robot.utils;


import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import org.photonvision.LEDMode;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import javax.annotation.Nullable;

// todo:   camera.hasTargets()
public class VisionModule extends SubsystemBase {

    public static final PhotonCamera camera = new PhotonCamera("GalaxiaCam");
    private static final LinearFilter filter = LinearFilter.movingAverage(10);
    private static double filteredDistance = -1;

    /**
     * @return the angle to the target from the vision network table.
     */
    @Nullable
    public static Double getVisionAngle() {
        if (!targetSeen()) return null;
        return camera.getLatestResult().getBestTarget().getPitch(); // TODO: check
    }

    /**
     * @return whether the vision sees the target
     */
    public static boolean targetSeen() {
        return camera.hasTargets();
    }

    public static void setLEDs(LEDMode ledMode) {
        camera.setLED(ledMode);
    }

    @Nullable
    public static Pose2d getPose(double cameraPitch) {
        if (!targetSeen() || camera.getLatestResult() == null || camera.getLatestResult().getBestTarget() == null) return null;
        return PhotonUtils.estimateFieldToRobot(Constants.Vision.HEIGHT, Constants.Vision.TARGET_HEIGHT,
                cameraPitch, camera.getLatestResult().getBestTarget().getPitch(),
                new Rotation2d(), new Rotation2d(), new Pose2d(), new Transform2d()); //TODO: use real rotations
    }

    /**
     * @return The distance the camera sees, in meters.
     */
    public static double getTargetRawDistance(double cameraPitch) {
        if (!camera.hasTargets())
            return -1;

        SmartDashboard.putNumber("target-pitch", Math.toRadians(camera.getLatestResult().getBestTarget().getPitch()));

        return PhotonUtils.calculateDistanceToTargetMeters(Constants.Vision.HEIGHT, Constants.Vision.TARGET_HEIGHT,
                cameraPitch, Math.toRadians(camera.getLatestResult().getBestTarget().getPitch()));
    }

    public static double getHoodDistance() {
        if (filteredDistance == -1) return -1;
        return filteredDistance + Constants.Vision.VISION_MODULE_HOOD_DISTANCE;
    }

    public static double getRobotDistance() {
        if (filteredDistance == -1) return -1;
        double a = Constants.Vision.VISION_ROTATION_RADIUS + filteredDistance;
        double b = Constants.Vision.VISION_TO_CENTER;
        return Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2) - 2 * a * b * Math.cos(Math.toRadians(-Robot.navx.getAngle()))); //Cosine law
    }

    @Nullable
    public static Pose2d getRobotPose() {
        Pose2d visionPose = getPose(Math.toRadians(34));

        double robotDistance = getRobotDistance();
        if (visionPose == null || robotDistance == -1) return null;
        return new Pose2d(
                UtilityFunctions.getPortLocation(false).getTranslation().getX() - visionPose.getRotation().getCos() * robotDistance,
                UtilityFunctions.getPortLocation(false).getTranslation().getY() + visionPose.getRotation().getSin() * robotDistance,
                Rotation2d.fromDegrees((-Robot.navx.getAngle()) - visionPose.getRotation().getDegrees())
        );
    }

    private static double calculateMovingAverage(double distance) {
        return filter.calculate(distance);
    }

    @Override
    public void periodic() {
        double distance = getTargetRawDistance(Math.toRadians(34));
        if (distance == -1) {
            filteredDistance = -1;
        } else {
            if (distance >= 0.1) {
                filteredDistance = calculateMovingAverage(distance);
                SmartDashboard.putNumber("FilteredDistance", filteredDistance);
            }
            SmartDashboard.putNumber("VisionDistance", distance);
        }
	}
}
