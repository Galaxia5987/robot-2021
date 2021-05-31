package frc.robot.utils;


import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import org.opencv.core.Mat;
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
        if (!targetSeen() || camera.getLatestResult() == null || camera.getLatestResult().getBestTarget() == null)
            return null;
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
        if (!targetSeen() || camera.getLatestResult() == null || camera.getLatestResult().getBestTarget() == null)
            return null;
        var target = camera.getLatestResult().getBestTarget();
        SmartDashboard.putNumber("gyro-angle", Robot.navx.getYaw());
        return PhotonUtils.estimateFieldToRobot(Constants.Vision.HEIGHT, Constants.Vision.TARGET_HEIGHT,
                cameraPitch, camera.getLatestResult().getBestTarget().getPitch(),
                new Rotation2d(Math.toRadians(target.getYaw())), new Rotation2d(Robot.navx.getYaw()), new Pose2d(new Translation2d(0, 0), new Rotation2d(0)), new Transform2d()); //TODO: use real rotations
//        var res = camera.getLatestResult();
//        if (res.hasTargets()) {
//            double imageCaptureTime = Timer.getFPGATimestamp() - res.getLatencyMillis();
//            Transform2d camToTargetTrans = res.getBestTarget().getCameraToTarget();
//            Pose2d camPose = Constants.kFarTargetPose.transformBy(camToTargetTrans.inverse());
//            m_poseEstimator.addVisionMeasurement(
//                    camPose.transformBy(Constants.kCameraToRobot), imageCaptureTime);
//        }
    }

    /**
     * @param cameraPitch the angle of the camera.
     * @return The distance the camera sees, in meters.
     */
    public static double getTargetRawDistance(double cameraPitch) {

        if (!targetSeen() || camera.getLatestResult() == null)
            return -1;
        var target = camera.getLatestResult().getBestTarget();
        if (target == null || Double.isNaN(target.getPitch()))
            return -1;
        System.out.println("Latest Result: " + camera.getLatestResult());
        System.out.println("Best Target: " + target);
        System.out.println("Pitch: " + target.getPitch());
        double targetPitch = Math.toRadians(target.getPitch());
        SmartDashboard.putNumber("target-pitch", targetPitch);

        if (Math.tan(cameraPitch + targetPitch) == 0) {
            return -1;
        }

        return PhotonUtils.calculateDistanceToTargetMeters(Constants.Vision.HEIGHT, Constants.Vision.TARGET_HEIGHT,
                cameraPitch, targetPitch);
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
        var pose = getPose(Math.toRadians(34));
        if (pose != null && pose.getTranslation() != null) {
            var trans = pose.getTranslation();
            SmartDashboard.putNumberArray("pose", new double[]{trans.getX(), trans.getY()});
        } else {
            SmartDashboard.putNumberArray("pose", new double[]{0,0});
        }
        double distance = getTargetRawDistance(Math.toRadians(34));
        if (distance == -1) {
            filteredDistance = -1;
        } else if (distance >= 0.1) {
            filteredDistance = calculateMovingAverage(distance);
            SmartDashboard.putNumber("FilteredDistance", filteredDistance);
        }
        SmartDashboard.putNumber("VisionDistance", distance);

    }
}
