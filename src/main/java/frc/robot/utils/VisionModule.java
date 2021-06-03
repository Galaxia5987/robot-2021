package frc.robot.utils;


import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Robot;
import org.photonvision.LEDMode;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import java.util.Optional;
import java.util.OptionalDouble;

public class VisionModule extends SubsystemBase {
    public final PhotonCamera camera = new PhotonCamera("GalaxiaCam");
    private final LinearFilter filter = LinearFilter.movingAverage(10);
    private final Solenoid piston = new Solenoid(Ports.Vision.SOLENOID);
    private double filteredDistance = -1;

    /**
     * @return the angle to the target from the vision network table.
     */
    public OptionalDouble getVisionAngle() {
        if (targetInvisible() || camera.getLatestResult() == null)
            return OptionalDouble.empty();
        var target = camera.getLatestResult().getBestTarget();
        if (target == null)
            return OptionalDouble.empty();
        return OptionalDouble.of(target.getPitch()); // TODO: check
    }

    public OptionalDouble getVisionYaw() {
        if (targetInvisible() || camera.getLatestResult() == null)
            return OptionalDouble.empty();
        var target = camera.getLatestResult().getBestTarget();
        if (target == null)
            return OptionalDouble.empty();
        return OptionalDouble.of(target.getYaw()); // TODO: check
    }

    /**
     * @return whether the vision sees the target
     */
    public boolean targetInvisible() {
        return !camera.hasTargets();
    }

    public void setLEDs(LEDMode ledMode) {
        camera.setLED(ledMode);
    }

    public void togglePiston() {
        // TODO: check whether the index is the tight index!!!!!!!.
        if (piston.get()) {
            piston.set(false);
            camera.setPipelineIndex(1);
        } else {
            piston.set(true);
            camera.setPipelineIndex(0);
        }
    }

    public Optional<Pose2d> getPose() {
        if (targetInvisible() || camera.getLatestResult() == null || camera.getLatestResult().getBestTarget() == null)
            return Optional.empty();
        var target = camera.getLatestResult().getBestTarget();
        SmartDashboard.putNumber("gyro-angle", Robot.navx.getAngle());
        return Optional.of(
                PhotonUtils.estimateFieldToRobot(Constants.Vision.HEIGHT, Constants.Vision.TARGET_HEIGHT,
                        getCurrentPitch(), target.getPitch(), new Rotation2d(Math.toRadians(target.getYaw())), new Rotation2d(Robot.navx.getAngle()),
                        new Pose2d(new Translation2d(0, 0), new Rotation2d(0)), new Transform2d())); //TODO: use real rotations
    }

    /**
     * @return The distance the camera sees, in meters.
     */
    public OptionalDouble getTargetRawDistance() {
        if (targetInvisible() || camera.getLatestResult() == null)
            return OptionalDouble.empty();
        var target = camera.getLatestResult().getBestTarget();
        if (target == null)
            return OptionalDouble.empty();

        double targetPitch = Math.toRadians(target.getPitch());
        SmartDashboard.putNumber("target-pitch", target.getPitch());

        return OptionalDouble.of(
                PhotonUtils.calculateDistanceToTargetMeters(
                        Constants.Vision.HEIGHT,
                        Constants.Vision.TARGET_HEIGHT,
                        getCurrentPitch(),
                        targetPitch)
        );
    }

    public double getHoodDistance() {
        if (filteredDistance == -1) return -1;
        return filteredDistance + Constants.Vision.VISION_MODULE_HOOD_DISTANCE;
    }

    public double getRobotDistance() {
        if (filteredDistance == -1) return -1;
        double a = Constants.Vision.VISION_ROTATION_RADIUS + filteredDistance;
        double b = Constants.Vision.VISION_TO_CENTER;
        return Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2) - 2 * a * b * Math.cos(Math.toRadians(-Robot.navx.getAngle()))); //Cosine law
    }


    public Optional<Pose2d> getRobotPose() {
        Optional<Pose2d> visionPose = getPose();
        double robotDistance = getRobotDistance();
        if (visionPose.isEmpty() || robotDistance == -1) return Optional.empty();
        var pose = visionPose.get();
        return Optional.of(new Pose2d(
                        UtilityFunctions.getPortLocation(false).getTranslation().getX() - pose.getRotation().getCos() * robotDistance,
                        UtilityFunctions.getPortLocation(false).getTranslation().getY() + pose.getRotation().getSin() * robotDistance,
                        Rotation2d.fromDegrees((-Robot.navx.getAngle()) - pose.getRotation().getDegrees())
                )
        );
    }

    private double calculateMovingAverage(double distance) {
        return filter.calculate(distance);
    }

    public double getCurrentPitch() {
        return Math.toRadians(piston.get() ? Constants.Vision.LOW_ANGLE : Constants.Vision.HIGH_ANGLE);
    }

    @Override
    public void periodic() {
        var distance = getTargetRawDistance();
        if (distance.isEmpty()) {
            filteredDistance = -1;
        } else if (distance.getAsDouble() >= 0.1) {
            filteredDistance = calculateMovingAverage(distance.getAsDouble());
            SmartDashboard.putNumber("FilteredDistance", filteredDistance);

            if (targetInvisible()) return;
            var target = camera.getLatestResult().getBestTarget();
            if (target != null) {
                var t = PhotonUtils.estimateCameraToTargetTranslation(distance.getAsDouble(), Rotation2d.fromDegrees(-target.getYaw()));
                SmartDashboard.putNumberArray("pose2", new double[]{t.getX(), t.getY()});
            }

        }
        SmartDashboard.putNumber("VisionDistance", distance.orElse(-1));
    }
}
