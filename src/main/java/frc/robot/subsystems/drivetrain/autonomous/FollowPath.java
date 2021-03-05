package frc.robot.subsystems.drivetrain.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.utils.auto.Path;
import org.ghrobotics.lib.debug.FalconDashboard;

/**
 * This command handles trajectory-following.
 * A modified fork of {@link edu.wpi.first.wpilibj2.command.RamseteCommand}
 */
public class FollowPath extends CommandBase {

    private final Timer timer = new Timer();
    private boolean resetDrivetrain = false;
    private Path path;
    private SwerveModuleState[] prevSpeeds;
    private double prevTime;

    private static final RamseteController follower = new RamseteController(Constants.Autonomous.kBeta, Constants.Autonomous.kZeta);
<<<<<<< Updated upstream
    private static final SimpleMotorFeedforward[] feedforward = new SimpleMotorFeedforward[4];
=======
    private static final SimpleMotorFeedforward leftfeedforward = new SimpleMotorFeedforward(Constants.Autonomous.leftkS, Constants.Autonomous.leftkV, Constants.Autonomous.leftkA);
    private static final SimpleMotorFeedforward rightfeedforward = new SimpleMotorFeedforward(Constants.Autonomous.rightkS, Constants.Autonomous.rightkV, Constants.Autonomous.rightkA);
>>>>>>> Stashed changes
    private final SwerveDrive swerveDrive;
    private Trajectory trajectory;

    public FollowPath(SwerveDrive swerveDrive, Trajectory trajectory, boolean resetDrivetrain) {
        addRequirements(swerveDrive);
        this.trajectory = trajectory;
        this.swerveDrive = swerveDrive;
        this.resetDrivetrain = resetDrivetrain;
        for (int i = 0; i < 4; i++) {
            feedforward[i] = new SimpleMotorFeedforward(
                    Constants.Autonomous.kS[i],
                    Constants.Autonomous.kV[i],
                    Constants.Autonomous.kA[i]
            );
        }
    }

    public FollowPath(SwerveDrive swerveDrive, Path path) {
        addRequirements(swerveDrive);
        this.path = path;
        this.swerveDrive = swerveDrive;
    }

    @Override
    public void initialize() {
        if (trajectory == null) {
            if (!path.hasTrajectory()) {
                path.generate(swerveDrive.getPose());
            }

            this.trajectory = path.getTrajectory();
        }

        if (resetDrivetrain)
            swerveDrive.setPose(trajectory.getInitialPose());

        FalconDashboard.INSTANCE.setFollowingPath(true);
        prevTime = 0;
        var initialState = trajectory.sample(0);

        prevSpeeds = SwerveDrive.kinematics.toSwerveModuleStates(
                new ChassisSpeeds(initialState.velocityMetersPerSecond, 0,
                        initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond)
        );

        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double curTime = timer.get();
        double dt = curTime - prevTime;

        Trajectory.State state = trajectory.sample(curTime);

        var targetWheelSpeeds = SwerveDrive.kinematics.toSwerveModuleStates(
                follower.calculate(swerveDrive.getPose(), state)
        );

        // feeds the corresponding control to each wheel
        for (int k = 0; k < 4; k++) {
            swerveDrive.getModule(k).setSpeed(
                    targetWheelSpeeds[k].speedMetersPerSecond +
                    feedforward[k].calculate(targetWheelSpeeds[k].speedMetersPerSecond,
                            (targetWheelSpeeds[k].speedMetersPerSecond - prevSpeeds[k].speedMetersPerSecond) / dt)
            );
            swerveDrive.getModule(k).setAngle(targetWheelSpeeds[k].angle.getRadians());
        }

        FalconDashboard.INSTANCE.setPathHeading(state.poseMeters.getRotation().getRadians());
        FalconDashboard.INSTANCE.setPathX(Units.metersToFeet(state.poseMeters.getTranslation().getX()));
        FalconDashboard.INSTANCE.setPathY(Units.metersToFeet(state.poseMeters.getTranslation().getY()));

        prevTime = curTime;
        prevSpeeds = targetWheelSpeeds;
    }

    @Override
    public void end(boolean interrupted) {
        FalconDashboard.INSTANCE.setFollowingPath(false);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasPeriodPassed(trajectory.getTotalTimeSeconds());
    }
}
