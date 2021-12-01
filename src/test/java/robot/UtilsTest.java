package robot;

import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Utils;
import frc.robot.imuglyandimproud.PathPlanner;
import frc.robot.imuglyandimproud.PathPlannerTrajectory;
import org.junit.Assert;
import org.junit.jupiter.api.Test;

public class UtilsTest {

    private double delta = 0.01;

    @Test
    public void floorMod() {
        double val = Utils.floorMod(-1.5 * Math.PI, Math.PI);
        double expected = Math.PI / 2;

        Assert.assertEquals(expected, val, delta);
    }

    @Test
    public void cartesianToPolar() {
        double[] polar = Utils.cartesianToPolar(1, 3);
        double[] expected = {Math.sqrt(10), Math.PI / 2.5};
        Assert.assertArrayEquals(expected, polar, delta);
    }

    @Test
    public void matrixVectorMult() {
        double[][] mat = {{0, 1, 0.5}, {1, 0, -0.5}, {0, 1, 0.5}, {1, 0, .5}, {0, 1, -0.5}, {1, 0, .5}, {0, 1, -0.5}, {1, 0, -0.5}};
        double[] v = {.7, 0, 0};
        double[] vec = Utils.matrixVectorMult(mat, v);
        double[] expected = {0, .7, 0, .7, 0, .7, 0, .7};

        Assert.assertArrayEquals(expected, vec, delta);

        double[][] m = {{3, 2, .5}, {1, 4, .25}, {6, 7, 2}, {2, 4.5, 5}};
        double[] V = {.5, 2, 3};
        double[] vector = Utils.matrixVectorMult(m, V);
        double[] exp = {7.0, 9.25, 23.0, 25.0};

        Assert.assertArrayEquals(exp, vector, delta);
    }

    @Test
    public void testFunction() {
        System.out.println("hello");
        // This will load the file "Example Path.path" and generate it with a max velocity of 8 m/s and a max acceleration of 5 m/s^2

        PathPlannerTrajectory examplePath = PathPlanner.loadPath("The Orbit", 8, 5);
        // To access PathPlanner specific information, such as holonomic rotation, the state must be cast to a PathPlannerState
        // Sample the state of the path at 1.2 seconds
        examplePath.getStates().forEach(e -> {
            System.out.println(((PathPlannerTrajectory.PathPlannerState) e));
        });
        System.out.println(examplePath.getEndState());
        PathPlannerTrajectory.PathPlannerState exampleState = (PathPlannerTrajectory.PathPlannerState) examplePath.sample(10);
        System.out.println(exampleState.holonomicRotation.getDegrees());
    }

    @Test
    public void testFunction2() {
        PathPlannerTrajectory examplePath = PathPlanner.loadPath("testing", 8, 5);
        // To access PathPlanner specific information, such as holonomic rotation, the state must be cast to a PathPlannerState
        // Sample the state of the path at 1.2 seconds
        examplePath.getStates().forEach(e -> {
//            System.out.println(((PathPlannerTrajectory.PathPlannerState) e));
        });

        HolonomicDriveController controller = new HolonomicDriveController(
                new PIDController(1.2, 0, 0),
                new PIDController(1.2, 0, 0),
                new ProfiledPIDController(2, 0, 0, new TrapezoidProfile.Constraints()) {{
                    enableContinuousInput(-Math.PI, Math.PI);
                }});
//        double desiredX = 0;
//        double desiredY = 1;
//        double desiredRotation = 0;
//        Trajectory.State desiredState = new Trajectory.State(0, 0, 0, new Pose2d(desiredX, desiredY, Rotation2d.fromDegrees(desiredRotation)), 0);
        PathPlannerTrajectory.PathPlannerState desiredState = examplePath.getEndState();
        Pose2d currentPose = examplePath.getInitialState().poseMeters;
        currentPose = new Pose2d(currentPose.getTranslation(), Utils.pathPlannerRotationToRealRotation(examplePath.getInitialState().holonomicRotation));
        System.out.println(currentPose + "\n" + desiredState);
        ChassisSpeeds targetChassisSpeeds = controller.calculate(currentPose, desiredState, desiredState.holonomicRotation);
//        Pose2d currentPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
//        ChassisSpeeds targetChassisSpeeds = controller.calculate(currentPose, desiredState, Rotation2d.fromDegrees(0));
        System.out.println(targetChassisSpeeds);
    }
}