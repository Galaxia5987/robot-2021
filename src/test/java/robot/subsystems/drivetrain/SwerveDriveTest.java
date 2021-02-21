package robot.subsystems.drivetrain;

import frc.robot.subsystems.drivetrain.SwerveDrive;
import org.junit.*;

import java.util.Arrays;

import static frc.robot.Constants.SwerveDrive.*;

public class SwerveDriveTest {

    private SwerveDrive swerveDrive;
    private SwerveDrive swerveField;

    private double forward, strafe, rotation, gyro;
    private double delta = 0.01;
    private double[] expectedHeading, expectedHeadingField, expectedVel,
            expectedVelField, robotHeading, robotHeadingField;

    @Before
    public void setup() {
        swerveDrive = new SwerveDrive(false, true);
        swerveField = new SwerveDrive(true, true);
    }

    @Test
    public void turnInPlace() {
        forward = 0;
        strafe = 0;
        rotation = -1;

        expectedHeading = new double[]{strafe, forward, rotation};
        expectedHeadingField = new double[]{strafe, forward, rotation};

        double vy = ROBOT_LENGTH / 2;
        double vx = ROBOT_WIDTH / 2;


        expectedVel = new double[]{-vy, vx, -vy, -vx, vy, vx, vy, -vx};
        expectedVelField = new double[]{-vy, vx, -vy, -vx, vy, vx, vy, -vx};

        getRobotHeading();
        getRobotHeadingField();
        calculateWheelVelocities();
        calculateWheelVelocitiesField();
    }

    @Test
    public void driveForward() {
        forward = 1;
        strafe = 0;
        rotation = 0;
        gyro =  -3 * Math.PI / 2;

        expectedHeading = new double[]{0, forward, 0};
        expectedHeadingField = new double[]{0, forward, 0};
        expectedVel = new double[]{0, forward, 0, forward, 0, forward, 0, forward};
        expectedVelField = new double[]{0, forward, 0, forward, 0, forward, 0, forward};

        getRobotHeading();
        getRobotHeadingField();
        calculateWheelVelocities();
        calculateWheelVelocitiesField();
    }

    @Test
    public void diagonal() {
        forward = 0.5;
        strafe = 0.5;
        rotation = 0;

        gyro = 3 * Math.PI / 2;

        expectedHeading = new double[]{strafe, -forward, rotation};
        expectedHeadingField = new double[]{0.5, -0.5, rotation};

        expectedVel = new double[]{strafe, -forward, strafe, -forward, strafe, -forward, strafe, -forward};
        expectedVelField = new double[]{0.5, -0.5, 0.5, -0.5, 0.5, -0.5, 0.5, -0.5};

        getRobotHeading();
        getRobotHeadingField();
        calculateWheelVelocities();
        calculateWheelVelocitiesField();

    }

    @Ignore
    public void diagonalRotation() {

    }

    public void getRobotHeading() {
        robotHeading = swerveDrive.getRobotHeading(forward, strafe, rotation, gyro);
        for (int i = 0; i < 3; i++)
            System.out.print(robotHeading[i] + " ");
        System.out.println();

        Assert.assertArrayEquals(expectedHeading, robotHeading, delta);
    }


    public void getRobotHeadingField() {
        robotHeadingField = swerveField.getRobotHeading(forward, strafe, rotation, gyro);
        for (int i = 0; i < 3; i++)
            System.out.print(robotHeadingField[i] + " ");
        System.out.println();
        Assert.assertArrayEquals(expectedHeadingField, robotHeadingField, delta);
    }
    

    public void calculateWheelVelocities() {
        double[] wheelVelocities = swerveDrive.calculateWheelVelocities(robotHeading);

        for(int i = 0; i < 8; i++) {
            System.out.print(wheelVelocities[i] + " ");
        }

        System.out.println();

        Assert.assertArrayEquals(expectedVel, wheelVelocities, delta);
    }


    public void calculateWheelVelocitiesField() {
        double[] wheelVelField = swerveField.calculateWheelVelocities(robotHeadingField);
        System.out.println(Arrays.toString(wheelVelField));
        Assert.assertArrayEquals(expectedVelField, wheelVelField, delta);
    }

    @Test
    public void calculateLockAngles() {
        double[] expected = new double[]{Math.PI / 4, 3 * Math.PI / 4, 5 * Math.PI / 4, 7 * Math.PI / 4};
        Assert.assertArrayEquals(expected, swerveDrive.calculateLockAngles(), delta);
    }

}