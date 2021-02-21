package robot.subsystems.drivetrain;

import frc.robot.subsystems.drivetrain.SwerveModule;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

public class SwerveModuleTest {

    private SwerveModule swerveModule;
    private double delta = 0.01;

    @Before
    public void setUp() {

    }

    @Test
    public void getTargetAngle() {
        double targetAngle = SwerveModule.getTargetAngle(-1.5 * Math.PI, -Math.PI / 4);
        double expectedAngle = Math.PI / 2;

        Assert.assertEquals(expectedAngle, targetAngle, delta);
    }
}