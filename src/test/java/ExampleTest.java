import edu.wpi.first.wpilibj.controller.PIDController;
import org.junit.Assert;
import org.junit.jupiter.api.Test;

public class ExampleTest {

    @Test
    public void exampleTest() {
        PIDController pidController = new PIDController(0.2, 0, 0);
        System.out.println(pidController.calculate(10));
        Assert.assertEquals(1,0);
    }
}
