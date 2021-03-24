package robot;

import frc.robot.Utils;
import org.junit.Assert;
import org.junit.Test;

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
        double[][] mat = { {0, 1, 0.5}, {1, 0, -0.5}, {0, 1, 0.5}, {1, 0, .5}, {0, 1, -0.5}, {1, 0, .5}, {0, 1, -0.5}, {1, 0, -0.5} };
        double[] v = {.7, 0, 0};
        double[] vec = Utils.matrixVectorMult(mat, v);
        double[] expected = {0, .7, 0, .7, 0, .7, 0, .7};

        Assert.assertArrayEquals(expected, vec, delta);

        double[][] m = { {3, 2, .5}, {1, 4, .25}, {6, 7, 2}, {2, 4.5, 5} };
        double[] V = {.5, 2, 3};
        double[] vector = Utils.matrixVectorMult(m, V);
        double[] exp = {7.0, 9.25, 23.0, 25.0};

        Assert.assertArrayEquals(exp, vector, delta);
    }
}