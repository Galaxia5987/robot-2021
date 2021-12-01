package frc.robot;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class Utils {

    /**
     * recreates the results of Math.floorMod() for Double type variables.
     * The result is the unsigned remainder of the mod method.
     *
     * @param value the numerator
     * @param mod   the denominator
     * @return the remainder of the division
     */
    public static double floorMod(double value, double mod) {
        value %= mod;
        value += mod;
        value %= mod;
        return value;
    }

    /**
     * converts cartesian coordinates to polar coordinates
     *
     * @param x the X cartesian coordinate
     * @param y the Y cartesian coordinate
     * @return a vector of length 2 with the polar coordinates [length, angle]
     */
    public static double[] cartesianToPolar(double x, double y) {
        double[] polar = new double[2];
        polar[0] = Math.hypot(x, y);
        polar[1] = Math.atan2(y, x);

        return polar;
    }

    /**
     * @param radius the radius of the vector
     * @param angle  the angle of the vector
     * @return the cartesian representation with x and y components
     */
    public static double[] polarToCartesian(double radius, double angle) {
        double[] coordinates = new double[2];
        coordinates[0] = Math.sin(angle) * radius;
        coordinates[1] = Math.cos(angle) * radius;
        return coordinates;
    }

    /**
     * Calculates a matrix-vector multiplication.
     * assuming that the number of columns in the matrix is equal to the number of rows in the vector.
     *
     * @param m a matrix of size R * C
     * @param v a vector of size C
     * @return a vector of length R with the corresponding matrix multiplication
     */
    public static double[] matrixVectorMult(double[][] m, double[] v) {
        double sum;
        double[] out = new double[m.length];

        for (int i = 0; i < m.length; i++) {
            sum = 0;
            for (int j = 0; j < v.length; j++) {
                sum += m[i][j] * v[j];
            }
            out[i] = sum;
        }

        return out;
    }

    /**
     * sets the value of the joystick to 0 if the value is less than the threshold
     *
     * @param val       the joystick value
     * @param threshold the threshold value
     * @return 0 if val is less than the threshold else val
     */
    public static double joystickDeadband(double val, double threshold) {
        if (Math.abs(val) < threshold)
            return 0;
        return val;
    }

    public static Rotation2d pathPlannerRotationToRealRotation(Rotation2d rotation2d) {
        rotation2d = Rotation2d.fromDegrees(rotation2d.getDegrees() < 0 ? rotation2d.getDegrees() + 360 : rotation2d.getDegrees());
        rotation2d = Rotation2d.fromDegrees(360 - rotation2d.getDegrees());
        return rotation2d;
    }
}
