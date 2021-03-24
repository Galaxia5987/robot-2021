package frc.robot.subsystems.shooter;

import frc.robot.Constants;

import java.io.*;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;

/**
 * The linear regression contains a list of velocity based on the distance, and should give an approximation of the velocity to apply.
 * In order to the linear regression object to approximate better, you need to pass more values.
 * The object calculate calculate a straight line between the closest point, and retrieve the velocity (in the straight line between the records).
 */
public class LinearRegression {
    private static final String DELIMITER = ",";
    private final Map<Double, Double> distanceVelocityMap; // <key - distance, value - velocity>, TreeMap iterates according to the "natural ordering" of the keys (in this case, smallest distance first).

    public LinearRegression(String pathToCsv) {
        this(new File(pathToCsv));
    }

    public LinearRegression(File csv) {
        this.distanceVelocityMap = new TreeMap<>();
        if (csv.isFile()) {
            try {
                read(new FileReader(csv));
            } catch (Exception e) {
                e.printStackTrace();
            }
        } else
            System.err.println("File not found: " + csv.getName());
    }

    public LinearRegression(Map<Double, Double> map) {
        this.distanceVelocityMap = new TreeMap<>(map); // just because I want to order the values by the key.
    }

    public LinearRegression(Reader reader) {
        this.distanceVelocityMap = new TreeMap<>();
        read(reader);
    }

    private void read(Reader reader) {
        String line = null;
        try (BufferedReader bufferedReader = new BufferedReader(reader)) {
            while ((line = bufferedReader.readLine()) != null) {
                String[] pair = line.split(DELIMITER);
                this.distanceVelocityMap.put(Double.parseDouble(pair[0]), Double.parseDouble(pair[1]));
            }
        } catch (IOException e) {
            System.err.println("MovingAverage class can't read the file");
        } catch (IndexOutOfBoundsException e) {
            System.err.println("Please check that every line has a pair of values, the problem occurred on this line " + line);
        }
    }

    /**
     * Retrieve an approximation for the velocity based on the distance.
     *
     * @param distance the distance from the target. [meters]
     * @return an approximation of the velocity to apply
     */
    public double estimateVelocityFromDistance(double distance) {
        double[] distances = getClosestDistances(distance);

        if (Math.abs(distances[0] - distance) <= Constants.Shooter.ALLOWED_ERROR) {
            return distanceVelocityMap.get(distances[0]);
        }

        double lowerVelocity = distanceVelocityMap.get(distances[0]);
        double higherVelocity = distanceVelocityMap.get(distances[1]);
        // y = mx + b
        // m = delta y / delta x
        double m = (higherVelocity - lowerVelocity) / (distances[1] - distances[0]);
        // b = y - m*x
        double b = higherVelocity - m * distances[1];

        return distance * m + b;
    }

    /**
     * Get a pair of distances of the closets distances, one below and one above.
     *
     * @param distance the distance to return the pairs.
     * @return a 2d array represents the closest values(below {distance, velocity}, above {distance, velocity}).
     */
    private double[] getClosestDistances(double distance) {
        Set<Double> keys = distanceVelocityMap.keySet();
        double min = 0;

        for (double val : keys) {
            if (val > distance) {
                return new double[]{min, val};
            }
            min = val;
        }
        return new double[]{min, min}; //Out of range, either way the robot can't shoot at this distance
    }
}
