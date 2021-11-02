package frc.robot.utils;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc.robot.Constants;
import org.apache.commons.lang.math.DoubleRange;

public class Utils {
    /**
     * Map the input from one range to another, and clamp the input in case the value not in range.
     *
     * @param minIn  the minimal input value.
     * @param maxIn  the maximal input value.
     * @param minOut the minimal output value.
     * @param maxOut the maximal output value.
     * @param input  the input to map.
     * @return the input after mapping.
     */
    public static double getMappedRange(double minIn, double maxIn,
                                        double minOut, double maxOut,
                                        double input) {
        if (input < minIn)
            return minOut;
        if (input > maxIn)
            return maxOut;
        return minOut + ((input - minIn) * (maxOut - minOut)) / (maxIn - minIn); // as described in https://rosettacode.org/wiki/Map_range
    }

    /**
     * Map the input from one range to another, and clamp the input in case the value not in range.
     *
     * @param rangeIn  the range of the input.
     * @param rangeOut the output range og the output.
     * @param input    the input to map.
     * @return the input after mapping.
     */
    public static double getMappedRange(double input, DoubleRange rangeIn, DoubleRange rangeOut) {
        return getMappedRange(rangeIn.getMinimumDouble(), rangeIn.getMaximumDouble(),
                rangeOut.getMinimumDouble(), rangeOut.getMaximumDouble(), input);
    }
}
