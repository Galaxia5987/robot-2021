package frc.robot.utils;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc.robot.Constants;

public class Functions {
    public static Pose2d getPortLocation(boolean innerPort) {
        return innerPort ? Constants.Vision.RED_INNER_POWER_PORT_LOCATION : Constants.Vision.RED_OUTER_POWER_PORT_LOCATION;
    }

}
