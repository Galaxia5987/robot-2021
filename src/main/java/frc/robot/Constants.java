/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double VOLTAGE = 12; //[volts]

    public static class ColorWheel {

        //RGB values (0-255)
        public static final Color RED_TARGET = ColorMatch.makeColor(255, 0, 0);
        public static final Color GREEN_TARGET = ColorMatch.makeColor(0, 255, 0);
        public static final Color BLUE_TARGET = ColorMatch.makeColor(0, 0, 255);
        public static final Color YELLOW_TARGET = ColorMatch.makeColor(255, 255, 0);

        public static final int REQUIRED_SPINS = 3; //Amount of spins the wheel must complete to win
        public static final int COLOR_WHEEL_SLOTS = 8; //Amount of color slots in the wheel
        public static final double REDUCE_POWER_BY = 0.5; //Percentage to cut the motor power by

        public static final String[] COLORS = new String[]{"YELLOW", "BLUE", "GREEN", "RED"};
    }
}
