/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int TALON_TIMEOUT = 10; // [ms]
    public static final double NOMINAL_VOLTAGE = 12; // [volts]

    // TODO: Change the values
    public static final class Conveyor {
        public static final double FORWARD_PEAK = 1; // [%]
        public static final double REVERSE_PEAK = 1; // [%]
        public static final SupplyCurrentLimitConfiguration SUPPLY_CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(false, 40,2,0); //prevent breakers from tripping.
        public static final StatorCurrentLimitConfiguration STATOR_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(false, 40,0,0); // control the acceleration

        public static final int INITIAL_BALLS_AMOUNT = 3;
        public static final int MAX_BALLS_AMOUNT = 5;

        public static final double SHOOTER_PROXIMITY_LOST_VOLTAGE = 0; // [volts]
        public static final double SHOOTER_PROXIMITY_SENSE_VOLTAGE = 0; // [volts]
        public static final double FUNNEL_PROXIMITY_LOST_VOLTAGE = 0; // [volts]
        public static final double FUNNEL_PROXIMITY_SENSE_VOLTAGE = 0; // [volts]
        public static final double[] YELLOW_RGB = {0.317, 0.552, 0.127};
        public static final double[] GREEN_RGB = {0.16, 0.571, 0.269};
        public static final double[] RED_RGB = {0.492, 0.348, 0.145};
        public static final double[] BLUE_RGB = {0.132, 0.427, 0.442};
        public static final double CONVEYOR_MOTOR_RETURN_POWER = 0; // [%]
    }
}
