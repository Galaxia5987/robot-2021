package frc.robot;

public class Ports {

    public static class PTO {
        public static final int MASTER = 32;
        public static final int SLAVE = 33;
        public static final int PISTON_FORWARD = 0;
    }

    public static class Climber {
        public static final int STOPPER_REVERSE_CHANNEL = 1;
        public static final int GEARBOX_REVERSE_CHANNEL = 3;

        public static final boolean IS_SENSOR_PHASE_INVERTED = false;
        public static final boolean IS_MASTER_INVERTED = false;
        public static final boolean IS_SLAVE_INVERTED = false;
    }

    public static final class Shooter {
        public static final boolean MAIN_INVERTED = false;
        public static final boolean AUX_INVERTED = false;
        public static final boolean IS_SENSOR_INVERTED = false;
    }
}