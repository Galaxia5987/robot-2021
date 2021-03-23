package frc.robot.utils;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 * This class is a modification of the regular laser sensor class, which adds a deadband, to ensure that the sensor
 * does not toggle multiple times.
 * this is crucial for the conveyor, because the amount of balls that enter are counted by the amount of times
 * the sensor switches.
 */
public class DeadbandProximity {
    private final AnalogInput sensor1;
    private final AnalogInput sensor2;
    private final double lostVoltage;
    private final double senseVoltage;
    private boolean objectSensed = false;
    private boolean stateChanged = false;

    /**
     * @param sensor1port      The channel number to represent. 0-3 are on-board 4-7 are on the MXP port.
     * @param sensor2port      The channel number to represent. 0-3 are on-board 4-7 are on the MXP port.
     * @param lostVoltage  The voltage where an object is considered as lost.
     * @param senseVoltage The voltage where an object is considered as sensed.
     */
    public DeadbandProximity(int sensor1port, int sensor2port, double lostVoltage, double senseVoltage) {
        this.sensor1 = new AnalogInput(sensor1port);
        this.sensor2 = new AnalogInput(sensor2port);
        this.lostVoltage = lostVoltage;
        this.senseVoltage = senseVoltage;
    }

    /**
     * Update the state of the laser sensor.
     * An object considered as sensed when the value goes above the {@link #senseVoltage}.
     * An object considered as lost when the value goes below the {@link #lostVoltage}.
     * This method ensures that the sensor doesn't toggle rapidly because of sensor noise.
     */
    public void updateState() {
        final boolean lastState = objectSensed;
        if (isObjectAway()) {
            objectSensed = false;
        } else if (isObjectClose()) {
            objectSensed = true;
        }
        stateChanged = (objectSensed != lastState);
    }

    /**
     * Get whether the sensor senses an object.
     * If you wish to check whether the sensor lost the object, use {@link #isObjectAway()} instead.
     *
     * @return whether the sensor sense an object.
     */
    private boolean isObjectClose() {
        return sensor1.getValue() > senseVoltage && sensor2.getValue() > senseVoltage;
    }

    /**
     * Get whether the sensor lost the object.
     * If you wish to check whether the sensor sense the object, use {@link #isObjectClose()} instead.
     *
     * @return whether the sensor lost the object.
     */
    private boolean isObjectAway() {
        return sensor1.getValue() < lostVoltage && sensor2.getValue() < lostVoltage;
    }

    /**
     * Get whether the sensor sensed and then lost (or the other way around) an object.
     *
     * @return whether an object passed the sensor sensor.
     */
    public boolean hasStateChanged() {
        return stateChanged;
    }

    /**
     * Get whether an object has been sensed by the sensor.
     *
     * @return whether an object was sensed by the sensor.
     */
    public boolean hasObjectSensed() {
        return objectSensed;
    }
}
