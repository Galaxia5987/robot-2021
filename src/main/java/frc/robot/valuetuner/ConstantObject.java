package frc.robot.valuetuner;

import java.util.function.DoubleConsumer;

/**
 * This class holds a constant value by key.
 */
public class ConstantObject {
    private String key;
    private double value;
    private DoubleConsumer changeListener;

    public ConstantObject(String key, double value) {
        this.key = key;
        this.value = value;
    }

    public ConstantObject(String key, double value, DoubleConsumer changeListener) {
        this.key = key;
        this.value = value;
        this.changeListener = changeListener;
    }

    public void setValue(double value) {
        changeListener.accept(value);
        this.value = value;
    }

    public double getValue() {
        return value;
    }

    public String getKey() {
        return key;
    }

    public void setChangeListener(DoubleConsumer changeListener) {
        this.changeListener = changeListener;
    }
}
