package frc.robot.valuetuner;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.DoubleConsumer;


/**
 * This class holds all the key value constants that will show up in the value tuner.
 */
public class WebConstant {
    private static final Map<String, ConstantObject> constantMap = new ConcurrentHashMap<>();
    private final ConstantObject constant;

    public WebConstant(String key, double value) {
        this.constant = new ConstantObject(key, value);
        constantMap.put(key, this.constant);
    }

//    public WebConstant(String key, double value, DoubleConsumer changeListener) {
//        this.constant = new ConstantObject(key, value, changeListener);
//    }

//    public void setChangeListener(DoubleConsumer changeListener) {
//        constant.setChangeListener(changeListener);
//    }

    public static Map<String, ConstantObject> getConstantMap() {
        return constantMap;
    }

    public double get() {
        return constant.getValue();
    }

}
