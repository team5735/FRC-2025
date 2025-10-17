// idea stolen with <3 from team 190's 2024 code.

package frc.robot.util;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class TunableNumber {
    private static final String kBaseTable = "tunables";

    private DoubleSubscriber subscriber;

    /**
     * Creates a new TunableNumber named name, the default table, and an initVal
     * of 0.
     *
     * @param name The name of the TunableNumber
     */
    public TunableNumber(String name) {
        init("", name, 0);
    }

    /**
     * Creates a new TunableNumber named name, the default table, and an initVal
     * of initVal.
     *
     * @param name    The name of the TunableNumber
     * @param initVal The initial value of the TunableNumber
     */
    public TunableNumber(String name, double initVal) {
        init("", name, initVal);
    }

    /**
     * Creates a new TunableNumber named name, the subtable subtable, and an initVal
     * of 0.
     *
     * @param subtable The name of the subtable to use
     * @param name     The name of the TunableNumber
     */
    public TunableNumber(String subtable, String name) {
        init(subtable, name, 0);
    }

    /**
     * Creates a new TunableNumber named name, the subtable subtable, and an initVal
     * of initVal.
     *
     * @param subtable The name of the subtable to use
     * @param name     The name of the TunableNumber
     * @param initVal  The initial value of the TunableNumber
     */
    public TunableNumber(String subtable, String name, double initVal) {
        init(subtable, name, initVal);
    }

    /**
     * Performs the initialization of a TunableNumber. This function does the actual
     * work of getting the correct table, publishing the default value, and
     * subscribing.
     *
     * @param subtable The subtable of kBaseTable to use.
     * @param name     The name of the TunableNumber
     * @param value    The initial value to publish to the topic.
     */
    private void init(String subtable, String name, double value) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(kBaseTable);
        if (subtable != "") {
            table = table.getSubTable(subtable);
        }
        DoubleTopic topic = table.getDoubleTopic(name);
        topic.publish().set(value);
        subscriber = topic.subscribe(value);
    }

    /**
     * Gets the current value of the TunableNumber.
     *
     * @return The value of this TunableNumber in NetworkTables.
     */
    public double get() {
        return subscriber.get();
    }
}
