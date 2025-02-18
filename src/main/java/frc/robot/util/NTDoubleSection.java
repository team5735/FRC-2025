package frc.robot.util;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NTDoubleSection {
    private static final String sectionName = "sections";

    private final NetworkTable table;
    private final String name;
    private Map<String, DoublePublisher> entries = new HashMap<>();

    /**
     * Creates a new NTDoubleSection. This makes the section as a subtable in the
     * top-level table specified by sectionName.
     *
     * @param name The name of the subtable where entries will be put.
     */
    public NTDoubleSection(String name) {
        table = NetworkTableInstance.getDefault().getTable(sectionName).getSubTable(name);
        this.name = name;
    }

    /**
     * Creates a new NTDoubleSection. This makes the section as a subtable in the
     * top-level table specified by sectionName. It also makes the specified entries
     * with addEntry.
     *
     * @param name    The name of the subtable where entries will be put.
     * @param entries The entries to create, as a shorthand.
     */
    public NTDoubleSection(String name, String... entries) {
        table = NetworkTableInstance.getDefault().getTable(sectionName).getSubTable(name);
        this.name = name;
        for (String entry : entries) {
            addEntry(entry);
        }
    }

    /**
     * Creates a new entry and adds it to the list of entries. The entry will be
     * created as a {@link DoubleTopic} with the name name in the table this
     * instance is for.
     * 
     * <p>
     * Nothing is published to the DoubleTopic initially.
     *
     * @param name The name of the entry.
     */
    public void addEntry(String name) {
        DoublePublisher publisher = table.getDoubleTopic(name).publish();
        // publisher.set(0);
        entries.put(name, publisher);
    }

    /**
     * Sets the entry to the value. This makes a {@link DoublePublisher} out of the
     * entry and uses .set() to set its value in NetworkTables.
     *
     * <p>
     * If the entry is not registered in this section, an error is printed and
     * nothing is published.
     * 
     * @param entry The entry to set
     * @param value The value to set the entry to
     */
    public void set(String entry, double value) {
        if (!entries.containsKey(entry)) {
            System.out.println("entry " + entry + " is not in NTDoubleSection " + this.name);
            return;
        }
        entries.get(entry).set(value);
    }
}
