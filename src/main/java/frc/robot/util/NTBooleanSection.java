package frc.robot.util;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NTBooleanSection {
    private static final String sectionName = "sections";

    private final NetworkTable table;
    private final String name;
    private Map<String, BooleanPublisher> entries = new HashMap<>();

    /**
     * Creates a new NTBooleanSection. This makes the section as a subtable in the
     * top-level table specified by sectionName.
     *
     * @param name The name of the subtable where entries will be put.
     */
    public NTBooleanSection(String name) {
        table = NetworkTableInstance.getDefault().getTable(sectionName).getSubTable(name);
        this.name = name;
    }

    /**
     * Creates a new NTBooleanSection. This makes the section as a subtable in the
     * top-level table specified by sectionName. It also makes the specified entries
     * with addEntry.
     *
     * @param name    The name of the subtable where entries will be put.
     * @param entries The entries to create, as a shorthand.
     */
    public NTBooleanSection(String name, String... entries) {
        table = NetworkTableInstance.getDefault().getTable(sectionName).getSubTable(name);
        this.name = name;
        for (String entry : entries) {
            addEntry(entry);
        }
    }

    /**
     * Creates a new entry and adds it to the list of entries. The entry will be
     * created as a {@link BooleanTopic} with the name name in the table this
     * instance is for.
     *
     * <p>
     * A value of 0 is published to the BooleanTopic initially.
     *
     * @param name The name of the entry.
     */
    public void addEntry(String name) {
        BooleanPublisher publisher = table.getBooleanTopic(name).publish();
        publisher.set(false);
        entries.put(name, publisher);
    }

    /**
     * Sets the entry to the value. This makes a {@link BooleanPublisher} out of the
     * entry and uses .set() to set its value in NetworkTables.
     *
     * <p>
     * If the entry is not registered in this section, an error is printed and
     * nothing is published.
     * 
     * @param entry The entry to set
     * @param value The value to set the entry to
     */
    public void set(String entry, boolean value) {
        if (!entries.containsKey(entry)) {
            System.out.println("entry " + entry + " is not in NTDoubleSection " + this.name);
            return;
        }
        entries.get(entry).set(value);
    }
}
