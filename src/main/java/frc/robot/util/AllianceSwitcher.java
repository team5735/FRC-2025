package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceSwitcher {
    public AllianceSwitcher() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Determines whether we need to switch the inputs to the other side of the
     * field. Inputs are always on the red side by default, so if it's blue, we need
     * to switch.
     *
     * @return Whether a switch is necessary.
     */
    private static boolean needsSwitch() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == Alliance.Blue;
        }
        return false;
    }

    /**
     * Returns the input, transformed if necessary according to needsSwitch.
     * 
     * @param in The translation to be transformed
     *
     * @return The possibly transformed translation.
     */
    public static Translation2d compute(Translation2d in) {
        if (needsSwitch()) {
            return new Translation2d(-in.getX(), in.getY());
        }
        return in;
    }

    /**
     * Returns the input, transformed if necessary according to needsSwitch.
     * 
     * @param in The translation to be transformed
     *
     * @return The possibly transformed translation.
     */
    public static Translation3d compute(Translation3d in) {
        if (needsSwitch()) {
            return new Translation3d(-in.getX(), in.getY(), in.getZ());
        }
        return in;
    }

    /**
     * Returns the input, transformed if necessary according to needsSwitch.
     * 
     * @param in The pose to be transformed
     *
     * @return The possibly transformed translation.
     */
    public static Pose2d compute(Pose2d in) {
        return new Pose2d(compute(in.getTranslation()), in.getRotation());
    }

    /**
     * Returns the input, transformed if necessary according to needsSwitch.
     * 
     * @param in The pose to be transformed
     *
     * @return The possibly transformed translation.
     */
    public static Pose3d compute(Pose3d in) {
        return new Pose3d(compute(in.getTranslation()), in.getRotation());
    }
}
