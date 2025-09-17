package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Meters;

public class Line {
    private double slope;

    private Distance centerX;
    private Distance centerY;

    NTDoubleSection doubles;

    /**
     * Construct a line with a given point and a given angle.
     *
     * This constructs a line that passes through the given position and has a slope
     * of tan(angle).
     *
     * @param position point the line is required to pass through
     * @param angle    the angle of the line
     * @param name     the name of the line, used for the NTDoubleSection
     * @return a new Line with the specified requirements
     *
     * @example
     *          ```
     *          // Represents a horizontal line passing through (0, 1).
     *          new Line(new Translation2d(0, 1), Rotation2d.kZero);
     *          ```
     */
    public Line(Translation2d position, Rotation2d angle, String name) {
        slope = Math.tan(angle.getRadians());
        centerX = position.getMeasureX();
        centerY = position.getMeasureY();

        this.doubles = new NTDoubleSection(name + "_line", "slope", "centerX", "centerY");

        doubles.set("slope", slope);
        doubles.set("centerX", centerX.in(Meters));
        doubles.set("centerY", centerY.in(Meters));
    }

    /**
     * Moves the current line by the given distance.
     *
     * Note that this function cares about sign. The line will be moved in the
     * direction of arctan(slope) + pi/2.
     * 
     * @param d the distance to offset the line by
     * @return this Line
     */
    public Line offsetBy(Distance d) {
        Translation2d trans = new Translation2d(d.in(Meters), // Translation2d doesn't fully support units
                Rotation2d.fromRadians(Math.atan(slope)).plus(Rotation2d.kCCW_90deg));
        this.centerX = this.centerX.plus(trans.getMeasureX());
        this.centerY = this.centerY.plus(trans.getMeasureY());

        doubles.set("centerX", centerX.in(Meters));
        doubles.set("centerY", centerY.in(Meters));
        return this;
    }

    /**
     * Returns the distance from position to the line represented by this object.
     *
     * This formula is most similar to <a href=
     * "https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Another_formula">this
     * formula</a>.
     * Note that the sign of all the terms within Math.abs is flipped when compared
     * to the formula presented on the Wikipedia article.
     * <p>
     * However, the absolute value is removed, in order to not accelerate towards
     * infinity when the line is passed.
     *
     * @param position the position to calculate from
     * @return the distance from the position to the line
     * @see getVectorFrom
     */
    public Distance getPIDMeasurement(Translation2d position) {
        return Meters
                .of(slope * position.getMeasureX().in(Meters) - position.getMeasureY().in(Meters) + centerY.in(Meters)
                        - slope * centerX.in(Meters)
                                / Math.sqrt(1 + slope * slope));
    }

    /**
     * Gets the vector from a position to the line.
     *
     * Returns the shortest vector such that when the restult is added to the input
     * position, a vector representing a point on this line is returned.
     * <p>
     * Intended for making a PID that moves the robot to be on the line.
     *
     * @param position the position to calculate from
     * @return the vector between the position and the line
     */
    public Translation2d getVectorFrom(Translation2d position) {
        double perpendicularAngle = Math.atan(slope) + Math.PI / 2;
        return new Translation2d(1, Rotation2d.fromRadians(perpendicularAngle));
    }

    /**
     * Returns a vector that points along this line.
     *
     * Returns a unit vector with the same angle as this line was originally given.
     *
     * @return an {@link Translation2d} of length 1 with the angle of the line
     *
     * @example
     *          ```
     *          Rotation2d angle = ...;
     *          Line line = new Line(..., angle, ...);
     *          line.getVectorAlongLine() == angle;
     *          ```
     */
    public Translation2d getVectorAlongLine() {
        return new Translation2d(1, Rotation2d.fromRadians(Math.atan(slope)));
    }
}
