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

    public Line(Rotation2d angle, Translation2d position, String name) {
        slope = Math.tan(angle.getRadians());
        centerX = position.getMeasureX();
        centerY = position.getMeasureY();

        this.doubles = new NTDoubleSection(name + "_line", "slope", "centerX", "centerY");

        doubles.set("slope", slope);
        doubles.set("centerX", centerX.in(Meters));
        doubles.set("centerY", centerY.in(Meters));
    }

    /**
     * Modifies the current line, moving it in the direction by d
     * units.
     * 
     * @return this Line
     */
    public Line offsetBy(Distance d) {
        Translation2d trans = new Translation2d(d.in(Meters), // unit is required to be meters
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
     * <p>
     * This formula is most similar to this one:
     * https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Another_formula
     * Note that the sign of all the terms within Math.abs is flipped when compared
     * to the formula presented on the Wikipedia article.
     *
     * However, the absolute value is removed, in order to not accelerate towards
     * infinity when the line is passed.
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
