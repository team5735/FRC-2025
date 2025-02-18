package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Line {
    private double slope;

    private double centerX;
    private double centerY;

    NTDoubleSection doubles;

    public Line(Pose2d pose, String name) {
        slope = Math.tan(pose.getRotation().getRadians());
        centerX = pose.getTranslation().getX();
        centerY = pose.getTranslation().getY();

        this.doubles = new NTDoubleSection(name + "_line", "slope", "centerX", "centerY");

        doubles.set("slope", slope);
        doubles.set("centerX", centerX);
        doubles.set("centerY", centerY);
    }

    /**
     * Modifies the current line, moving it in the direction by d
     * units.
     * 
     * <p>
     * I know you can do the same thing with the constructor Translation2d(double,
     * Rotation2d), but this is easier to check and more descriptive.
     * 
     * deltaVectorTheta is the angle of the vector between the old and new center
     * points for the Line.
     * 
     * @return this Line
     */
    public Line offsetBy(double d) {
        double deltaVectorTheta = Math.atan(slope) + Math.PI / 2;
        double deltaX = Math.cos(deltaVectorTheta) * d;
        double deltaY = Math.sin(deltaVectorTheta) * d;
        this.centerX += deltaX;
        this.centerY += deltaY;

        doubles.set("centerX", centerX);
        doubles.set("centerY", centerY);
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
    public double getPIDMeasurement(Translation2d position) {
        return slope * position.getX() - position.getY() + centerY - slope * centerX
                / Math.sqrt(1 + slope * slope);
    }

    /**
     * {@returns a {@link Translation2d} such that adding it to position returns a
     * point on the line represented by this object}
     */
    public Translation2d getVectorFrom(Translation2d position) {
        double perpendicularAngle = Math.atan(slope) + Math.PI / 2;
        return new Translation2d(1, Rotation2d.fromRadians(perpendicularAngle));
    }

    public Translation2d getVectorAlongLine() {
        return new Translation2d(1, Rotation2d.fromRadians(Math.atan(slope)));
    }
}
