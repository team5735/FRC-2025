package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Arc {
    Translation2d center;
    double radius;
    Rotation2d thetaStart, thetaEnd;

    public Arc(Translation2d center, double radius, Rotation2d thetaStart, Rotation2d thetaEnd) {
        this.center = center;
        this.radius = radius;
        this.thetaStart = thetaStart;
        this.thetaEnd = thetaEnd;
    }

    private static double normalizeAngle(double theta) {
        double twoPi = 2 * Math.PI;
        theta = theta % twoPi;
        return theta < 0 ? theta + twoPi : theta;
    }

    private static boolean angleInRange(double theta, double start, double end) {
        theta = normalizeAngle(theta);
        start = normalizeAngle(start);
        end = normalizeAngle(end);

        if (start <= end) {
            return theta >= start && theta <= end;
        } else {
            return theta >= start || theta <= end;
        }
    }

    private Translation2d nearestTranslation2dOnCircleArc(Translation2d position) {
        Translation2d givenToCenter = center.minus(position);
        Rotation2d thetaA = givenToCenter.getAngle();

        if (angleInRange(thetaA.getRadians(), thetaStart.getRadians(), thetaEnd.getRadians())) {
            return new Translation2d(radius, thetaA);
        }

        Translation2d p1 = position.plus(new Translation2d(radius, thetaStart));
        Translation2d p2 = position.plus(new Translation2d(radius, thetaEnd));

        return center.getDistance(p1) <= center.getDistance(p2) ? p1 : p2;
    }

}
