package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
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

    public static double normalizeAngle(double theta) {
        double twoPi = 2 * Math.PI;
        theta = theta % twoPi;
        return theta < 0 ? theta + twoPi : theta;
    }

    public boolean angleInRange(double theta) {
        theta = normalizeAngle(theta);
        double start = normalizeAngle(thetaStart.getRadians());
        double end = normalizeAngle(thetaEnd.getRadians());

        if (start <= end) {
            return theta >= start && theta <= end;
        } else {
            return theta >= start || theta <= end;
        }
    }

    public Translation2d nearestPointOnArc(Translation2d position) {
        Translation2d givenToCenter = center.minus(position);
        Rotation2d thetaA = givenToCenter.getAngle();

        if (angleInRange(thetaA.getRadians())) {
            return new Translation2d(radius, thetaA);
        }

        Translation2d p1 = position.plus(new Translation2d(radius, thetaStart));
        Translation2d p2 = position.plus(new Translation2d(radius, thetaEnd));

        return center.getDistance(p1) <= center.getDistance(p2) ? p1 : p2;
    }

    public Pose2d nearestPoseOnArc(Translation2d position) {
        Translation2d targetTrans = nearestPointOnArc(position);
        Rotation2d botAngle = center.minus(targetTrans).getAngle();
        return new Pose2d(targetTrans, botAngle);
    }
}
