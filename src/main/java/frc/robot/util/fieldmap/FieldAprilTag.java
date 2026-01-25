package frc.robot.util.fieldmap;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Distance;

public record FieldAprilTag(
        int id,
        Distance size,
        Pose3d wpiBluePose // coordinates of the april tag where lower left corner (blue) is 0,0,0
) {}
