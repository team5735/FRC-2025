package frc.robot.util.fieldmap;

import edu.wpi.first.units.measure.Distance;
import java.util.List;

public record FieldSpec(
        // uses wpiBlue coords (0,0,0 in blue alliance corner)
        Distance length, // the longer end of the field, the x- direction
        Distance width,  // shorter end of the field, y- direction
        List<FieldAprilTag> tags
) {}
