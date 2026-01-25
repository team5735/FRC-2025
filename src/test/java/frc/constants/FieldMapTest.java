package frc.constants;

import java.nio.file.Path;
import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import static edu.wpi.first.units.Units.Meters;

import frc.robot.util.fieldmap.FieldFmapReader;
import frc.robot.util.fieldmap.FieldAprilTag;
import frc.robot.util.fieldmap.FieldSpec;

public class FieldMapTest {
    @Test
    void fieldMapTest() throws Exception {
        FieldSpec field = FieldFmapReader.readFmap(Path.of("./src/main/java/frc/robot/constants/FRC2026_ANDYMARK.fmap"));

        for (FieldAprilTag tag : field.tags()) {
            Pose3d pose = tag.wpiBluePose();
            Translation3d t = pose.getTranslation();
            Rotation3d r = pose.getRotation();

            System.out.printf(
                    "Tag %d\n" +
                    "  Size (m): %.3f\n" +
                    "  Translation (m):\n" +
                    "    x = %.3f\n" +
                    "    y = %.3f\n" +
                    "    z = %.3f\n" +
                    "  Rotation (deg):\n" +
                    "    roll  = %.2f\n" +
                    "    pitch = %.2f\n" +
                    "    yaw   = %.2f\n\n",
                    tag.id(),
                    tag.size().in(Meters),
                    t.getX(),
                    t.getY(),
                    t.getZ(),
                    Math.toDegrees(r.getX()),
                    Math.toDegrees(r.getY()),
                    Math.toDegrees(r.getZ())
            );
        }
        
    }
}
