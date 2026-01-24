package frc.robot.util;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Meters;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

public final class FieldFmapReader {

    private static final ObjectMapper mapper = new ObjectMapper();

    public static List<FieldAprilTag> readFmap(Path fmapPath) throws IOException {
        JsonNode root = mapper.readTree(fmapPath.toFile());
        JsonNode fiducials = root.get("fiducials");

        // in meters
        double fieldLength = root.get("fieldlength").asDouble();
        double fieldWidth = root.get("fieldwidth").asDouble();

        // coordinates of the field center in wpiBlue coordinate system
        // used to transform april tag poses in field center coordinates to wpiBlue
        // coords (blue alliance lower left is 0,0,0)
        Translation3d fieldCenter = new Translation3d(fieldLength/2.0, fieldWidth/2.0, 0);

        if (fiducials == null || !fiducials.isArray()) {
            throw new IllegalArgumentException("Invalid fmap: missing fiducials array");
        }

        List<FieldAprilTag> tags = new ArrayList<>();

        for (JsonNode node : fiducials) {
            int id = node.get("id").asInt();
            Distance size = Meters.of(node.get("size").asDouble() / 1000.0);
            double[] m = readTransform(node.get("transform"));
            Pose3d pose = poseFromMatrix(m);
            Pose3d wpiBluePose = new Pose3d(pose.getTranslation().plus(fieldCenter),
                                            pose.getRotation());
            tags.add(new FieldAprilTag(id, size, wpiBluePose));
        }

        return tags;
    }

    private static double[] readTransform(JsonNode arrayNode) {
        if (arrayNode == null || arrayNode.size() != 16) {
            throw new IllegalArgumentException("Transform must be a 16-element array");
        }

        double[] m = new double[16];
        for (int i = 0; i < 16; i++) {
            m[i] = arrayNode.get(i).asDouble();
        }
        return m;
    }

    private static Pose3d poseFromMatrix(double[] m) {
        Translation3d translation = new Translation3d(
                m[3],
                m[7],
                m[11]
        );
    
        Matrix<N3, N3> rotMat = new Matrix<>(N3.instance, N3.instance);
        rotMat.set(0, 0, m[0]);
        rotMat.set(0, 1, m[1]);
        rotMat.set(0, 2, m[2]);
        rotMat.set(1, 0, m[4]);
        rotMat.set(1, 1, m[5]);
        rotMat.set(1, 2, m[6]);
        rotMat.set(2, 0, m[8]);
        rotMat.set(2, 1, m[9]);
        rotMat.set(2, 2, m[10]);
    
        Rotation3d rotation = new Rotation3d(rotMat);
    
        return new Pose3d(translation, rotation);
    }
}
