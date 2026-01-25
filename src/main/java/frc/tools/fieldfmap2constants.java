package frc.tools;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;

import static edu.wpi.first.units.Units.Meters;

import frc.robot.util.fieldmap.*;

public final class fieldfmap2constants {
    public static void main(String[] args) {
        if (args.length != 1) {
            System.err.println("Usage: fieldfmap2constants <inputFile>");
            System.exit(1);
        }

        Path inputPath = Path.of(args[0]);

        if (!Files.exists(inputPath)) {
            System.err.println("Input file does not exist: " + inputPath);
            System.exit(2);
        }

        try {
            runConversion(inputPath);
            System.out.println("Conversion complete:");
            System.out.println("  Input : " + inputPath);
        } catch (Exception e) {
            System.err.println("Conversion failed:");
            e.printStackTrace();
            System.exit(3);
        }
    }

    private static void runConversion(Path srcFile) throws IOException {
        FieldSpec field = FieldFmapReader.readFmap(srcFile);

        Map<Integer, FieldAprilTag> tags = new HashMap<>();
        for (FieldAprilTag tag : field.tags()) {
            tags.put(tag.id(), tag);
        }
        System.out.printf("public static final Distance FIELD_LENGTH = Meters.of(%.3f);\n",field.length().in(Meters));
        System.out.printf("public static final Distance FIELD_WIDTH = Meters.of(%.3f);\n",field.width().in(Meters));
        System.out.printf("\n");
        String s = mapAsJavaLiteral(tags);
        System.out.printf("public static final Map<Integer, FieldAprilTag> APRIL_TAGS = %s", s);
    }

    private static String mapAsJavaLiteral(Map<Integer, FieldAprilTag> map) {
        StringBuilder sb = new StringBuilder();

        sb.append("Map.ofEntries(\n");
        String sep = "  ";
        for (var entry : map.entrySet()) {
            sb.append(String.format("%s    Map.entry(%d, %s)\n", sep, entry.getKey(), recordToLiteral(entry.getValue())));
            sep = ", ";
        }
        sb.append(");\n");
        return sb.toString();
    }

    private static String recordToLiteral(FieldAprilTag tag) {
        var t = tag.wpiBluePose().getTranslation();
        var r = tag.wpiBluePose().getRotation();

        double rollDeg = Math.toDegrees(r.getX());
        double pitchDeg = Math.toDegrees(r.getY());
        double yawDeg = Math.toDegrees(r.getZ());

        return String.format(
                "new FieldAprilTag(%d, Meters.of(%.3f), " +
                        "new Pose3d(new Translation3d(%.3f, %.3f, %.3f), " +
                        "new Rotation3d(Math.toRadians(%.1f), Math.toRadians(%.1f), Math.toRadians(%.1f))))",
                tag.id(),
                tag.size().in(Meters),
                t.getX(),
                t.getY(),
                t.getZ(),
                rollDeg,
                pitchDeg,
                yawDeg);
    }

}
