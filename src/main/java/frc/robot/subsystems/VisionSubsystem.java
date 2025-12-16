// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import java.util.Arrays;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.NTDoubleSection;

public class VisionSubsystem extends SubsystemBase {
    DrivetrainSubsystem drivetrain;
    @SuppressWarnings("unused")
    private double driftEstimateTicks;

    public static final String LIMELIGHTS[] = { "limelight-left" };

    private NTDoubleSection doubles = new NTDoubleSection("vision", "drivetrainYaw", "drivetrainOmegaZ",
            "drivetrainYaw", "status");

    public VisionSubsystem(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        for (String limelight : LIMELIGHTS) {
            doubles.addEntry(limelight + "_status");
        }
    }

    private boolean trySeedPigeon(String name) {
        if (!LimelightHelpers.getTV(name)) {
            return false;
        }
        drivetrain.getPigeon2()
                .setYaw(LimelightHelpers.getBotPoseEstimate_wpiBlue(name).pose.getRotation().getDegrees());
        drivetrain.resetPose(
                new Pose2d(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name).pose.getTranslation(),
                        drivetrain.getPigeon2().getRotation2d()));
        return true;
    }

    public void seedPigeon() {
        trySeedPigeon("limelight-left");
    }

    private LimelightHelpers.PoseEstimate lastEstimate;

    private LimelightHelpers.PoseEstimate getMt2Estimate(String limelightName) {
        LimelightHelpers.SetRobotOrientation(limelightName, drivetrain.getPigeon2().getYaw().getValueAsDouble(), 0, 0,
                0, 0, 0);
        doubles.set("drivetrainYaw", drivetrain.getPigeon2().getYaw().getValueAsDouble());
        doubles.set("drivetrainOmegaZ", drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble());

        return LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
    }

    private boolean drivetrainIsNaNOrInf() {
        return Double.isNaN(drivetrain.getEstimatedPosition().getX())
                || Double.isInfinite(drivetrain.getEstimatedPosition().getX()) ||
                Double.isNaN(drivetrain.getEstimatedPosition().getY())
                || Double.isInfinite(drivetrain.getEstimatedPosition().getY());
    }

    public void maybeUpdateVisionMeasurement(String limelightName) {
        LimelightHelpers.PoseEstimate estimate;
        if (VisionConstants.IS_MT2) {
            estimate = getMt2Estimate(limelightName);
        } else {
            estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        }

        if (estimate == null) {
            doubles.set(limelightName + "_status", 1);
            return;
        }

        if (lastEstimate != null && estimate.pose == lastEstimate.pose) {
            doubles.set(limelightName + "_status", 5);
            return;
        }
        lastEstimate = estimate;

        if (estimate.tagCount == 1 && estimate.rawFiducials.length == 1) {
            var firstFiducial = estimate.rawFiducials[0];
            if (firstFiducial.ambiguity > .7) {
                doubles.set(limelightName + "_status", 2);
                return;
            } else if (limelightName == "limelight-right" // our Limelight 3
                    && firstFiducial.distToCamera > 3) {
                doubles.set(limelightName + "_status", 3);
                return;
            }
        } else if (estimate.tagCount == 0) {
            doubles.set(limelightName + "_status", 4);
            return;
        }

        updateVisionMeasurement(limelightName, estimate);
    }

    private int pigeonTimer = 0;

    private void updateVisionMeasurement(String limelightName, LimelightHelpers.PoseEstimate estimate) {
        if (Arrays.stream(estimate.rawFiducials)
                .allMatch(tag -> tag.distToCamera < VisionConstants.RESET_PIGEON_DISTANCE.in(Meters))
                && pigeonTimer > VisionConstants.RESET_PIGEON_TICKS) {
            drivetrain.getPigeon2().setYaw(estimate.pose.getRotation().getMeasure());
            pigeonTimer = 0;
        }
        pigeonTimer++;

        doubles.set(limelightName + "_status", 0);
        double[] stddevs = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("stddevs")
                .getDoubleArray(new double[12]);
        double xdev, ydev;
        if (VisionConstants.IS_MT2) {
            xdev = stddevs[6];
            ydev = stddevs[7];
        } else {
            xdev = stddevs[0];
            ydev = stddevs[1];
        }
        if (drivetrain.getEstimatedPosition().getTranslation()
                .getDistance(estimate.pose.getTranslation()) > 4
                || drivetrainIsNaNOrInf()) {
            drivetrain.resetPose(estimate.pose);
        }
        drivetrain.addVisionMeasurement(estimate.pose, estimate.timestampSeconds,
                VecBuilder.fill(xdev, ydev, 9999999));
    }

    @Override
    public void periodic() {
        for (String limelight : LIMELIGHTS) {
            maybeUpdateVisionMeasurement(limelight);
        }
    }

    private void resetToMt1() {
        for (String limelight : LIMELIGHTS) {
            if (LimelightHelpers.getTV(limelight)) {
                drivetrain.resetPose(LimelightHelpers.getBotPose2d(limelight));
                System.out.println("Robot pose set to mt1 report from " + limelight);
                break;
            }
        }
    }

    public Command getWaitForMt1() {
        return Commands.idle() // wait
                .until(() -> { // until any limelight sees a tag
                    return Arrays.stream(LIMELIGHTS).anyMatch(limelight -> LimelightHelpers.getTV(limelight));
                })
                .andThen(Commands.run(() -> resetToMt1(), drivetrain).withTimeout(.1)) // reset pose
        ;
    }
}
