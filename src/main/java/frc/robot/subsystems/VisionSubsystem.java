// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.NTDoubleSection;

public class VisionSubsystem extends SubsystemBase {
    DrivetrainSubsystem drivetrain;
    @SuppressWarnings("unused")
    private double driftEstimateTicks;

    private static final String LIMELIGHTS[] = { "limelight_left", "limelight_right" };

    // Initializes the vision subsystem
    public VisionSubsystem(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
    }

    private void updateVisionMeasurement(String limelight_name) {
        LimelightHelpers.SetRobotOrientation(limelight_name,
                drivetrain.getEstimatedPosition().getRotation().getDegrees(), 0, 0,
                0, 0, 0);

        LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight_name);

        boolean reject = false;
        if (estimate.tagCount == 1 && estimate.rawFiducials.length == 1) {
            var firstFiducial = estimate.rawFiducials[0];
            if (firstFiducial.ambiguity > .7 || firstFiducial.distToCamera > 3) {
                reject = true;
            }
        } else if (estimate.tagCount == 0) {
            reject = true;
        }

        if (!reject) {
            double[] stddevs = NetworkTableInstance.getDefault().getTable(limelight_name).getEntry("stddevs")
                    .getDoubleArray(new double[12]);
            double mt2xdev = stddevs[6];
            double mt2ydev = stddevs[7];
            drivetrain.addVisionMeasurement(estimate.pose, estimate.timestampSeconds,
                    VecBuilder.fill(mt2xdev, mt2ydev, 9999999));
        }

        if (Double.isNaN(drivetrain.getEstimatedPosition().getX())
                || Double.isInfinite(drivetrain.getEstimatedPosition().getX()) ||
                Double.isNaN(drivetrain.getEstimatedPosition().getY())
                || Double.isInfinite(drivetrain.getEstimatedPosition().getY())) {
            drivetrain.resetPose(LimelightHelpers.getBotPose2d_wpiBlue(limelight_name));
            System.out.println("reset pose estimator due to NaN or inf");
        }
    }

    // all in degrees
    NTDoubleSection telemetry_doubles = new NTDoubleSection("test_telem_doubles", "mt1_rz", "mt2_rz", "pigeon",
            "poseest", "averagedMt1");

    @Override
    public void periodic() {
        for (String limelight : LIMELIGHTS) {
            updateVisionMeasurement(limelight);
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
