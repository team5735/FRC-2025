// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.NTDoubleSection;

public class Vision extends SubsystemBase {
    Drivetrain drivetrain;
    @SuppressWarnings("unused")
    private double driftEstimateTicks;

    private static final String LIMELIGHTS[] = { "limelight", "limelight_back" };

    // Initializes the vision subsystem
    public Vision(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public void seedPigeon() {
        if (!LimelightHelpers.getTV("limelight")) {
            return;
        }
        drivetrain.getPigeon2()
                .setYaw(LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight").pose.getRotation().getDegrees());
        drivetrain.resetPose(
                new Pose2d(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight").pose.getTranslation(),
                        drivetrain.getPigeon2().getRotation2d()));
    }

    public Command getSeedPigeon() {
        return run(() -> seedPigeon()).until(() -> LimelightHelpers.getTV("limelight"));
    }

    private void updateVisionMeasurement(String limelight_name) {
        LimelightHelpers.SetRobotOrientation(limelight_name,
                drivetrain.getEstimatedPosition().getRotation().getDegrees(), 0, 0,
                0, 0, 0);

        double[] targetPose_CameraSpace = LimelightHelpers.getTargetPose_CameraSpace(limelight_name);

        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight_name);
        if (mt2 == null) {
            // failed to get mt2 or it's not new
            SmartDashboard.putNumber("poseestimator_status", -1);
            return;
        } else if (mt2.tagCount == 0) {
            // no tags
            SmartDashboard.putNumber("poseestimator_status", -2);
            return;
        } else if (drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble() > 360) {
            // something has probably gone very wrong or this measurement will not be great
            SmartDashboard.putNumber("poseestimator_status", -3);
            return;
        } else if (mt2.pose.getTranslation().getDistance(drivetrain.getEstimatedPosition().getTranslation()) > 1) {
            // limelight estimation is more than 1 meter away from the robot
            SmartDashboard.putNumber("poseestimator_status", -4);
            return;
        } else if (targetPose_CameraSpace != null && targetPose_CameraSpace.length > 2
                && (Math.sqrt(targetPose_CameraSpace[0] * targetPose_CameraSpace[0]
                        + targetPose_CameraSpace[1] * targetPose_CameraSpace[1]) > 1)) {
            // limelight is more than 1 meter away from the target
            SmartDashboard.putNumber("poseestimator_status", -5);
            return;
        } else {
            SmartDashboard.putNumber("poseestimator_status", 0);
        }

        double[] stddevs = NetworkTableInstance.getDefault().getTable(limelight_name).getEntry("stddevs")
                .getDoubleArray(new double[12]);
        double mt2xdev = stddevs[6];
        double mt2ydev = stddevs[7];
        drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds, VecBuilder.fill(mt2xdev, mt2ydev, 9999999));

        if (Double.isNaN(drivetrain.getEstimatedPosition().getX())
                || Double.isInfinite(drivetrain.getEstimatedPosition().getX()) ||
                Double.isNaN(drivetrain.getEstimatedPosition().getY())
                || Double.isInfinite(drivetrain.getEstimatedPosition().getY())) {
            drivetrain.resetPose(LimelightHelpers.getBotPose2d_wpiBlue(limelight_name));
            System.out.println("had to reset pose estimator due to detected NaN");
        }
    }

    // all in deg
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
