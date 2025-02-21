// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
    DrivetrainSubsystem drivetrain;
    @SuppressWarnings("unused")
    private double driftEstimateTicks;

    private static final String LIMELIGHTS[] = { "limelight_front", "limelight_back" };

    // Initializes the vision subsystem
    public VisionSubsystem(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
    }

    private void updateVisionMeasurement(String limelight_name) {
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight_name);
        if (mt2 == null) {
            // failed to get mt2
            SmartDashboard.putNumber("poseestimator_status", -1);
        }
        if (mt2.tagCount == 0) {
            // no tags
            SmartDashboard.putNumber("poseestimator_status", -2);
        }
        SmartDashboard.putNumber("poseestimator_status", 0);

        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
        drivetrain.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds);
    }

    @Override
    public void periodic() {
        for (String limelight : LIMELIGHTS) {
            LimelightHelpers.SetRobotOrientation(limelight,
                    this.drivetrain.getEstimatedPosition().getRotation().getDegrees(), 0, 0,
                    0, 0, 0);

            updateVisionMeasurement(limelight);
        }
    }
}
