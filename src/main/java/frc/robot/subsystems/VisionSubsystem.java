// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private static final String LIMELIGHTS[] = { "limelight", "limelight_back" };

    // Initializes the vision subsystem
    public VisionSubsystem(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
    }

    LinearFilter mt1RzAverage = LinearFilter.movingAverage(VisionConstants.AVERAGING_WINDOW);
    double lastRot = Double.NaN;
    double curRot;
    int ticksWithNoTv = 0;

    public void seedPigeon() {
        if (!LimelightHelpers.getTV("limelight")) {
            return;
        }
        drivetrain.getPigeon2().setYaw(curRot);
        drivetrain.resetPose(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight").pose);
    }

    public Command getSeedPigeon() {
        return run(() -> seedPigeon()).until(() -> LimelightHelpers.getTV("limelight"));
    }

    private void updateVisionMeasurement(String limelight_name) {
        LimelightHelpers.SetRobotOrientation(limelight_name,
                drivetrain.getPigeon2().getRotation2d().getDegrees(), 0, 0,
                0, 0, 0);

        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight_name);
        if (mt1 == null) {
            // failed to get mt2 or it's not new
            SmartDashboard.putNumber("poseestimator_status", -1);
            return;
        } else if (mt1.tagCount == 0) {
            // no tags
            SmartDashboard.putNumber("poseestimator_status", -2);
            return;
        } else
            SmartDashboard.putNumber("poseestimator_status", 0);

        double[] stddevs = NetworkTableInstance.getDefault().getTable(limelight_name).getEntry("stddevs")
                .getDoubleArray(new double[12]);
        double mt2xdev = stddevs[6];
        double mt2ydev = stddevs[7];
        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(mt2xdev, mt2ydev, 9999999));
        drivetrain.addVisionMeasurement(
                mt1.pose,
                mt1.timestampSeconds);
    }

    private void addMt1Reading() {
        if (!LimelightHelpers.getTV(null)) {
            ticksWithNoTv++;
            if (ticksWithNoTv > 5) {
                double[] inputBuffer = new double[VisionConstants.AVERAGING_WINDOW];
                Arrays.fill(inputBuffer, drivetrain.getEstimatedPosition().getRotation().getDegrees());
                mt1RzAverage.reset(inputBuffer, new double[0]);
            }
            return;
        }

        ticksWithNoTv = 0;
        double thisRot = LimelightHelpers.getBotPose2d_wpiBlue(null).getRotation().getDegrees();
        if (thisRot == lastRot) {
            return;
        }
        lastRot = thisRot;
        curRot = mt1RzAverage.calculate(thisRot);
        telemetry_doubles.set("averagedMt1", curRot);
    }

    // all in deg
    NTDoubleSection telemetry_doubles = new NTDoubleSection("test_telem_doubles", "mt1_rz", "mt2_rz", "pigeon",
            "poseest", "averagedMt1");

    @Override
    public void periodic() {
        for (String limelight : LIMELIGHTS) {
            updateVisionMeasurement(limelight);
        }

        addMt1Reading();
    }

    public void scheduleWaitForApriltagCommand() {
        Commands.idle(this) // do nothing
                .until(() -> { // until any limelight sees a tag
                    return Arrays.stream(LIMELIGHTS).anyMatch(limelight -> LimelightHelpers.getTV(limelight));
                })
                .andThen(runOnce(() -> { // and then reset the robot pose
                    for (String limelight : LIMELIGHTS) {
                        if (LimelightHelpers.getTV(limelight)) {
                            drivetrain.resetPose(LimelightHelpers.getBotPose2d(limelight));
                            System.out.println("Robot pose set to mt1 report from " + limelight);
                            break;
                        }
                    }
                }))
                .schedule();
    }
}
