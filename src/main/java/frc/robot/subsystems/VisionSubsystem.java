// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.NTDoubleSection;

public class VisionSubsystem extends SubsystemBase {
    DrivetrainSubsystem drivetrain;
    private final StructPublisher<Pose2d> limelightPosePublisher = NetworkTableInstance.getDefault()
            .getTable("telemetry").getStructTopic("pose estimation", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> limelightMt2Publisher = NetworkTableInstance.getDefault()
            .getTable("telemetry").getStructTopic("mt2", Pose2d.struct).publish();
    @SuppressWarnings("unused")
    private double driftEstimateTicks;

    // Initializes the vision subsystem
    public VisionSubsystem(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
    }

    LinearFilter mt1RzAverage = LinearFilter.movingAverage(VisionConstants.AVERAGING_WINDOW);
    double lastRot = Double.NaN;
    double curRot;
    int ticksWithNoTv = 0;

    public void seedPigeon() {
        drivetrain.getPigeon2().setYaw(curRot);
    }

    public Command getSeedPigeon() {
        return runOnce(() -> {
            double[] inputBuffer = new double[VisionConstants.AVERAGING_WINDOW];
            Arrays.fill(inputBuffer, drivetrain.getEstimatedPosition().getRotation().getDegrees());
            mt1RzAverage.reset(inputBuffer, new double[0]);
        }).andThen(run(() -> seedPigeon()).ignoringDisable(true));
    }

    private Pose2d updateVisionMeasurement() {
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (mt2 == null) {
            // failed to get mt2
            SmartDashboard.putNumber("poseestimator_status", -1);
            return null;
        }
        if (mt2.tagCount == 0) {
            // no tags
            SmartDashboard.putNumber("poseestimator_status", -2);
            return null;
        }
        SmartDashboard.putNumber("poseestimator_status", 0);

        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
        drivetrain.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds);

        return mt2.pose;
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
        LimelightHelpers.SetRobotOrientation(null,
                this.drivetrain.getEstimatedPosition().getRotation().getDegrees(), 0, 0,
                0, 0, 0);

        Pose2d mt2 = updateVisionMeasurement();
        if (mt2 != null) {
            this.limelightMt2Publisher.set(mt2);
        }
        this.limelightPosePublisher.set(drivetrain.getEstimatedPosition());

        telemetry_doubles.set("mt1_rz", LimelightHelpers.getBotPose2d_wpiBlue(null).getRotation().getDegrees());
        telemetry_doubles.set("mt2_rz",
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(null).pose.getRotation().getDegrees());
        telemetry_doubles.set("pigeon", drivetrain.getPigeon2().getRotation2d().getDegrees());
        telemetry_doubles.set("poseest", drivetrain.getEstimatedPosition().getRotation().getDegrees());

        addMt1Reading();
    }
}
