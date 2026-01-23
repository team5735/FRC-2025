// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drivetrain.PIDToPose;
import frc.robot.constants.Constants;
import frc.robot.constants.drivetrain.CompbotTunerConstants;
import frc.robot.constants.drivetrain.DevbotTunerConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.Arc;
import frc.robot.util.TunableNumber;

public class RobotContainer {
    private final double MAX_SPEED = CompbotTunerConstants.SPEED_AT_12_VOLTS.in(MetersPerSecond);

    private final Telemetry logger = new Telemetry(MAX_SPEED);

    private final CommandXboxController driveController = new CommandXboxController(Constants.DRIVE_CONTROLLER_PORT);

    public static final DrivetrainSubsystem drivetrain;

    static {
        switch (Constants.DRIVETRAIN_TYPE) {
            case DEVBOT:
                drivetrain = DevbotTunerConstants.createDrivetrain();
                break;
            case COMPBOT:
            default:
                drivetrain = CompbotTunerConstants.createDrivetrain();
                break;
        }
    }
    public static final VisionSubsystem vision = new VisionSubsystem(drivetrain);

    public RobotContainer() {
        PathfindingCommand.warmupCommand().schedule();
        DriverStation.silenceJoystickConnectionWarning(true);
        configureBindings();
    }

    Arc hubArcModel = new Arc(new Translation2d(), 0, new Rotation2d(), new Rotation2d());

    TunableNumber testPIDFaceX = new TunableNumber("test pid_face_x", 2);
    TunableNumber testPIDFaceY = new TunableNumber("test pid_face_y", 2);

    TunableNumber testPIDToX = new TunableNumber("test pid_to_x", 2);
    TunableNumber testPIDToY = new TunableNumber("test pid_to_y", 2);
    TunableNumber testPIDToRotation = new TunableNumber("test pid_to_rotation", 2);

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.joystickDriveCommand(
                        () -> driveController.getLeftX(),
                        () -> driveController.getLeftY(),
                        () -> driveController.getLeftTriggerAxis(),
                        () -> driveController.getRightTriggerAxis(),
                        () -> driveController.getHID().getBButton()));

        drivetrain.registerTelemetry(logger::telemeterize);

        driveController.a()
                .onTrue(new PIDToPose(drivetrain, () -> {
                    Translation2d currentPose = drivetrain.getEstimatedPosition().getTranslation();
                    Rotation2d targetAngle = new Translation2d(testPIDFaceX.get(), testPIDFaceY.get())
                            .minus(currentPose)
                            .getAngle();
                    return new Pose2d(currentPose, targetAngle);
                }, "face a translation in-place"));

        driveController.b()
                .onTrue(new PIDToPose(drivetrain, () -> {
                    Translation2d destinationTrans = hubArcModel
                            .nearestPointOnArc(drivetrain.getEstimatedPosition().getTranslation());
                    Rotation2d angleToArcCenter = hubArcModel.getCenter().minus(destinationTrans).getAngle();
                    return new Pose2d(destinationTrans, angleToArcCenter);
                }, "drive to arc"));

        driveController.x()
                .onTrue(new PIDToPose(drivetrain,
                        () -> new Pose2d(testPIDToX.get(), testPIDToY.get(),
                                Rotation2d.fromDegrees(testPIDToRotation.get())),
                        "pid to pose"));

        driveController.y().onTrue(vision.runOnce(() -> vision.seedPigeon()));

        driveController.povUp().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
