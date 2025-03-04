// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;
import frc.robot.constants.drivetrain.CompbotTunerConstants;
import frc.robot.constants.drivetrain.DevbotTunerConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
    private final double MAX_SPEED = CompbotTunerConstants.SPEED_AT_12_VOLTS.in(MetersPerSecond);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MAX_SPEED);

    private final SendableChooser<Command> autoChooser;

    private final CommandXboxController driveController = new CommandXboxController(Constants.DRIVE_CONTROLLER_PORT);
    private final CommandXboxController subsystemController = new CommandXboxController(
            Constants.SUBSYSTEM_CONTROLLER_PORT);

    public static final DrivetrainSubsystem drivetrain;

    static {
        switch (Constants.ROBOT_DRIVETRAIN) {
            case DEVBOT:
                drivetrain = DevbotTunerConstants.createDrivetrain();
                break;
            case COMPBOT:
            default:
                drivetrain = CompbotTunerConstants.createDrivetrain();
                System.out.println("Making proper drivetrain");
                break;
        }
    }
    // private static final VisionSubsystem vision = new
    // VisionSubsystem(drivetrain);

    public static final AlgaeSubsystem algaer = new AlgaeSubsystem();
    public static final CoralSubsystem coraler = new CoralSubsystem();
    public static final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public static final CANdleSubsystem LEDs = new CANdleSubsystem();

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Choose an Auto", autoChooser);
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                drivetrain.joystickDriveCommand(
                        () -> driveController.getLeftX(),
                        () -> driveController.getLeftY(),
                        () -> driveController.getLeftTriggerAxis(),
                        () -> driveController.getRightTriggerAxis(),
                        () -> {
                            if (driveController.getHID().getBButton()) {
                                return DrivetrainSubsystem.CONSTANTS.getSlowMultiplier();
                            }
                            return 1.0;
                        }));

        drivetrain.registerTelemetry(logger::telemeterize);

        driveController.a().whileTrue(drivetrain.brakeCommand()); // also used for branch scoring
        // drivecontroller.b() slow mode
        driveController.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driveController.leftBumper().whileTrue(coraler.feedStageCommand());
        driveController.rightBumper().whileTrue(coraler.outtakeCommand());

        // driveController.x().whileTrue(new AlignToReef(drivetrain, vision, () ->
        // ReefAlignment.ALGAE));
        // driveController.y().onTrue(Commands.runOnce(() -> {
        // vision.seedPigeon();
        // }));

        // reset the field-centric heading on left bumper press

        // Coral manipulator temporary testing bindings
        subsystemController.a().whileTrue(coraler.simpleManipCommand());
        subsystemController.b().whileTrue(coraler.simpleFeedCommand());
        // subsystemController.x().whileTrue(coraler.feedToManipCommand());
        subsystemController.x().whileTrue(coraler.feedStageCommand());

        subsystemController.y().whileTrue(coraler.l4BranchCommand());

        // TODO test feed delay
        subsystemController.leftBumper().whileTrue(coraler.simpleEjectOutCommand());
        subsystemController.rightBumper().whileTrue(coraler.simpleEjectResetCommand());

        // driveController.povDown().whileTrue(new DriveToBranch(drivetrain, () ->
        // ReefAlignment.ALGAE));
    }

    public Command getAutonomousCommand() {
        Command auto = autoChooser.getSelected();
        if (auto == null) {
            System.out.println("auto is null");
            return drivetrain.brakeCommand();
        }

        return auto;
    }
}
