// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.Constants;
import frc.robot.commands.vision.AlignToReef;
import frc.robot.commands.vision.DriveToBranch;
import frc.robot.constants.drivetrain.CompbotTunerConstants;
import frc.robot.constants.drivetrain.DevbotTunerConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.ReefAlignment;

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

    private AlgaeSubsystem algaer = new AlgaeSubsystem();
    private CoralSubsystem coraler = new CoralSubsystem();
    private CANdleSubsystem LEDs = new CANdleSubsystem();

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Choose an Auto", autoChooser);
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                // both joysticks are combined here so that you only need one at a time to test
                // anything.
                drivetrain.joystickDriveCommand(
                        () -> driveController.getLeftX(),
                        () -> driveController.getLeftY(),
                        () -> driveController.getLeftTriggerAxis(),
                        () -> driveController.getRightTriggerAxis()));

        driveController.a().whileTrue(drivetrain.brakeCommand());
        driveController.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(
                        new Rotation2d(-driveController.getLeftY(), -driveController.getLeftX()))));

        // driveController.x().whileTrue(new AlignToReef(drivetrain, vision, () ->
        // ReefAlignment.ALGAE));
        // driveController.y().onTrue(Commands.runOnce(() -> {
        // vision.seedPigeon();
        // }));

        // reset the field-centric heading on left bumper press
        driveController.povUp().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driveController.back().and(driveController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driveController.back().and(driveController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driveController.start().and(driveController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driveController.start().and(driveController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // subsystemController.rightBumper().whileTrue(algaer.grabStopCommand());

        /*
         * subsystemController.a().whileTrue(coraler.simpleFeedCommand());
         * subsystemController.b().whileTrue(coraler.outtakeCommand());
         * subsystemController.x().whileTrue(coraler.troughCommand());
         * subsystemController.y().onTrue(coraler.branchCommand());
         */
        driveController.leftBumper().onTrue(LEDs.colorAimedCommand());
        driveController.rightBumper().onTrue(LEDs.colorReadyCommand());

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