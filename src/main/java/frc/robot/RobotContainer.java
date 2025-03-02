// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.vision.AlignToReef;
import frc.robot.commands.vision.DriveToBranch;
import frc.robot.constants.Constants;
import frc.robot.constants.drivetrain.CompbotTunerConstants;
import frc.robot.constants.drivetrain.DevbotTunerConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.ReefAlignment;

public class RobotContainer {
    private final double MAX_SPEED = CompbotTunerConstants.SPEED_AT_12_VOLTS.in(MetersPerSecond);

    private final Telemetry logger = new Telemetry(MAX_SPEED);

    private final SendableChooser<Command> autoChooser;

    private final CommandXboxController driveController = new CommandXboxController(Constants.DRIVE_CONTROLLER_PORT);
    private final CommandXboxController subsystemController = new CommandXboxController(
            Constants.SUBSYSTEM_CONTROLLER_PORT);

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
    private static final VisionSubsystem vision = new VisionSubsystem(drivetrain);

    private AlgaeSubsystem algaer = new AlgaeSubsystem();
    private CoralSubsystem coraler = new CoralSubsystem();

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Choose an Auto", autoChooser);
        PathfindingCommand.warmupCommand().schedule();
        DriverStation.silenceJoystickConnectionWarning(true);
        vision.scheduleWaitForApriltagCommand();
        configureBindings();
    }

    private boolean movingForward = false;

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.joystickDriveCommand(
                        () -> driveController.getLeftX(),
                        () -> driveController.getLeftY(),
                        () -> driveController.getLeftTriggerAxis(),
                        () -> driveController.getRightTriggerAxis()));

        drivetrain.registerTelemetry(logger::telemeterize);

        System.out.println(ReefAlignment.LEFT.getParallel().in(Meters));
        System.out.println(ReefAlignment.RIGHT.getParallel().in(Meters));
        driveController.x()
                .whileTrue(new AlignToReef(drivetrain, vision, ReefAlignment.LEFT, () -> movingForward));
        driveController.a()
                .whileTrue(new AlignToReef(drivetrain, vision, ReefAlignment.RIGHT, () -> movingForward));
        driveController.y().onTrue(Commands.runOnce(() -> {
            vision.seedPigeon();
        }));

        driveController.b().whileTrue(new FunctionalCommand(() -> {
            movingForward = true;
        }, () -> {
        }, (cancelled) -> {
            movingForward = false;
        }, () -> false));

        // reset the field-centric heading on left bumper press
        driveController.povUp().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driveController.back().and(driveController.a()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driveController.back().and(driveController.b()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driveController.start().and(driveController.a()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driveController.start().and(driveController.b()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        subsystemController.rightBumper().whileTrue(algaer.grabStopCommand());

        subsystemController.a().whileTrue(coraler.simpleFeedCommand());
        subsystemController.b().whileTrue(coraler.outtakeCommand());
        subsystemController.x().whileTrue(coraler.troughCommand());
        subsystemController.y().onTrue(coraler.branchCommand());

        driveController.povDown().whileTrue(
                new DriveToBranch(drivetrain, () -> ReefAlignment.LEFT));
        driveController.povLeft().whileTrue(
                new DriveToBranch(drivetrain, () -> ReefAlignment.LEFT));
        driveController.povRight().whileTrue(
                new DriveToBranch(drivetrain, () -> ReefAlignment.RIGHT));
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
