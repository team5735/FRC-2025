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
import frc.robot.constants.drivetrain.CompbotTunerConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RobotContainer {
    private double MaxSpeed = CompbotTunerConstants.SPEED_AT12_VOLTS.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                          // speed

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final SendableChooser<Command> autoChooser;

    private final CommandXboxController driverController = new CommandXboxController(0);

    public static final DrivetrainSubsystem drivetrain = CompbotTunerConstants.createDrivetrain();

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
                drivetrain.joystickDriveCommand(
                        () -> driverController.getLeftX(),
                        () -> driverController.getLeftY(),
                        () -> driverController.getLeftTriggerAxis(),
                        () -> driverController.getRightTriggerAxis()));

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(
                        new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

        driverController.povDown().onTrue(
                Commands.runOnce(() -> {
                    drivetrain.seedFieldCentric();
                    drivetrain.getPigeon2().setYaw(0);
                }, drivetrain));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
        driverController.a().whileTrue(algaer.grabStopCommand());
        driverController.b().whileTrue(algaer.spitStopCommand());
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
