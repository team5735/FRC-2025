// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.vision.AlignToReef;
import frc.robot.constants.Constants;
import frc.robot.constants.CoralConstants;
import frc.robot.constants.ElevatorConstants.Level;
import frc.robot.constants.drivetrain.CompbotTunerConstants;
import frc.robot.constants.drivetrain.DevbotTunerConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.ReefAlignment;

public class RobotContainer {
    private final double MAX_SPEED = CompbotTunerConstants.SPEED_AT_12_VOLTS.in(MetersPerSecond);

    private final Telemetry logger = new Telemetry(MAX_SPEED);

    private final SendableChooser<Command> autoChooser;

    private final CommandXboxController driveController = new CommandXboxController(Constants.DRIVE_CONTROLLER_PORT);
    private final CommandXboxController subsystemController = new CommandXboxController(
            Constants.SUBSYSTEM_CONTROLLER_PORT);

    private final CommandXboxController testController = new CommandXboxController(2);

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

    public static final AlgaeSubsystem algaer = new AlgaeSubsystem();
    public static final CoralSubsystem coraler = new CoralSubsystem();
    public static final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public static final CANdleSubsystem LEDs = new CANdleSubsystem();

    public RobotContainer() {
        // CameraServer.startAutomaticCapture();
        Map<String, Command> commandsForAuto = new HashMap<>();

        commandsForAuto.put("l1AndScore", elevator.toLevelAndCoral(Level.L1, coraler));
        commandsForAuto.put("l2AndScore", elevator.toLevelAndCoral(Level.L2, coraler));
        commandsForAuto.put("l3AndScore", elevator.toLevelAndCoral(Level.L3, coraler));
        commandsForAuto.put("l4AndScore", elevator.toLevelAndCoral(Level.L4, coraler));

        commandsForAuto.put("elevatorBase", elevator.toLevelCommand(Level.BASE));
        commandsForAuto.put("l1", elevator.toLevelCommand(Level.L1));
        commandsForAuto.put("l2", elevator.toLevelCommand(Level.L2));
        commandsForAuto.put("l3", elevator.toLevelCommand(Level.L3));
        commandsForAuto.put("l4", elevator.toLevelCommand(Level.L4));

        commandsForAuto.put("troughScore", coraler.troughCommand().withTimeout(CoralConstants.TROUGH_TIMEOUT));
        commandsForAuto.put("branchScore", coraler.branchCommand().withTimeout(CoralConstants.BRANCH_TIMEOUT));
        commandsForAuto.put("l4Score", coraler.l4BranchCommand());

        commandsForAuto.put("intake", coraler.feedStageCommand());
        commandsForAuto.put("backTrough", coraler.unfeedCommand().withTimeout(Seconds.of(1.5)));

        NamedCommands.registerCommands(commandsForAuto);

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
                        () -> driveController.getRightTriggerAxis(),
                        () -> driveController.getHID().getBButton()));

        drivetrain.registerTelemetry(logger::telemeterize);

        // driveController.a().whileTrue(drivetrain.brakeCommand()); // also used for
        // branch scoring
        driveController.rightStick().whileTrue(coraler.branchCommand());
        // drivecontroller.b() slow mode and driving forward
        driveController.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        driveController.x().whileTrue(drivetrain.brakeCommand());

        driveController.leftBumper().whileTrue(coraler.unfeedCommand());
        driveController.rightBumper().onTrue(coraler.feedStageCommand());

        driveController.povRight().and(driveController.a()).onTrue(elevator.toLevelCommand(Level.L1));
        driveController.povDown().and(driveController.a()).onTrue(elevator.toLevelCommand(Level.L2));
        driveController.povLeft().and(driveController.a()).onTrue(elevator.toLevelCommand(Level.L3));
        // driveController.povUp().and(driveController.a()).onTrue(elevator.toLevelCommand(Level.L4));

        driveController.back().onTrue(Commands.runOnce(() -> elevator.resetMeasurement()));

        driveController.start().whileTrue(Commands.runOnce(() -> elevator.swapEnableStatus()));

        coraler.beamBreakEngaged().whileTrue(LEDs.colorFedCommand());

        driveController.povDown().and(driveController.a().negate()).onTrue(elevator.toLevelCommand(Level.BASE));
        // driveController.povLeft().and(driveController.a().negate())
        // .whileTrue(new AlignToReef(drivetrain, vision, ReefAlignment.LEFT, () ->
        // movingForward));
        // driveController.povRight().and(driveController.a().negate())
        // .whileTrue(new AlignToReef(drivetrain, vision, ReefAlignment.RIGHT, () ->
        // movingForward));
        // driveController.povUp().and(driveController.a().negate()).onTrue(vision.getSeedPigeon());

        driveController.b().whileTrue(new FunctionalCommand(() -> {
            movingForward = true;
        }, () -> {
        }, (cancelled) -> {
            movingForward = false;
        }, () -> false));

        // reset the field-centric heading on left bumper press

        // Coral manipulator temporary testing bindings
        subsystemController.a().whileTrue(coraler.simpleManipCommand());
        // subsystemController.b().whileTrue(coraler.branchCommand());
        subsystemController.x().whileTrue(coraler.l4BranchCommand());
        subsystemController.y().whileTrue(coraler.troughCommand());

        subsystemController.leftBumper().whileTrue(coraler.flipOutCommand());
        subsystemController.rightBumper().whileTrue(coraler.flipperResetCommand());

        subsystemController.povUp().whileTrue(elevator.manualElevatorUp());
        subsystemController.povDown().whileTrue(elevator.manualElevatorDown());

        // testController.a().whileTrue(elevator.toLevelCommand(Level.SMARTDASHBOARD));
        testController.start().onTrue(Commands.runOnce(() -> elevator.resetMeasurement()));
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
