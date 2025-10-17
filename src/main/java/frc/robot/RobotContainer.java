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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.vision.AlignToReef;
import frc.robot.commands.vision.DriveToBranch;
import frc.robot.constants.Constants;
import frc.robot.constants.CoralConstants;
import frc.robot.constants.ElevatorConstants.Height;
import frc.robot.constants.drivetrain.CompbotTunerConstants;
import frc.robot.constants.drivetrain.DevbotTunerConstants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.util.ReefAlignment;

public class RobotContainer {
    private final double MAX_SPEED = CompbotTunerConstants.SPEED_AT_12_VOLTS.in(MetersPerSecond);

    private final Telemetry logger = new Telemetry(MAX_SPEED);

    private final SendableChooser<Command> autoChooser;

    private final CommandXboxController driveController = new CommandXboxController(Constants.DRIVE_CONTROLLER_PORT);
    private final CommandXboxController subsystemController = new CommandXboxController(
            Constants.SUBSYSTEM_CONTROLLER_PORT);

    private final CommandXboxController testController = new CommandXboxController(2);

    public static final Drivetrain drivetrain;

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
    public static final Vision vision = new Vision(drivetrain);

    public static final Algae algaer = new Algae();
    public static final Coral coraler = new Coral();
    public static final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public static final CANdleSubsystem LEDs = new CANdleSubsystem();

    public RobotContainer() {
        Map<String, Command> commandsForAuto = new HashMap<>();

        commandsForAuto.put("l1AndScore", elevator.getSetLevelAndCoral(Height.L1, coraler));
        commandsForAuto.put("l2AndScore", elevator.getSetLevelAndCoral(Height.L2, coraler));
        commandsForAuto.put("l3AndScore", elevator.getSetLevelAndCoral(Height.L3, coraler));
        commandsForAuto.put("l4AndScore", elevator.getSetLevelAndCoral(Height.L4, coraler));

        commandsForAuto.put("elevatorBase", elevator.getSetLevel(Height.BASE));
        commandsForAuto.put("l1", elevator.getSetLevel(Height.L1));
        commandsForAuto.put("l2", elevator.getSetLevel(Height.L2));
        commandsForAuto.put("l3", elevator.getSetLevel(Height.L3));
        commandsForAuto.put("l4", elevator.getSetLevel(Height.L4));

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
        configureBindings();
    }

    private ReefAlignment chooseAlignment() {
        switch (driveController.getHID().getPOV()) {
            case 270:
                return ReefAlignment.LEFT;
            case 90:
                return ReefAlignment.RIGHT;
            default:
                return ReefAlignment.ALGAE;
        }
    }

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

        // also used for branch scoring
        driveController.leftStick().onTrue(elevator.getSetLevel(Height.BASE));
        driveController.rightStick().whileTrue(coraler.branchCommand());

        driveController.povRight().and(driveController.a()).onTrue(elevator.getSetLevel(Height.L1));
        driveController.povDown().and(driveController.a()).onTrue(elevator.getSetLevel(Height.L2));
        driveController.povLeft().and(driveController.a()).onTrue(elevator.getSetLevel(Height.L3));
        driveController.povUp().and(driveController.a()).onTrue(elevator.getSetLevel(Height.L4));
        // drivecontroller.b() slow mode

        Command pathPlannerDrive = new DriveToBranch(drivetrain, () -> chooseAlignment());
        driveController.x().onTrue(pathPlannerDrive);
        driveController.y().and(driveController.pov(-1)) // pov -1 is unpressed
                .onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driveController.povDown().and(driveController.y()).onTrue(elevator.getSetLevel(Height.BASE));
        driveController.povLeft().and(driveController.y())
                .whileTrue(new AlignToReef(drivetrain, vision));
        driveController.povRight().and(driveController.y())
                .whileTrue(new AlignToReef(drivetrain, vision));
        driveController.povUp().and(driveController.y()).onTrue(vision.getSeedPigeon());

        driveController.leftBumper().whileTrue(coraler.unfeedCommand());
        driveController.rightBumper().onTrue(coraler.feedStageCommand());

        coraler.beamBreakEngaged().whileTrue(LEDs.colorFedCommand());

        // Coral manipulator temporary testing bindings
        subsystemController.a().whileTrue(coraler.simpleManipCommand());
        // subsystemController.b().whileTrue(coraler.branchCommand());
        // subsystemController.x().whileTrue(coraler.l4BranchCommand());
        subsystemController.y().whileTrue(coraler.troughCommand());

        subsystemController.leftBumper().whileTrue(coraler.flipOutCommand());
        subsystemController.rightBumper().whileTrue(coraler.flipperResetCommand());

        subsystemController.rightTrigger(0.1).or(subsystemController.leftTrigger(0.1))
                .whileTrue(LEDs.colorAngryCommand());

        subsystemController.povUp().whileTrue(elevator.getSetSpeed(1));
        subsystemController.povDown().whileTrue(elevator.getSetSpeed(-1));

        testController.a().whileTrue(elevator.sysId());
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
