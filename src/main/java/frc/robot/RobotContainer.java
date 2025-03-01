// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.ElevatorConstants.Level;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
    public RobotContainer() {
        configureBindings();
    }

    private final CommandXboxController testController = new CommandXboxController(0);
    private final ElevatorSubsystem elevator = new ElevatorSubsystem();

    private void configureBindings() {
        testController.povUp().onTrue(elevator.toLevelCommand(Level.SMARTDASHBOARD));
        testController.povDown().onTrue(elevator.toLevelCommand(Level.BASE));
        testController.start().onTrue(Commands.runOnce(() -> elevator.resetMeasurement()));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
