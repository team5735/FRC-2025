// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ClimberSubsystem;

public class RobotContainer {
    private CommandXboxController driveController = new CommandXboxController(0);

    private ClimberSubsystem climber = new ClimberSubsystem();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        driveController.a().whileTrue(climber.climbCommand());
        driveController.b().whileTrue(climber.outCommand());
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
