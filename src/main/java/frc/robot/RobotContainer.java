// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
    public RobotContainer() {
        configureBindings();
    }

    private final CommandXboxController testController = new CommandXboxController(0);
    private final ElevatorSubsystem elevator = new ElevatorSubsystem();

    private void configureBindings() {
        testController.a().whileTrue(elevator.sysIdDynamic(Direction.kForward));
        testController.b().whileTrue(elevator.sysIdDynamic(Direction.kReverse));
        testController.x().whileTrue(elevator.sysIdQuasistatic(Direction.kForward));
        testController.y().whileTrue(elevator.sysIdQuasistatic(Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
