package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveToArc extends Command {
    private DrivetrainSubsystem drivetrain;

    public DriveToArc(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }
}
