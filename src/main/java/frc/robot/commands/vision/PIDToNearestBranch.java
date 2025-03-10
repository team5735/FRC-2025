package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import java.util.function.Supplier;

public class PIDToNearestBranch extends Command {

    private DrivetrainSubsystem drivetrain;
    private VisionSubsystem vision;
    private Supplier<Translation2d> targetSupplier;

    public PIDToNearestBranch(DrivetrainSubsystem drivetrain, VisionSubsystem vision,
            Supplier<Translation2d> targetSupplier) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.targetSupplier = targetSupplier;
        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
    }
}
