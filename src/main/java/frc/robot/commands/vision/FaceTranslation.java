package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.TunableNumber;
import frc.robot.util.TunablePIDController;

public class FaceTranslation extends Command {
    private DrivetrainSubsystem drivetrain;
    private TunablePIDController controller = new TunablePIDController("face_translation");
    private TunableNumber target_x = new TunableNumber("face_translation", "x");
    private TunableNumber target_y = new TunableNumber("face_translation", "y");

    public FaceTranslation(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        Translation2d target = new Translation2d(target_x.get(), target_y.get());
        Translation2d currentPos = drivetrain.getEstimatedPosition().getTranslation();
        Translation2d botToTarget = target.minus(currentPos);
        Rotation2d angle = new Rotation2d(botToTarget.getX(), botToTarget.getY());
        controller.setup(angle.getRadians());
    }

    @Override
    public void execute() {
        double omega = controller.calculate(drivetrain.getEstimatedPosition().getRotation().getRadians());
        drivetrain.pidDrive(new Translation2d(), omega);
    }

    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }
}
