package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.CoralConstants;
import frc.robot.util.TunableNumber;

public class CoralSubsystem extends SubsystemBase {
    private final SparkFlex vortexManipTop = new SparkFlex(Constants.CORAL_MOTOR_BOTTOM_ID, MotorType.kBrushless);
    private final SparkFlex vortexManipBottom = new SparkFlex(Constants.CORAL_MOTOR_TOP_ID, MotorType.kBrushless);

    private final SparkMax neoFlipper = new SparkMax(Constants.CORAL_EJECTOR_ID, MotorType.kBrushless);

    private final DigitalInput beam = new DigitalInput(Constants.INTAKE_BEAM_PIN);

    private final TalonFX falconFeeder = new TalonFX(Constants.FEEDER_FALCON_ID);

    private TunableNumber troughTopVolts = new TunableNumber("coral", "out_top_volts", CoralConstants.TROUGH_TOP_VOLTS);
    private TunableNumber troughBottomVolts = new TunableNumber("coral", "out_bottom_volts",
            CoralConstants.TROUGH_BOTTOM_VOLTS);

    private TunableNumber inTopVolts = new TunableNumber("coral", "in_top_volts", CoralConstants.INTAKE_TOP_VOLTS);
    private TunableNumber inBottomVolts = new TunableNumber("coral", "in_bottom_volts",
            CoralConstants.INTAKE_BOTTOM_VOLTS);

    private TunableNumber flipperVolts = new TunableNumber("coral", "flipper_volts", CoralConstants.FLIPPER_VOLTS);

    private TunableNumber feedVolts = new TunableNumber("feed", "feed_volts", CoralConstants.FEEDER_VOLTS);
    private TunableNumber unfeedVolts = new TunableNumber("unfeed", "unfeed_volts", CoralConstants.UNFEED_VOLTS);

    public CoralSubsystem() {
        vortexManipTop.configure(
                new SparkFlexConfig().inverted(false).idleMode(IdleMode.kBrake),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        vortexManipBottom.configure(
                new SparkFlexConfig().inverted(true).idleMode(IdleMode.kBrake),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        neoFlipper.configure(
                new SparkMaxConfig().idleMode(IdleMode.kBrake),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        falconFeeder.getConfigurator()
                .apply(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.Clockwise_Positive));
    }

    private void intakeTop() {
        vortexManipTop.setVoltage(inTopVolts.get());
    }

    private void intakeBottom() {
        vortexManipBottom.setVoltage(inBottomVolts.get());
    }

    private void outtakeTop() {
        vortexManipTop.setVoltage(-inTopVolts.get());
    }

    private void outtakeBottom() {
        vortexManipBottom.setVoltage(-inBottomVolts.get());
    }

    private void troughTop() {
        vortexManipTop.setVoltage(troughTopVolts.get());
    }

    private void troughBottom() {
        vortexManipBottom.setVoltage(troughBottomVolts.get());
    }

    private void branchTop() {
        vortexManipTop.setVoltage(CoralConstants.BRANCH_TOP_VOLTS);
    }

    private void branchBottom() {
        vortexManipBottom.setVoltage(CoralConstants.BRANCH_BOTTOM_VOLTS);
    }

    private void stopManipulator() {
        vortexManipTop.setVoltage(0);
        vortexManipBottom.setVoltage(0);
    }

    private void flipOut() {
        neoFlipper.setVoltage(flipperVolts.get());
    }

    private void flipperResetPose() {
        neoFlipper.setVoltage(-flipperVolts.get());
    }

    private void stopFlipper() {
        neoFlipper.setVoltage(0);
    }

    public void feed() {
        falconFeeder.setVoltage(feedVolts.get());
    }

    public void unfeed() {
        falconFeeder.setVoltage(unfeedVolts.get());
    }

    public void stopFeed() {
        falconFeeder.setVoltage(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("intakeSwitchStatus", getSwitchStatus());
    }

    public boolean getSwitchStatus() {
        return beam.get();
    }

    public Trigger beamBreakEngaged() {
        return new Trigger(() -> !getSwitchStatus());
    }

    // COMMANDS:

    public Command simpleFeedOutCommand() {
        return startEnd(() -> feed(), () -> stopFeed());
    }

    public Command feedStageCommand() {
        return run(() -> feed())
                .withDeadline(new WaitCommand(CoralConstants.FEED_DELAY_SECONDS))
                .finallyDo(() -> stopFeed());
    }

    public Command unfeedCommand() {
        return startEnd(() -> {
            outtakeTop();
            outtakeBottom();
            unfeed();
        }, () -> {
            stopManipulator();
            stopFeed();
        });
    }

    public Command simpleManipCommand() {
        return startEnd(() -> {
            intakeTop();
            intakeBottom();
        }, () -> stopManipulator());
    }

    public Command stopManipCommand() {
        return runOnce(() -> stopManipulator());
    }

    public Command flipOutCommand() {
        return startEnd(() -> flipOut(), () -> stopFlipper());
    }

    public Command flipperResetCommand() {
        return startEnd(() -> flipperResetPose(), () -> stopFlipper());
    }

    public Command troughCommand() {
        return startEnd(() -> {
            troughTop();
            troughBottom();
        }, () -> stopManipulator());
    }

    public Command branchCommand() {
        return startEnd(() -> {
            branchTop();
            branchBottom();
        }, () -> stopManipulator());
    }

    public Command l4BranchCommand() {
        return new SequentialCommandGroup(branchCommand(),
                Commands.waitTime(CoralConstants.L4_EJECTION_TIMEOUT), startEnd(() -> {
                    intakeTop();
                    intakeBottom();
                    flipOut();
                }, () -> {
                    stopFlipper();
                    stopManipulator();
                }).withTimeout(CoralConstants.L4_EJECTION_TIMEOUT),
                new WaitCommand(CoralConstants.FLIPPER_RESET_DELAY),
                flipperResetCommand().withTimeout(CoralConstants.FLIPPER_RESET_TIMOUT));
    } // TODO test flipper times OR create a setpoint to return to

    public Command outputBasedOnLevel(ElevatorSubsystem elevator) {
        switch (elevator.getActiveLevel()) {
            case L1:
                return troughCommand().withTimeout(CoralConstants.TROUGH_TIMEOUT);
            case L2:
            case L3:
                return branchCommand().withTimeout(CoralConstants.BRANCH_TIMEOUT);
            case L4:
                return l4BranchCommand();
            case BASE:
            default:
                return Commands.none();
        }
    }
    // public Command feedWithBeamCommand() {
    // return startRun(() -> {
    // feed();
    // // intakeTop();
    // // intakeBottom();
    // }, () -> {
    // }).until(beamBreakEngaged().negate()).finallyDo(() -> {
    // stopFeed();
    // // stopManipulator();
    // });
    // }

}
