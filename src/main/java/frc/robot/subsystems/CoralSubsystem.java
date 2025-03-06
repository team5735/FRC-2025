package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    private TunableNumber outTopVolts = new TunableNumber("coral", "out_top_volts", CoralConstants.TROUGH_TOP_VOLTS);
    private TunableNumber outBottomVolts = new TunableNumber("coral", "out_bottom_volts",
            CoralConstants.TROUGH_BOTTOM_VOLTS);
    private TunableNumber inTopVolts = new TunableNumber("coral", "in_top_volts", CoralConstants.INTAKE_TOP_VOLTS);
    private TunableNumber inBottomVolts = new TunableNumber("coral", "in_bottom_volts",
            CoralConstants.INTAKE_BOTTOM_VOLTS);

    private TunableNumber ejectorVolts = new TunableNumber("coral", "ejector_volts", CoralConstants.EJECT_VOLTS);

    private TunableNumber feederVolts = new TunableNumber("feed", "feed_volts", CoralConstants.FEEDER_VOLTS);
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

        falconFeeder.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
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
        vortexManipTop.setVoltage(outTopVolts.get());
    }

    private void troughBottom() {
        vortexManipBottom.setVoltage(outBottomVolts.get());
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

    private void ejectOut() {
        neoFlipper.setVoltage(ejectorVolts.get());
    }

    private void ejectResetPose() {
        neoFlipper.setVoltage(-ejectorVolts.get());
    }

    private void stopEject() {
        neoFlipper.setVoltage(0);
    }

    public void feed() {
        falconFeeder.setVoltage(feederVolts.get());
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

    public Command stopManipCommand() {
        return runOnce(() -> stopManipulator());
    }

    public Command simpleEjectResetCommand() {
        return startEnd(() -> ejectResetPose(), () -> stopEject());
    }

    public Command feedStageCommand() {
        return startRun(() -> {
            feed();
            // intakeTop();
            // intakeBottom();
        }, () -> {
        }).until(beamBreakEngaged().negate()).finallyDo(() -> {
            stopFeed();
            // stopManipulator();
        });
    }

    /*
     * TODO: fix feedToManipCommand to execute the following
     * Stage 1: beam engaged, feeder starts running
     * Stage 2: beam broken, add a beam delay
     * stage 3: beam engaged, feeder stops
     */

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
        return branchCommand().withTimeout(CoralConstants.L4_EJECTION_TIMEOUT).andThen(startEnd(() -> {
            intakeTop();
            ejectOut();
        }, () -> {
            stopEject();
            stopManipulator();
        }));
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
}
