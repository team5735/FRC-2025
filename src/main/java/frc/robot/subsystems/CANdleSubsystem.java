
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.awt.Color;
import java.util.function.Supplier;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANdleConstants.LedState;
import frc.robot.constants.Constants;

public class CANdleSubsystem extends SubsystemBase {
    private CANdle candle;

    public CANdleSubsystem() {
        candle = new CANdle(Constants.CANDLE_ID);

        candle.configLEDType(LEDStripType.GRB);

        this.setDefaultCommand(idling());
    }

    public void setIdle() {
        candle.animate(new RainbowAnimation(0.5, 1.0, 8));
    }

    public Command idling() {
        return runOnce(() -> setIdle());
    }

    public Command colorFedCommand() {
        return setToColorByState(LedState.FED);
    }

    public Command colorPathingCommand() {
        return setToColorByState(LedState.PATHING);
    }

    public Command colorPathEndCommand() {
        return setToColorByState(LedState.PATHEND);
    }

    private Command setToColorByState(LedState state) {
        return runOnce(() -> {
            setToColor(state.ledColor);
        });
    }

    public Command manualSetRGB(Supplier<Double> r, Supplier<Double> g, Supplier<Double> b) {
        return run(() -> {
            setToColor(new Color(
                    Math.abs((int) (r.get() * 255)),
                    Math.abs((int) (g.get() * 255)),
                    Math.abs((int) (b.get() * 255))));
        });
    }

    public Command manualSetHSV(Supplier<Double> h, Supplier<Double> s, Supplier<Double> v) {
        return run(() -> {
            edu.wpi.first.wpilibj.util.Color c = edu.wpi.first.wpilibj.util.Color.fromHSV(
                    Math.abs(h.get().intValue()),
                    Math.abs((int) (s.get() * 255)),
                    Math.abs((int) (v.get() * 255)));
            setToColor(new Color(
                    (int) (c.red * 255),
                    (int) (c.green * 255),
                    (int) (c.blue * 255)));
        });
    }

    private void setToColor(Color color) {
        candle.setLEDs(color.getRed(), color.getGreen(), color.getBlue());
    }
}