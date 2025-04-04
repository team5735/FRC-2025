package frc.robot.constants;

import java.awt.Color;

public class CANdleConstants {
    public enum LedState {
        ANGRY(Color.RED),
        FED(Color.GREEN),
        IDLE(new Color(255, 30, 0)); // color matching is hard :((

        public final Color ledColor;

        LedState(Color ledColor) {
            this.ledColor = ledColor;
        }
    }
}