package frc.robot.constants;

import java.awt.Color;

public class CANdleConstants {
    public enum LedState {
        FED(Color.GREEN),
        IDLE(new Color(255, 100, 0)); // Orange-y maybe??

        public final Color ledColor;

        LedState(Color ledColor) {
            this.ledColor = ledColor;
        }
    }
}