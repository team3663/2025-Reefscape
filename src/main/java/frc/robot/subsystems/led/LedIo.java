package frc.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;
import edu.wpi.first.wpilibj.util.Color;

public interface LedIo {
    default void updateInputs(LedInputs inputs) {
    }

    default void setColor(Color color) {
    }

    default void setAnimation(Animation animation) {
    }
}