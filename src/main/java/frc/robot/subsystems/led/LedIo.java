package frc.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;

public interface LedIo {
    default void updateInputs(LedInputs inputs) {}

    default void setColor(LedColor color) {}

    default void setAnimation(Animation animation) {}
}