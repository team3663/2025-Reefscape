package frc.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

@Logged
public class Led {
    private final double ANIMATION_SPEED = 0.001;
    private final double LED_BRIGHTNESS = 0.01;
    private final int NUM_LEDS = 30;
    private final int POCKET_SIZE = 3;

    private final LedIo io;
    private final LedInputs inputs = new LedInputs();

    private final Animation startAnimation = new RgbFadeAnimation(LED_BRIGHTNESS, ANIMATION_SPEED, NUM_LEDS);
    private Animation currentAnimation;
    private LedColor currentColor;
    private Pattern currentPattern = Pattern.SOLID;

    public Led(LedIo io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
    }

    public void setColor(LedColor color) {
        currentColor = color;
        setPattern(currentPattern);
    }

    public void setPattern(Pattern pattern) {
        switch (pattern) {
            case SOLID:
                currentAnimation = null;
                io.setAnimation(currentAnimation);
                io.setColor(currentColor);
                currentPattern = Pattern.SOLID;
                break;

            case FADE:
                io.setColor(currentColor);
                currentAnimation = new RgbFadeAnimation(LED_BRIGHTNESS, ANIMATION_SPEED, NUM_LEDS);
                io.setAnimation(currentAnimation);
                currentPattern = Pattern.FADE;
                break;

            case LARSON:
                io.setColor(currentColor);
                currentAnimation = new LarsonAnimation(currentColor.red, currentColor.green, currentColor.blue, 0, ANIMATION_SPEED, NUM_LEDS, LarsonAnimation.BounceMode.Center, POCKET_SIZE);
                io.setAnimation(currentAnimation);
                currentPattern = Pattern.LARSON;
                break;

            case STROBE:
                io.setColor(currentColor);
                currentAnimation = new StrobeAnimation(currentColor.red, currentColor.green, currentColor.blue, 0, ANIMATION_SPEED, NUM_LEDS);
                io.setAnimation(currentAnimation);
                currentPattern = Pattern.STROBE;
                break;
        }
    }

    public Command setLedColor(LedColor color) {
        return runOnce(
                () -> io.setColor(color)
        );
    }

    public enum Pattern {
        SOLID, FADE, LARSON, STROBE;
    }
}