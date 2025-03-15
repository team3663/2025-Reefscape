package frc.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMode;

import java.util.function.Supplier;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

@Logged
public class Led extends SubsystemBase {
    private static final Pattern CORAL_PATTERN = Pattern.SOLID;
    private static final Color CORAL_LEVEL_1_COLOR = Color.kYellow;
    private static final Color CORAL_LEVEL_2_COLOR = Color.kGreen;
    private static final Color CORAL_LEVEL_3_COLOR = Color.kCyan;
    private static final Color CORAL_LEVEL_4_COLOR = Color.kBlue;

    private static final Pattern ALGAE_PATTERN = Pattern.SOLID;
    private static final Color ALGAE_PROCESSOR_COLOR = Color.kAntiqueWhite;
    private static final Color ALGAE_REMOVE_LOWER_COLOR = new Color(0.8f, 0.25f, 0.25f);
    private static final Color ALGAE_REMOVE_UPPER_COLOR = Color.kDarkRed;
    private static final Color ALGAE_NET_COLOR = Color.kPurple;
    private static final Color ALGAE_PICKUP_GROUND_COLOR = Color.kChocolate;

    private final double ANIMATION_SPEED = 0.001;
    private final double LED_BRIGHTNESS = 1.0;
    private final int NUM_LEDS = 200;
    private final int POCKET_SIZE = 3;

    private final LedIo io;
    private final LedInputs inputs = new LedInputs();

    private final Animation startAnimation = new RgbFadeAnimation(LED_BRIGHTNESS, ANIMATION_SPEED, NUM_LEDS);
    private Animation currentAnimation;
    private Color currentColor = new Color();
    private Pattern currentPattern = Pattern.SOLID;

    public Led(LedIo io) {
        this.io = io;
        io.setAnimation(startAnimation);
    }

    public void periodic() {
        io.updateInputs(inputs);
    }

    public void setColor(Color color) {
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
                currentAnimation = new LarsonAnimation((int) (currentColor.red * 255), (int) (currentColor.green * 255), (int) (currentColor.blue * 255), 0, ANIMATION_SPEED, NUM_LEDS, LarsonAnimation.BounceMode.Center, POCKET_SIZE);
                io.setAnimation(currentAnimation);
                currentPattern = Pattern.LARSON;
                break;

            case STROBE:
                io.setColor(currentColor);
                currentAnimation = new StrobeAnimation((int) (currentColor.red * 255), (int) (currentColor.green * 255), (int) (currentColor.blue * 255), 0, ANIMATION_SPEED, NUM_LEDS);
                io.setAnimation(currentAnimation);
                currentPattern = Pattern.STROBE;
                break;
        }
    }

    public Command setLedColor(Color color) {
        return runOnce(() -> {
                    currentColor = color;
                    io.setColor(color);
                }
        );
    }

    public Command signalCommand(Supplier<RobotMode> robotMode) {
        return run(() -> {
                    switch (robotMode.get()) {
                        case CORAL_LEVEL_1 -> {
                            currentColor = CORAL_LEVEL_1_COLOR;
                            setPattern(CORAL_PATTERN);
                        }
                        case CORAL_LEVEL_2 -> {
                            currentColor = CORAL_LEVEL_2_COLOR;
                            setPattern(CORAL_PATTERN);
                        }
                        case CORAL_LEVEL_3 -> {
                            currentColor = CORAL_LEVEL_3_COLOR;
                            setPattern(CORAL_PATTERN);
                        }
                        case CORAL_LEVEL_4 -> {
                            currentColor = CORAL_LEVEL_4_COLOR;
                            setPattern(CORAL_PATTERN);
                        }
                        case ALGAE_NET -> {
                            currentColor = ALGAE_NET_COLOR;
                            setPattern(ALGAE_PATTERN);
                        }
                        case ALGAE_PROCESSOR -> {
                            currentColor = ALGAE_PROCESSOR_COLOR;
                            setPattern(ALGAE_PATTERN);
                        }
                        case ALGAE_REMOVE_LOWER -> {
                            currentColor = ALGAE_REMOVE_LOWER_COLOR;
                            setPattern(ALGAE_PATTERN);
                        }
                        case ALGAE_REMOVE_UPPER -> {
                            currentColor = ALGAE_REMOVE_UPPER_COLOR;
                            setPattern(ALGAE_PATTERN);
                        }
                        case ALGAE_PICKUP_GROUND -> {
                            currentColor = ALGAE_PICKUP_GROUND_COLOR;
                            setPattern(ALGAE_PATTERN);
                        }
                    }
                }
        );
    }

    public Command intakeFlash() {
        return runOnce(() -> {
            currentColor = Color.kWhite;
            setPattern(Pattern.STROBE);
        }).andThen(waitSeconds(0.5));
    }

    public enum Pattern {
        SOLID, FADE, LARSON, STROBE;
    }
}