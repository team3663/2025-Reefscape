package frc.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;

public class LedCandleIo implements LedIo {
    private final CANdle candle;

    private Animation animation;

    public LedCandleIo(CANdle candle) {
        this.candle = candle;
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = CANdle.LEDStripType.RGB;
        candle.configAllSettings(config);
    }

    @Override
    public void updateInputs(LedInputs inputs) {
        inputs.current = candle.getCurrent();
        inputs.temperature = candle.getTemperature();
    }

    @Override
    public void setColor(LedColor color) {
        candle.setLEDs(color.red, color.green, color.blue);
    }

    @Override
    public void setAnimation(Animation animation) {
        candle.animate(animation);
    }
}