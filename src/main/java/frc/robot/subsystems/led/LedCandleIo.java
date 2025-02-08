package frc.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj.util.Color;

public class LedCandleIo implements LedIo {
    private final CANdle candle;

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
    public void setColor(Color color) {
        candle.setLEDs((int) (color.red * 255), (int) (color.green * 255), (int) (color.green * 255));
    }

    @Override
    public void setAnimation(Animation animation) {
        candle.animate(animation);
    }
}