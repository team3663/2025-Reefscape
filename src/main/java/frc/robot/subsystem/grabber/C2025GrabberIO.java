package frc.robot.subsystem.grabber;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.S1StateValue;
import edu.wpi.first.math.util.Units;

public class C2025GrabberIO implements GrabberIO {
    private final TalonFX motor;
    private final CANdi beamBreak;

    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final NeutralOut stopRequest = new NeutralOut();

    public C2025GrabberIO(TalonFX motor, CANdi beamBreak) {
        this.motor = motor;
        this.beamBreak = beamBreak;

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor.getConfigurator().apply(motorConfig);

        CANdiConfiguration CANdiConfig = new CANdiConfiguration();
        beamBreak.getConfigurator().apply(CANdiConfig);
    }

    @Override
    public void updateInputs(GrabberInputs inputs) {
        inputs.currentVelocity = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble());
        inputs.currentAppliedVoltage = motor.getMotorVoltage().getValueAsDouble();
        inputs.currentPosition = Units.rotationsToRadians(motor.getPosition().getValueAsDouble());
        inputs.motorTemperature = motor.getDeviceTemp().getValueAsDouble();
        inputs.currentDraw = motor.getSupplyCurrent().getValueAsDouble();

        inputs.beamBreakState = beamBreak.getS1State().getValue() == S1StateValue.High;
    }

    @Override
    public void stop() {
        motor.setControl(stopRequest);
    }

    @Override
    public void setTargetVoltage(double voltage) {
        motor.setControl(voltageRequest.withOutput(voltage));
    }
}