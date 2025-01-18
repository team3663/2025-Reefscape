package frc.robot.subsystem.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.S1StateValue;
import edu.wpi.first.math.util.Units;
//TODO figure out CANdi
public class C2025ClimberIO implements ClimberIO{
    private final TalonFX motor;
    private final CANdi beamBrake;
    private final NeutralOut stopRequest = new NeutralOut();
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);

    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    public C2025ClimberIO(TalonFX motor, CANdi beamBrake) {
        this.motor = motor;
        this.beamBrake = beamBrake;

        TalonFXConfiguration config = new TalonFXConfiguration();
        //PID for position
        config.Slot1.kV = 0.115;
        config.Slot1.kA = 0.01;
        config.Slot1.kP = 0.5;
        config.Slot1.kI = 0.0;
        config.Slot1.kD = 0.0;

        config.MotionMagic.MotionMagicCruiseVelocity = 5500.0/60.0;
        config.MotionMagic.MotionMagicAcceleration = 2500.0/60.0;

        motor.getConfigurator().apply(config);
    }
    @Override
    public void updateInputs(ClimberInputs inputs){
        inputs.currentAppliedVoltage = motor.getMotorVoltage().getValueAsDouble();
        inputs.currentPosition = Units.rotationsToRadians(motor.getPosition().getValueAsDouble());
        inputs.motorTemperature = motor.getDeviceTemp().getValueAsDouble();
        inputs.currentDraw = motor.getSupplyCurrent().getValueAsDouble();

        inputs.beamBrakeState = beamBrake.getS1State().getValue() == S1StateValue.High;

        //SIM STATES

    }

    @Override
    public void resetPosition() {
        motor.setPosition(0);
    }

    @Override
    public void stop() {
        motor.setControl(stopRequest);
    }

    @Override
    public void setTargetPosition(double position) {
        motor.setControl(positionRequest.withPosition(Units.rotationsToRadians(2)));
    }
    @Override
    public void setTargetVoltage(double voltage) {
        motor.setControl(voltageRequest.withOutput(voltage));
    }

}
