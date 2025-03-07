package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class C2025ElevatorIO implements ElevatorIO {
    private static final Elevator.Constants CONSTANTS = new Elevator.Constants(0.0, Units.inchesToMeters(60.8));

    private static final double GEAR_RATIO = (38.0 / 10.0);
    private static final double PULLEY_RADIUS = Units.inchesToMeters(0.7525);
    private static final double PULLEY_CIRCUMFERENCE = (2 * Math.PI * PULLEY_RADIUS);

    private final TalonFX motor;
    private final TalonFX motor2;

    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);
    private final VoltageOut sysIdRequest = new VoltageOut(0.0);
    private final NeutralOut stopRequest = new NeutralOut();
    private final VoltageOut voltageRequest = new VoltageOut(0.0);

    public C2025ElevatorIO(TalonFX motor, TalonFX motor2) {
        this.motor = motor;
        this.motor2 = motor2;

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = Constants.SUPERSTRUCTURE_COAST ? NeutralModeValue.Coast : NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.CurrentLimits.SupplyCurrentLimit = 60.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.Slot0.kV = 0.45;
        config.Slot0.kA = 0.0;
        config.Slot0.kP = 7.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kG = 0.35;
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        config.MotionMagic.MotionMagicAcceleration = 30.0;
        config.MotionMagic.MotionMagicCruiseVelocity = 26.0;

        motor.getConfigurator().apply(config);
        motor2.getConfigurator().apply(config);

        motor2.setControl(new Follower(motor.getDeviceID(), true));
    }

    @Override
    public Elevator.Constants getConstants() {
        return CONSTANTS;
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        // Inputs for Motor 1
        inputs.currentVelocityMotor1 = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble());
        inputs.currentAppliedVoltageMotor1 = motor.getMotorVoltage().getValueAsDouble();
        inputs.currentPositionMotor1 = motor.getPosition().getValueAsDouble() * PULLEY_CIRCUMFERENCE;
        inputs.motorTemperatureMotor1 = motor.getDeviceTemp().getValueAsDouble();
        inputs.currentDrawMotor1 = motor.getSupplyCurrent().getValueAsDouble();

        // Inputs for Motor 2
        inputs.currentVelocityMotor2 = Units.rotationsToRadians(motor2.getVelocity().getValueAsDouble());
        inputs.currentAppliedVoltageMotor2 = motor2.getMotorVoltage().getValueAsDouble();
        inputs.currentPositionMotor2 = motor.getPosition().getValueAsDouble() * PULLEY_CIRCUMFERENCE;
        inputs.motorTemperatureMotor2 = motor2.getDeviceTemp().getValueAsDouble();
        inputs.currentDrawMotor2 = motor2.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void resetPosition(double position) {
        motor.setPosition(position / PULLEY_CIRCUMFERENCE);
    }

    @Override
    public void stop() {
        motor.setControl(stopRequest);
    }

    @Override
    public void setTargetVoltage(double voltage) {
        motor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void setTargetPosition(double position) {
        motor.setControl(positionRequest.withPosition(position / PULLEY_CIRCUMFERENCE));
    }

    @Override
    public void runSysId(Voltage voltage) {
        motor.setControl(sysIdRequest.withOutput(voltage));
    }
}