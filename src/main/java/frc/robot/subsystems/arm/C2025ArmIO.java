package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;

public class C2025ArmIO implements ArmIO {
    // TODO: Get real values from CAD
    private static final Arm.Constants CONSTANTS = new Arm.Constants(
            0.2, Units.degreesToRadians(0.0), Units.degreesToRadians(180.0),
            0.05, Units.degreesToRadians(-76.377), Units.degreesToRadians(74.09));
    private final double WRIST_GEAR_RATIO = (48.0 / 16.0) * (66.0 / 18.0) * (36.0 / 15.0);
    private final double SHOULDER_GEAR_RATIO = (60.0 / 16.0) * (60.0 / 22.0) * (72.0 / 12.0);

    private final TalonFX shoulderMotor;
    private final TalonFX wristMotor;
    private final CANcoder shoulderCanCoder;

    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final NeutralOut stopRequest = new NeutralOut();

    public C2025ArmIO(TalonFX shoulderMotor, TalonFX wristMotor, CANcoder shoulderCanCoder) {
        this.shoulderMotor = shoulderMotor;
        this.wristMotor = wristMotor;
        this.shoulderCanCoder = shoulderCanCoder;

        // CANCoder config
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        canCoderConfig.MagnetSensor.MagnetOffset = 0.351806640625;

        shoulderCanCoder.getConfigurator().apply(canCoderConfig);

        // Shoulder motor config
        TalonFXConfiguration shoulderConfig = new TalonFXConfiguration();
        shoulderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        shoulderConfig.CurrentLimits.SupplyCurrentLimit = 60;
        shoulderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // SysID gave us the following values, but I don't think these are right...
        // kV = 0.10508
        // KA = 0.004985
        // kS = 0.13898

        shoulderConfig.Slot0.kV = 7.0;
        shoulderConfig.Slot0.kA = 0.0;
        shoulderConfig.Slot0.kP = 50.0;
        shoulderConfig.Slot0.kI = 0.0;
        shoulderConfig.Slot0.kD = 5.0;
        shoulderConfig.Slot0.kG = 0.25;
        shoulderConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        shoulderConfig.Feedback.FeedbackRemoteSensorID = this.shoulderCanCoder.getDeviceID();
        shoulderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        shoulderConfig.Feedback.RotorToSensorRatio = SHOULDER_GEAR_RATIO;

        shoulderConfig.MotionMagic.MotionMagicJerk = 15.0;
        shoulderConfig.MotionMagic.MotionMagicAcceleration = 3.0;
        shoulderConfig.MotionMagic.MotionMagicCruiseVelocity = 1.63;

        shoulderMotor.getConfigurator().apply(shoulderConfig);

        // Wrist motor config
        TalonFXConfiguration wristConfig = new TalonFXConfiguration();
        wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        wristConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        wristConfig.Feedback.SensorToMechanismRatio = WRIST_GEAR_RATIO;
        wristConfig.CurrentLimits.SupplyCurrentLimit = 30;
        wristConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        wristConfig.Slot0.kV = 3.0;
        wristConfig.Slot0.kA = 0.0;
        wristConfig.Slot0.kP = 50.0;
        wristConfig.Slot0.kI = 0.0;
        wristConfig.Slot0.kD = 0.0;
        wristConfig.Slot0.kS = 0.2;

        wristConfig.MotionMagic.MotionMagicAcceleration = 15;
        wristConfig.MotionMagic.MotionMagicCruiseVelocity = 4;
        wristConfig.MotionMagic.MotionMagicJerk = 100.0;

        wristMotor.getConfigurator().apply(wristConfig);
    }

    @Override
    public Arm.Constants getConstants() {
        return CONSTANTS;
    }

    @Override
    public void updateInputs(ArmInputs inputs) {
        // Wrist inputs
        inputs.currentWristAppliedVoltage = wristMotor.getMotorVoltage().getValueAsDouble();
        inputs.currentWristVelocity = Units.rotationsToRadians(wristMotor.getVelocity().getValueAsDouble());
        inputs.currentWristPosition = Units.rotationsToRadians(wristMotor.getPosition().getValueAsDouble());
        inputs.currentWristDraw = wristMotor.getSupplyCurrent().getValueAsDouble();

        // Shoulder inputs
        inputs.currentAppliedShoulderVoltage = shoulderMotor.getMotorVoltage().getValueAsDouble();
        inputs.currentShoulderVelocity = Units.rotationsToRadians(shoulderMotor.getVelocity().getValueAsDouble());
        inputs.currentShoulderPosition = Units.rotationsToRadians(shoulderMotor.getPosition().getValueAsDouble());
        inputs.currentShoulderDraw = shoulderMotor.getSupplyCurrent().getValueAsDouble();
        inputs.currentShoulderEncoderPosition = Units.rotationsToRadians(shoulderCanCoder.getPosition().getValueAsDouble());
    }

    @Override
    public void stopShoulder() {
        shoulderMotor.setControl(stopRequest);
    }

    @Override
    public void setShoulderTargetPosition(double position) {
        shoulderMotor.setControl(positionRequest.withPosition(Units.radiansToRotations(position)));
    }

    @Override
    public void sysIdShoulder(Voltage voltage) {
        shoulderMotor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void stopWrist() {
        wristMotor.setControl(stopRequest);
    }

    @Override
    public void resetWristPosition(double position) {
        wristMotor.setPosition(Units.radiansToRotations(position));
    }

    @Override
    public void setWristTargetPosition(double position) {
        wristMotor.setControl(positionRequest.withPosition(Units.radiansToRotations(position)));
    }

    @Override
    public void setWristTargetVoltage(double voltage) {
        wristMotor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void sysIdWrist(Voltage voltage) {
        wristMotor.setControl(voltageRequest.withOutput(voltage));
    }
}