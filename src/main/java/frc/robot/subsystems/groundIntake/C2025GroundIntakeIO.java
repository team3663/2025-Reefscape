package frc.robot.subsystems.groundIntake;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.util.Units;

public class C2025GroundIntakeIO implements GroundIntakeIO {
    // TODO get minimum and maximum angles for pivot
    private static final GroundIntake.Constants CONSTANTS = new GroundIntake.Constants(
            Units.degreesToRadians(0.0), Units.degreesToRadians(0.0)
    );

    // TODO Confirm I did the INTAKE_GEAR_RATIO correctly, get PIVOT_GEAR_RATIO, and proximity sensor threshold/hystereis
    private final double INTAKE_GEAR_RATIO = 10.0;
    private final double PIVOT_GEAR_RATIO = 0.0;
    private final double PROXIMITY_THRESHOLD = 0.0;
    private final double PROXIMITY_HYSTERESIS = 0.0;

    private final TalonFX pivotMotor;
    private final TalonFX intakeMotor;
    private final CANcoder pivotCanCoder;
    private final CANrange gamePieceDetector;

    // TODO add sims

    private final NeutralOut stopRequest = new NeutralOut();
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);

    public C2025GroundIntakeIO(TalonFX pivotMotor, TalonFX intakeMotor, CANcoder pivotCanCoder, CANrange gamePieceDetector) {
        this.pivotMotor = pivotMotor;
        this.intakeMotor = intakeMotor;
        this.pivotCanCoder = pivotCanCoder;
        this.gamePieceDetector = gamePieceDetector;

        // Proximity Sensor config
        CANrangeConfiguration canRangeConfig = new CANrangeConfiguration();
        canRangeConfig.ProximityParams.ProximityThreshold = PROXIMITY_THRESHOLD;
        canRangeConfig.ProximityParams.ProximityHysteresis = PROXIMITY_HYSTERESIS;

        // CANCoder config
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        // TODO Get offset value
        canCoderConfig.MagnetSensor.MagnetOffset = 0.0;
        pivotCanCoder.getConfigurator().apply(canCoderConfig);

        // Pivot motor
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        pivotConfig.Slot0.kV = 0.0;
        pivotConfig.Slot0.kP = 0.0;
        pivotConfig.Slot0.kI = 0.0;
        pivotConfig.Slot0.kD = 0.0;
        pivotConfig.Slot0.kG = 0.0;
        pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        pivotConfig.Feedback.FeedbackRemoteSensorID = this.pivotCanCoder.getDeviceID();
        pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        pivotConfig.Feedback.RotorToSensorRatio = PIVOT_GEAR_RATIO;

        pivotMotor.getConfigurator().apply(pivotConfig);

        // Intake motor
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeConfig.Feedback.SensorToMechanismRatio = INTAKE_GEAR_RATIO;

        intakeMotor.getConfigurator().apply(intakeConfig);
    }

    @Override
    public GroundIntake.Constants getConstants() {
        return CONSTANTS;
    }

    @Override
    public void updateInputs(GroundIntakeInputs inputs) {
        // Pivot Motor
        inputs.pivotCurrentVelocity = Units.rotationsToRadians(pivotMotor.getVelocity().getValueAsDouble());
        inputs.pivotCurrentAppliedVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();
        inputs.pivotMotorTemperature = pivotMotor.getDeviceTemp().getValueAsDouble();
        inputs.pivotCurrentDraw = pivotMotor.getSupplyCurrent().getValueAsDouble();
        inputs.pivotCurrentPosition = Units.rotationsToRadians(pivotMotor.getPosition().getValueAsDouble());

        // Intake Motor
        inputs.intakeCurrentVelocity = Units.rotationsToRadians(intakeMotor.getVelocity().getValueAsDouble());
        inputs.intakeCurrentAppliedVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
        inputs.intakeMotorTemperature = intakeMotor.getDeviceTemp().getValueAsDouble();
        inputs.intakeCurrentDraw = intakeMotor.getSupplyCurrent().getValueAsDouble();

        inputs.gamePieceDetected = gamePieceDetector.getIsDetected().getValue().booleanValue();
    }

    // Pivot Motor
    @Override
    public void stopPivot() {
        pivotMotor.setControl(stopRequest);
    }

    @Override
    public void setTargetPositionPivot(double position) {
        pivotMotor.setControl(positionRequest.withPosition(Units.radiansToRotations(position)));
    }

    @Override
    public void setTargetVoltagePivot(double voltage) {
        pivotMotor.setControl(voltageRequest.withOutput(voltage));
    }

    // Intake Motor
    @Override
    public void stopIntake() {
        intakeMotor.setControl(stopRequest);
    }

    @Override
    public void setTargetVoltageIntake(double voltage) {
        intakeMotor.setControl(voltageRequest.withOutput(voltage));
    }
}

