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
            Units.degreesToRadians(0.0), Units.degreesToRadians(168.0)
    );

    private final double INTAKE_GEAR_RATIO = 10.0;
    private final double PIVOT_GEAR_RATIO = (58.0 / 12.0) * (56.0 / 22.0) * (32.0 / 12.0);
    //TODO figure out actual proximity values
    private final double PROXIMITY_THRESHOLD = Units.inchesToMeters(0.0);
    private final double PROXIMITY_HYSTERESIS = Units.inchesToMeters(0.0);

    private final TalonFX pivotMotor;
    private final TalonFX intakeMotor;
    private final CANrange gamePieceDetector;

    // TODO add sims

    private final NeutralOut stopRequest = new NeutralOut();
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);

    public C2025GroundIntakeIO(TalonFX pivotMotor, TalonFX intakeMotor, CANrange gamePieceDetector) {
        this.pivotMotor = pivotMotor;
        this.intakeMotor = intakeMotor;
        this.gamePieceDetector = gamePieceDetector;

        // Proximity Sensor config
        CANrangeConfiguration canRangeConfig = new CANrangeConfiguration();
//        canRangeConfig.ProximityParams.ProximityThreshold = Units.inchesToMeters(PROXIMITY_THRESHOLD);
//        canRangeConfig.ProximityParams.ProximityHysteresis = Units.inchesToMeters(PROXIMITY_HYSTERESIS);
        gamePieceDetector.getConfigurator().apply(canRangeConfig);

        // Pivot motor
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        pivotConfig.Voltage.PeakReverseVoltage = -3.0;
        pivotConfig.Voltage.PeakForwardVoltage = 3.0;

        pivotConfig.Slot0.kV = 0.25;
        pivotConfig.Slot0.kP = 10.0;
        pivotConfig.Slot0.kI = 0.0;
        pivotConfig.Slot0.kD = 0.0;
        pivotConfig.Slot0.kG = 0.0;
        pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = Units.degreesToRotations(1000.0);
        pivotConfig.MotionMagic.MotionMagicAcceleration = Units.degreesToRotations(250.0);

        pivotConfig.Feedback.SensorToMechanismRatio = PIVOT_GEAR_RATIO;

        pivotMotor.getConfigurator().apply(pivotConfig);

        // Intake motor
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeConfig.Feedback.SensorToMechanismRatio = INTAKE_GEAR_RATIO;

        intakeMotor.getConfigurator().apply(intakeConfig);
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

    @Override
    public GroundIntake.Constants getConstants() {
        return CONSTANTS;
    }

    // Pivot Motor
    @Override
    public void stopPivot() {
        pivotMotor.setControl(stopRequest);
    }

    @Override
    public void resetPivotPosition(double position){
        pivotMotor.setPosition(Units.radiansToRotations(position));
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

