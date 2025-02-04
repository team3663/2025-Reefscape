package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;

public class C2025ArmIO implements ArmIO {
    // TODO: Get real values from CAD
    private static final Arm.Constants CONSTANTS = new Arm.Constants(0.2, 0.05);

    private final TalonFX shoulderMotor;
    private final TalonFX wristMotor;
    private final CANcoder shoulderCanCoder;

    private final DCMotorSim shoulderSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1),
                    0.001, 1.0),
            DCMotor.getKrakenX60(1).withReduction(1.0));

    private final DCMotorSim wristSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1),
                    0.001, 1.0),
            DCMotor.getKrakenX60(1).withReduction(1.0));

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
        canCoderConfig.MagnetSensor.MagnetOffset = 0.0;
//        canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

        shoulderCanCoder.getConfigurator().apply(canCoderConfig);

        // Shoulder motor config
        TalonFXConfiguration shoulderConfig = new TalonFXConfiguration();
        shoulderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        shoulderConfig.Slot0.kV = 0.115;
        shoulderConfig.Slot0.kA = 0.01;
        shoulderConfig.Slot0.kP = 0.5;
        shoulderConfig.Slot0.kI = 0.0;
        shoulderConfig.Slot0.kD = 0.0;

        if (!Robot.isSimulation()) {
            shoulderConfig.Feedback.FeedbackRemoteSensorID = this.shoulderCanCoder.getDeviceID();
            shoulderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        }

        shoulderConfig.MotionMagic.MotionMagicAcceleration = 2500.0 / 60.0;
        shoulderConfig.MotionMagic.MotionMagicCruiseVelocity = 5500.0 / 60.0;

        shoulderMotor.getConfigurator().apply(shoulderConfig);

        // Wrist motor config
        TalonFXConfiguration wristConfig = new TalonFXConfiguration();
        wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        wristConfig.Slot0.kV = 0.115;
        wristConfig.Slot0.kA = 0.01;
        wristConfig.Slot0.kP = 0.5;
        wristConfig.Slot0.kI = 0.0;
        wristConfig.Slot0.kD = 0.0;

        wristConfig.MotionMagic.MotionMagicAcceleration = 2500.0 / 60.0;
        wristConfig.MotionMagic.MotionMagicCruiseVelocity = 5500.0 / 60.0;

        wristMotor.getConfigurator().apply(wristConfig);
    }

    @Override
    public Arm.Constants getConstants() {
        return CONSTANTS;
    }

    @Override
    public void updateInputs(ArmInputs inputs) {
        // Sims for Wrist and Shoulder
        if (Robot.isSimulation()) {
            // Shoulder sim
            var shoulderSimState = shoulderMotor.getSimState();
            shoulderSim.setInputVoltage(shoulderSimState.getMotorVoltage());

            // Updates sim for shoulder every 20 milliseconds
            shoulderSim.update(Robot.kDefaultPeriod);
            shoulderSimState.setRotorAcceleration(shoulderSim.getAngularAcceleration());
            shoulderSimState.setRotorVelocity(shoulderSim.getAngularVelocity());
            shoulderSimState.setRawRotorPosition(shoulderSim.getAngularPosition());

            // Wrist sim
            var wristSimState = wristMotor.getSimState();
            wristSim.setInputVoltage(wristSimState.getMotorVoltage());

            // Updates sim for wrist every 20 milliseconds
            wristSim.update(Robot.kDefaultPeriod);
            wristSimState.setRotorAcceleration(wristSim.getAngularAcceleration());
            wristSimState.setRotorVelocity(wristSim.getAngularVelocity());
            wristSimState.setRawRotorPosition(wristSim.getAngularPosition());
        }

        // Wrist inputs
        inputs.currentWristAppliedVoltage = wristMotor.getMotorVoltage().getValueAsDouble();
        inputs.currentWristVelocity = Units.rotationsToRadians(wristMotor.getVelocity().getValueAsDouble());
        inputs.currentWristPosition = Units.rotationsToRadians(wristMotor.getPosition().getValueAsDouble());
        inputs.wristMotorPosition = wristMotor.getDeviceTemp().getValueAsDouble();
        inputs.currentWristDraw = wristMotor.getSupplyCurrent().getValueAsDouble();

        // Shoulder inputs
        inputs.currentAppliedShoulderVoltage = shoulderMotor.getMotorVoltage().getValueAsDouble();
        inputs.currentShoulderVelocity = Units.rotationsToRadians(shoulderMotor.getVelocity().getValueAsDouble());
        inputs.currentShoulderPosition = Units.rotationsToRadians(shoulderMotor.getPosition().getValueAsDouble());
        inputs.shoulderMotorPosition = shoulderMotor.getDeviceTemp().getValueAsDouble();
        inputs.currentShoulderDraw = shoulderMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void stopShoulder() {
        shoulderMotor.setControl(stopRequest);
    }

    @Override
    public void resetShoulderPosition() {
        shoulderMotor.setPosition(0.0);
    }

    @Override
    public void setShoulderTargetPosition(double position) {
        shoulderMotor.setControl(positionRequest.withPosition(Units.radiansToRotations(position)));
    }

    @Override
    public void stopWrist() {
        wristMotor.setControl(stopRequest);
    }

    @Override
    public void resetWristPosition() {
        wristMotor.setPosition(0.0);
    }

    @Override
    public void setWristTargetPosition(double position) {
        wristMotor.setControl(positionRequest.withPosition(Units.radiansToRotations(position)));
    }
    @Override
    public void setWristTargetVoltage(double voltage){
        wristMotor.setControl(voltageRequest.withOutput(voltage));
    }
}