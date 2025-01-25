package frc.robot.config;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.CTREDrivetrainIO;
import frc.robot.subsystems.drivetrain.DrivetrainIO;

public class C2024RobotFactory implements RobotFactory {
    private static final CANBus DRIVETRAIN_CAN_BUS = new CANBus("3663");

    private static final double MODULE_WHEEL_INSET = Units.inchesToMeters(2.625);
    private static final double INTAKE_X_OFFSET = Units.inchesToMeters(5.037024);
    private static final double FRAME_X_LENGTH = Units.inchesToMeters(28.287024);
    private static final double FRAME_Y_LENGTH = Units.inchesToMeters(27.5625);
    static final double FRONT_MODULE_X_OFFSET = FRAME_X_LENGTH / 2.0 - MODULE_WHEEL_INSET;
    private static final double BACK_MODULE_X_OFFSET = FRAME_X_LENGTH / 2.0 - INTAKE_X_OFFSET - MODULE_WHEEL_INSET;
    private static final double MODULE_Y_OFFSET = FRAME_Y_LENGTH / 2.0 - MODULE_WHEEL_INSET;

    private static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS = new SwerveDrivetrainConstants()
            .withCANBusName(DRIVETRAIN_CAN_BUS.getName())
            .withPigeon2Id(0)
            .withPigeon2Configs(new Pigeon2Configuration());

    private static final double DRIVE_INERTIA = 0.001;
    private static final double DRIVE_FRICTION_VOLTAGE = 0.0;
    private static final TalonFXConfiguration DRIVE_CONFIG = new TalonFXConfiguration();
    private static final Slot0Configs DRIVE_PID_CONSTANTS = new Slot0Configs();
    private static final TalonFXConfiguration STEER_CONFIG = new TalonFXConfiguration();
    private static final double MAX_DRIVE_VELOCITY = DCMotor.getFalcon500Foc(1)
            .freeSpeedRadPerSec / Constants.MK4I_3PLUS_REDUCTION * Constants.MK4I_WHEEL_RADIUS;

    // Creating a constants factory for the drive and steer motors of the drivetrain
    private static final SwerveModuleConstantsFactory<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> MODULE_CONSTANTS_FACTORY
            = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
            .withDriveMotorGearRatio(Constants.MK4I_3PLUS_REDUCTION)
            .withDriveInertia(DRIVE_INERTIA)
            .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE)
            .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
            .withDriveMotorInitialConfigs(DRIVE_CONFIG)
            .withDriveMotorGains(DRIVE_PID_CONSTANTS)
            .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
            .withSteerMotorGearRatio(Constants.MK4I_STEER_REDUCTION)
            .withSteerInertia(Constants.MK4I_STEER_INERTIA)
            .withSteerFrictionVoltage(Constants.MK4I_STEER_FRICTION_VOLTAGE)
            .withSteerMotorInitialConfigs(STEER_CONFIG)
            .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
            .withSteerMotorGains(Constants.MK4I_STEER_PID_CONSTANTS)
            .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.FusedCANcoder)
            .withWheelRadius(Constants.MK4I_WHEEL_RADIUS)
            .withSpeedAt12Volts(MAX_DRIVE_VELOCITY);

    // Front Left
    private static final int DRIVETRAIN_FRONT_LEFT_DRIVE_ID = 1;
    private static final int DRIVETRAIN_FRONT_LEFT_STEER_ID = 2;
    private static final int DRIVETRAIN_FRONT_LEFT_ENCODER_ID = 1;
    private static final double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET = Units.degreesToRotations(-159.87);

    // Front Right
    private static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_ID = 3;
    private static final int DRIVETRAIN_FRONT_RIGHT_STEER_ID = 4;
    private static final int DRIVETRAIN_FRONT_RIGHT_ENCODER_ID = 2;
    private static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = Units.degreesToRotations(-128.85);

    // Back Left
    private static final int DRIVETRAIN_BACK_LEFT_DRIVE_ID = 5;
    private static final int DRIVETRAIN_BACK_LEFT_STEER_ID = 6;
    private static final int DRIVETRAIN_BACK_LEFT_ENCODER_ID = 3;
    private static final double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = Units.degreesToRotations(-70.14);

    // Back Right
    private static final int DRIVETRAIN_BACK_RIGHT_DRIVE_ID = 7;
    private static final int DRIVETRAIN_BACK_RIGHT_STEER_ID = 8;
    private static final int DRIVETRAIN_BACK_RIGHT_ENCODER_ID = 4;
    private static final double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = Units.degreesToRotations(-234.84);


    @Override
    public DrivetrainIO createDrivetrainIo() {
        // Configuring front left module
        var frontLeftConfig = MODULE_CONSTANTS_FACTORY.createModuleConstants(
                DRIVETRAIN_FRONT_LEFT_STEER_ID,
                DRIVETRAIN_FRONT_LEFT_DRIVE_ID,
                DRIVETRAIN_FRONT_LEFT_ENCODER_ID,
                DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET,
                FRONT_MODULE_X_OFFSET, MODULE_Y_OFFSET,
                true, true, false
        );

        // Configuring front right module
        var frontRightConfig = MODULE_CONSTANTS_FACTORY.createModuleConstants(
                DRIVETRAIN_FRONT_RIGHT_STEER_ID,
                DRIVETRAIN_FRONT_RIGHT_DRIVE_ID,
                DRIVETRAIN_FRONT_RIGHT_ENCODER_ID,
                DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET,
                FRONT_MODULE_X_OFFSET, -MODULE_Y_OFFSET,
                false, true, false
        );

        // Configuring back left module
        var backLeftConfig = MODULE_CONSTANTS_FACTORY.createModuleConstants(
                DRIVETRAIN_BACK_LEFT_STEER_ID,
                DRIVETRAIN_BACK_LEFT_DRIVE_ID,
                DRIVETRAIN_BACK_LEFT_ENCODER_ID,
                DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET,
                -BACK_MODULE_X_OFFSET, MODULE_Y_OFFSET,
                true, true, false
        );

        // Configuring back right module
        var backRightConfig = MODULE_CONSTANTS_FACTORY.createModuleConstants(
                DRIVETRAIN_BACK_RIGHT_STEER_ID,
                DRIVETRAIN_BACK_RIGHT_DRIVE_ID,
                DRIVETRAIN_BACK_RIGHT_ENCODER_ID,
                DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET,
                -BACK_MODULE_X_OFFSET, -MODULE_Y_OFFSET,
                false, true, false
        );

        return new CTREDrivetrainIO(DRIVETRAIN_CONSTANTS,
                frontLeftConfig, frontRightConfig,
                backLeftConfig, backRightConfig);
    }
}


