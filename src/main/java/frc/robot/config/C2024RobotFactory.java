package frc.robot.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.CTREDrivetrainIO;
import frc.robot.subsystems.drivetrain.DrivetrainIO;

public class C2024RobotFactory implements RobotFactory{

    private static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS = new SwerveDrivetrainConstants()
            .withCANBusName(Constants.DRIVETRAIN_CAN_BUS.getName())
            .withPigeon2Id(0)
            .withPigeon2Configs(new Pigeon2Configuration());

    private static final double DRIVE_INERTIA = 0.001;
    private static final double DRIVE_FRICTION_VOLTAGE = 0.25;
    private static final TalonFXConfiguration DRIVE_CONFIG = new TalonFXConfiguration();
    private static final Slot0Configs DRIVE_PID_CONSTANTS = new Slot0Configs();
    private static final TalonFXConfiguration STEER_CONFIG = new TalonFXConfiguration();

    // Creating a constants factory for the drive and steer motors of the drivetrain
    private static final SwerveModuleConstantsFactory<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> MODULE_CONSTANTS_FACTORY
            = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
            .withDriveMotorGearRatio(Constants.MK4I_2PLUS_REDUCTION)
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
            .withWheelRadius(Constants.MK4I_WHEEL_RADIUS);

    // Front Left
    private static final int DRIVETRAIN_FRONT_LEFT_STEER_ID = 1;
    private static final int DRIVETRAIN_FRONT_LEFT_DRIVE_ID = 2;
    private static final int DRIVETRAIN_FRONT_LEFT_ENCODER_ID = 1;
    private static final double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET = Units.degreesToRadians(19.16015625);

    // Front Right
    private static final int DRIVETRAIN_FRONT_RIGHT_STEER_ID = 3;
    private static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_ID = 4;
    private static final int DRIVETRAIN_FRONT_RIGHT_ENCODER_ID = 2;
    private static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = Units.degreesToRadians(-130.078125);

    // Back Left
    private static final int DRIVETRAIN_BACK_LEFT_STEER_ID = 5;
    private static final int DRIVETRAIN_BACK_LEFT_DRIVE_ID = 6;
    private static final int DRIVETRAIN_BACK_LEFT_ENCODER_ID = 3;
    private static final double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = Units.degreesToRadians(111.26953125);

    // Back Right
    private static final int DRIVETRAIN_BACK_RIGHT_STEER_ID = 7;
    private static final int DRIVETRAIN_BACK_RIGHT_DRIVE_ID = 8;
    private static final int DRIVETRAIN_BACK_RIGHT_ENCODER_ID = 4;
    private static final double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = Units.degreesToRadians(125.5078125);


    @Override
    public DrivetrainIO createDrivetrainIo() {
        // Configuring front left module
        var frontLeftConfig = MODULE_CONSTANTS_FACTORY.createModuleConstants(
                DRIVETRAIN_FRONT_LEFT_STEER_ID,
                DRIVETRAIN_FRONT_LEFT_DRIVE_ID,
                DRIVETRAIN_FRONT_LEFT_ENCODER_ID,
                DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET,
                Constants.MODULE_X_OFFSET, Constants.MODULE_Y_OFFSET,
                false, false, false
        );

        // Configuring front right module
        var frontRightConfig = MODULE_CONSTANTS_FACTORY.createModuleConstants(
                DRIVETRAIN_FRONT_RIGHT_STEER_ID,
                DRIVETRAIN_FRONT_RIGHT_DRIVE_ID,
                DRIVETRAIN_FRONT_RIGHT_ENCODER_ID,
                DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET,
                Constants.MODULE_X_OFFSET, -Constants.MODULE_Y_OFFSET,
                false, false, false
        );

        // Configuring back left module
        var backLeftConfig = MODULE_CONSTANTS_FACTORY.createModuleConstants(
                DRIVETRAIN_BACK_LEFT_STEER_ID,
                DRIVETRAIN_BACK_LEFT_DRIVE_ID,
                DRIVETRAIN_BACK_LEFT_ENCODER_ID,
                DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET,
                -Constants.MODULE_X_OFFSET, Constants.MODULE_Y_OFFSET,
                false, false, false
        );

        // Configuring back right module
        var backRightConfig = MODULE_CONSTANTS_FACTORY.createModuleConstants(
                DRIVETRAIN_BACK_RIGHT_STEER_ID,
                DRIVETRAIN_BACK_RIGHT_DRIVE_ID,
                DRIVETRAIN_BACK_RIGHT_ENCODER_ID,
                DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET,
                -Constants.MODULE_X_OFFSET, -Constants.MODULE_Y_OFFSET,
                false, false, false
        );

        return new CTREDrivetrainIO(DRIVETRAIN_CONSTANTS,
                frontLeftConfig, frontRightConfig,
                backLeftConfig, backRightConfig);
    }
}


