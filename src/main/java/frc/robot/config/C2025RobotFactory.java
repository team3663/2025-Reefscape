package frc.robot.config;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.C2025ArmIO;
import frc.robot.subsystems.climber.C2025ClimberIO;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.drivetrain.CTREDrivetrainIO;
import frc.robot.subsystems.drivetrain.DrivetrainIO;
import frc.robot.subsystems.elevator.C2025ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.grabber.C2025GrabberIO;
import frc.robot.subsystems.grabber.GrabberIO;
import frc.robot.subsystems.led.LedCandleIo;
import frc.robot.subsystems.led.LedIo;
import frc.robot.subsystems.vision.LimelightIO;
import frc.robot.subsystems.vision.VisionIO;

public class C2025RobotFactory implements RobotFactory {
    private static final CANBus DRIVETRAIN_CAN_BUS = new CANBus("3663");

    private static final double MODULE_WHEEL_INSET = Units.inchesToMeters(2.625);
    private static final double FRAME_X_LENGTH = Units.inchesToMeters(27.0);
    private static final double FRAME_Y_LENGTH = Units.inchesToMeters(27.0);
    private static final double MODULE_X_OFFSET = FRAME_X_LENGTH / 2.0 - MODULE_WHEEL_INSET;
    private static final double MODULE_Y_OFFSET = FRAME_Y_LENGTH / 2.0 - MODULE_WHEEL_INSET;

    private static final double ROBOT_MOMENT_OF_INERTIA = 6.0;
    private static final double ROBOT_WEIGHT_KG = 61.235;

    private static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS = new SwerveDrivetrainConstants()
            .withCANBusName(DRIVETRAIN_CAN_BUS.getName())
            .withPigeon2Id(1)
            .withPigeon2Configs(new Pigeon2Configuration());

    private static final double DRIVE_INERTIA = 0.01;
    private static final double DRIVE_FRICTION_VOLTAGE = 0.25;
    private static final TalonFXConfiguration DRIVE_CONFIG = new TalonFXConfiguration();
    private static final Slot0Configs DRIVE_PID_CONSTANTS = new Slot0Configs();
    private static final TalonFXConfiguration STEER_CONFIG = new TalonFXConfiguration();
    private static final double MAX_DRIVE_VELOCITY = DCMotor.getFalcon500Foc(1)
            .freeSpeedRadPerSec / Constants.MK4_3PLUS_REDUCTION * Constants.MK4_WHEEL_RADIUS;

    // Creating a constants factory for the drive and steer motors of the drivetrain
    private static final SwerveModuleConstantsFactory<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> MODULE_CONSTANTS_FACTORY
            = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
            .withDriveMotorGearRatio(Constants.MK4_2PLUS_REDUCTION)
            .withDriveInertia(DRIVE_INERTIA)
            .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE)
            .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
            .withDriveMotorInitialConfigs(DRIVE_CONFIG)
            .withDriveMotorGains(DRIVE_PID_CONSTANTS)
            .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
            .withSteerMotorGearRatio(Constants.MK4N_STEER_REDUCTION)
            .withSteerInertia(Constants.MK4N_STEER_INERTIA)
            .withSteerFrictionVoltage(Constants.MK4N_STEER_FRICTION_VOLTAGE)
            .withSteerMotorInitialConfigs(STEER_CONFIG)
            .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
            .withSteerMotorGains(Constants.MK4N_STEER_PID_CONSTANTS)
            .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.FusedCANcoder)
            .withWheelRadius(Constants.MK4N_WHEEL_RADIUS)
            .withSpeedAt12Volts(MAX_DRIVE_VELOCITY);


    // offsets are found with gears to the right
    // Front Left
    private static final int DRIVETRAIN_FRONT_LEFT_STEER_ID = 2;
    private static final int DRIVETRAIN_FRONT_LEFT_DRIVE_ID = 1;
    private static final int DRIVETRAIN_FRONT_LEFT_ENCODER_ID = 2;
    private static final double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET = Units.degreesToRotations(108.28125);

    // Front Right
    private static final int DRIVETRAIN_FRONT_RIGHT_STEER_ID = 4;
    private static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_ID = 3;
    private static final int DRIVETRAIN_FRONT_RIGHT_ENCODER_ID = 4;
    private static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = Units.degreesToRotations(142.55859);

    // Back Left
    private static final int DRIVETRAIN_BACK_LEFT_STEER_ID = 6;
    private static final int DRIVETRAIN_BACK_LEFT_DRIVE_ID = 5;
    private static final int DRIVETRAIN_BACK_LEFT_ENCODER_ID = 6;
    private static final double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = Units.degreesToRotations(292.32422);

    // Back Right
    private static final int DRIVETRAIN_BACK_RIGHT_STEER_ID = 8;
    private static final int DRIVETRAIN_BACK_RIGHT_DRIVE_ID = 7;
    private static final int DRIVETRAIN_BACK_RIGHT_ENCODER_ID = 8;
    private static final double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = Units.degreesToRotations(-177.89063);


    @Override
    public DrivetrainIO createDrivetrainIo() {
        // Configuring front left module
        var frontLeftConfig = MODULE_CONSTANTS_FACTORY.createModuleConstants(
                DRIVETRAIN_FRONT_LEFT_STEER_ID,
                DRIVETRAIN_FRONT_LEFT_DRIVE_ID,
                DRIVETRAIN_FRONT_LEFT_ENCODER_ID,
                DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET,
                MODULE_X_OFFSET, MODULE_Y_OFFSET,
                false, true, false
        );

        // Configuring front right module
        var frontRightConfig = MODULE_CONSTANTS_FACTORY.createModuleConstants(
                DRIVETRAIN_FRONT_RIGHT_STEER_ID,
                DRIVETRAIN_FRONT_RIGHT_DRIVE_ID,
                DRIVETRAIN_FRONT_RIGHT_ENCODER_ID,
                DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET,
                MODULE_X_OFFSET, -MODULE_Y_OFFSET,
                false, true, false
        );

        // Configuring back left module
        var backLeftConfig = MODULE_CONSTANTS_FACTORY.createModuleConstants(
                DRIVETRAIN_BACK_LEFT_STEER_ID,
                DRIVETRAIN_BACK_LEFT_DRIVE_ID,
                DRIVETRAIN_BACK_LEFT_ENCODER_ID,
                DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET,
                -MODULE_X_OFFSET, MODULE_Y_OFFSET,
                false, true, false
        );

        // Configuring back right module
        var backRightConfig = MODULE_CONSTANTS_FACTORY.createModuleConstants(
                DRIVETRAIN_BACK_RIGHT_STEER_ID,
                DRIVETRAIN_BACK_RIGHT_DRIVE_ID,
                DRIVETRAIN_BACK_RIGHT_ENCODER_ID,
                DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET,
                -MODULE_X_OFFSET, -MODULE_Y_OFFSET,
                false, true, false
        );

        return new CTREDrivetrainIO(ROBOT_WEIGHT_KG, ROBOT_MOMENT_OF_INERTIA,
                DRIVETRAIN_CONSTANTS,
                frontLeftConfig, frontRightConfig,
                backLeftConfig, backRightConfig);
    }

//    @Override
//    public ArmIO createArmIo() {
//        return new C2025ArmIO(new TalonFX(12), new TalonFX(13), new CANcoder(12));
//    }
//
//    @Override
//    public ClimberIO createClimberIo() {
//        return new C2025ClimberIO(new TalonFX(11), new CANdi(0), new CANcoder(0));
//    }
//
//    @Override
//    public ElevatorIO createElevatorIo() {
//        return new C2025ElevatorIO(new TalonFX(9), new TalonFX(10));
//    }
//
//    @Override
//    public GrabberIO createGrabberIo() {
//        return new C2025GrabberIO(new TalonFX(14), new DigitalInput(0));
//    }
//
//    @Override
//    public LedIo createLedIo() {
//        return new LedCandleIo(new CANdle(1));
//    }
//
//    @Override
//    public VisionIO[] createVisionIo() {
//
//        Rotation3d leftRotation = new Rotation3d(Constants.LEFT_CAMERA_ROLL, Constants.LEFT_CAMERA_PITCH, Constants.LEFT_CAMERA_YAW);
//        Transform3d leftTransform = new Transform3d(Constants.LEFT_CAMERA_X, Constants.LEFT_CAMERA_Y, Constants.LEFT_CAMERA_Z, leftRotation);
//
//        Rotation3d rightRotation = new Rotation3d(Constants.RIGHT_CAMERA_ROLL, Constants.RIGHT_CAMERA_PITCH, Constants.RIGHT_CAMERA_YAW);
//        Transform3d rightTransform = new Transform3d(Constants.RIGHT_CAMERA_X, Constants.RIGHT_CAMERA_Y, Constants.RIGHT_CAMERA_Z, rightRotation);
//
//        return new VisionIO[]{
//                new LimelightIO(Constants.LEFT_CAMERA_NAME, leftTransform),
//                new LimelightIO(Constants.RIGHT_CAMERA_NAME, rightTransform),
//
//        };
//    }
}
