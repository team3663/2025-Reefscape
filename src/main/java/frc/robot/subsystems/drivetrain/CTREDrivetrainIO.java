package frc.robot.subsystems.drivetrain;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Robot;

import java.util.Optional;

public class CTREDrivetrainIO implements DrivetrainIO {
    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain;
    private final Drivetrain.Constants constants;

    private final SwerveRequest.FieldCentric fieldOrientedRequest = new SwerveRequest.FieldCentric();
    private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeedsRequest = new SwerveRequest.ApplyRobotSpeeds();
    private final SwerveRequest.Idle stopRequest = new SwerveRequest.Idle();
    private final SwerveRequest.SysIdSwerveTranslation sysIdTranslationRequest = new SwerveRequest.SysIdSwerveTranslation();

    private volatile SwerveDrivetrain.SwerveDriveState lastState = new SwerveDrivetrain.SwerveDriveState();

    private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);

    private double yawOffset = 0;

    @SafeVarargs
    public CTREDrivetrainIO(
            double robotWeightKG,
            double robotMOI,
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>... moduleConstants
    ) {
        this.drivetrain = new SwerveDrivetrain<>(
                TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants,
                0.0,
                VecBuilder.fill(0.05, 0.05, 0.001),
                VecBuilder.fill(10.0, 10.0, 10.0),
                moduleConstants
        );

        headingController.enableContinuousInput(-Math.PI, Math.PI);

        drivetrain.registerTelemetry(state -> lastState = state.clone());

        double maxModuleVelocity = Double.MAX_VALUE;
        double maxDriveBaseRadius = 0.0;
        for (int index = 0; index < moduleConstants.length; index++) {
            double x = moduleConstants[index].LocationX;
            double y = moduleConstants[index].LocationY;
            double moduleVelocity = moduleConstants[index].SpeedAt12Volts;
            double driveBaseRadius = Math.hypot(x, y);

            maxModuleVelocity = Math.min(maxModuleVelocity, moduleVelocity);
            maxDriveBaseRadius = Math.max(maxDriveBaseRadius, driveBaseRadius);
        }

        ModuleConfig moduleConfig = new ModuleConfig(moduleConstants[0].WheelRadius, maxModuleVelocity,
                Constants.WHEEL_COF, DCMotor.getKrakenX60Foc(1), moduleConstants[0].DriveMotorGearRatio,
                moduleConstants[0].DriveMotorInitialConfigs.CurrentLimits.StatorCurrentLimit, 1);
        RobotConfig robotConfig = new RobotConfig(robotWeightKG, robotMOI, moduleConfig, drivetrain.getModuleLocations());

        constants = new Drivetrain.Constants(maxModuleVelocity,
                maxModuleVelocity / maxDriveBaseRadius, robotConfig);
    }

    @Override
    public Drivetrain.Constants getConstants() {
        return constants;
    }

    @Override
    public void updateInputs(DrivetrainInputs inputs) {
        // Updates inputs and robot sim state (if in simulation)
        if (Robot.isSimulation()) {
            drivetrain.updateSimState(Robot.kDefaultPeriod, RobotController.getBatteryVoltage());
        }

        var state = lastState;

        inputs.successfulDaqs = state.SuccessfulDaqs;
        inputs.failedDaqs = state.FailedDaqs;
        inputs.odometryPeriod = state.OdometryPeriod;

        inputs.pose = state.Pose;
        inputs.yaw = new Rotation2d(drivetrain.getPigeon2().getYaw().getValue().in(Units.Radians) + yawOffset);
        inputs.chassisSpeeds = state.Speeds;
        inputs.moduleStates = state.ModuleStates;
        inputs.moduleTargets = state.ModuleTargets;
        inputs.slip= drivetrain.getModule(0).getDriveMotor().getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void driveSysIdTranslation(Voltage voltage) {
        drivetrain.setControl(sysIdTranslationRequest.withVolts(voltage));
    }

    @Override
    public void resetOdometry(Pose2d newPose) {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            Rotation2d rotation = switch (alliance.get()) {
                case Blue -> Rotation2d.kZero;
                case Red -> Rotation2d.k180deg;
            };
            drivetrain.setOperatorPerspectiveForward(rotation);
        }

        yawOffset = newPose.getRotation().getRadians() - drivetrain.getPigeon2().getYaw().getValue().in(Units.Radians);

        drivetrain.resetPose(newPose);
    }

    @Override
    public void followTrajectory(SwerveSample sample) {
        this.driveFieldOriented(sample);
    }

    @Override
    public void resetFieldOriented() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            Rotation2d rotation = switch (alliance.get()) {
                case Blue -> Rotation2d.kZero;
                case Red -> Rotation2d.k180deg;
            };
            drivetrain.resetRotation(rotation);
            drivetrain.setOperatorPerspectiveForward(rotation);

            yawOffset = rotation.getRadians() - drivetrain.getPigeon2().getYaw().getValue().in(Units.Radians);
        }
    }

    @Override
    public void driveFieldOriented(double xVelocity, double yVelocity, double angularVelocity) {
        drivetrain.setControl(fieldOrientedRequest
                .withVelocityX(xVelocity)
                .withVelocityY(yVelocity)
                .withRotationalRate(angularVelocity)
                .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.OperatorPerspective));
    }

    @Override
    public void driveBlueAllianceOriented(double xVelocity, double yVelocity, double angularVelocity) {
        drivetrain.setControl(fieldOrientedRequest
                .withVelocityX(xVelocity)
                .withVelocityY(yVelocity)
                .withRotationalRate(angularVelocity)
                .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance));
    }

    @Override
    public void driveRobotRelative(ChassisSpeeds robotSpeeds) {
        drivetrain.setControl(applyRobotSpeedsRequest
                .withSpeeds(robotSpeeds));
    }

    @Override
    public void driveFieldOriented(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose = drivetrain.getState().Pose;

        // Generate the next speeds for the robot
        double xVelocity = sample.vx + xController.calculate(pose.getX(), sample.x);
        double yVelocity = sample.vy + yController.calculate(pose.getY(), sample.y);
        double angularVelocity = sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading);

        drivetrain.setControl(fieldOrientedRequest
                .withVelocityX(xVelocity)
                .withVelocityY(yVelocity)
                .withRotationalRate(angularVelocity)
                .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
        );
    }

    @Override
    public void addVisionMeasurement(double timestamp, Pose2d pose, Matrix<N3, N1> stdDevs) {
        drivetrain.addVisionMeasurement(pose, Utils.fpgaToCurrentTime(timestamp), stdDevs);
    }

    @Override
    public void stop() {
        drivetrain.setControl(stopRequest);
    }
}
