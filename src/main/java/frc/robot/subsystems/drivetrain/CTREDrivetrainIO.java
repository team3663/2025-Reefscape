package frc.robot.subsystems.drivetrain;

import choreo.trajectory.SwerveSample;
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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Robot;

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

    @SafeVarargs
    public CTREDrivetrainIO(
            double robotWeightKG,
            double robotMOI,
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>... moduleConstants
    ) {
        this.drivetrain = new SwerveDrivetrain<>(
                TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, moduleConstants
        );
        RobotConfig robotConfig;
        ModuleConfig moduleConfig;

        //TODO FIX!!!
        moduleConfig = new ModuleConfig(Constants.MK4_WHEEL_RADIUS, 5.0,
                Constants.WHEEL_COF, DCMotor.getKrakenX60Foc(1), moduleConstants[0].DriveMotorGearRatio,
                moduleConstants[0].DriveMotorInitialConfigs.CurrentLimits.StatorCurrentLimit, 4);

        robotConfig = new RobotConfig(robotWeightKG, robotMOI, moduleConfig, drivetrain.getModuleLocations());

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
        inputs.chassisSpeeds = state.Speeds;
        inputs.moduleStates = state.ModuleStates;
        inputs.moduleTargets = state.ModuleTargets;

        ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation());
    }

    @Override
    public void driveSysIdTranslation(Voltage voltage) {
        drivetrain.setControl(sysIdTranslationRequest.withVolts(voltage));
    }

    @Override
    public void resetOdometry(Pose2d newPose) {
        drivetrain.resetPose(newPose);
    }

    @Override
    public void followTrajectory(SwerveSample sample) {
        this.driveFieldOriented(sample);
    }

    @Override
    public void resetFieldOriented() {
        drivetrain.seedFieldCentric();
    }

    @Override
    public void driveFieldOriented(double xVelocity, double yVeolcity, double angularVeolcity) {
        drivetrain.setControl(fieldOrientedRequest
                .withVelocityX(xVelocity)
                .withVelocityY(yVeolcity)
                .withRotationalRate(angularVeolcity));
    }

    @Override
    public void driveRobotRelative(ChassisSpeeds robotSpeeds){
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
        );
    }

    @Override
    public void stop() {
        drivetrain.setControl(stopRequest);
    }
}
