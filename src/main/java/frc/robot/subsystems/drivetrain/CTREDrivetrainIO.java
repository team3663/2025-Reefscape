package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Robot;

public class CTREDrivetrainIO implements DrivetrainIO {
    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain;

    private final SwerveRequest.FieldCentric fieldOrientedRequest = new SwerveRequest.FieldCentric();
    private final SwerveRequest.Idle stopRequest = new SwerveRequest.Idle();

    private volatile SwerveDrivetrain.SwerveDriveState lastState = new SwerveDrivetrain.SwerveDriveState();

    @SafeVarargs
    public CTREDrivetrainIO(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>... moduleConstants
    ) {
        this.drivetrain = new SwerveDrivetrain<>(
                TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, moduleConstants
        );

        drivetrain.registerTelemetry(state -> lastState = state.clone());
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


    }

    @Override
    public void driveFieldOriented(double xVelocity, double yVeolcity, double angularVeolcity) {
        drivetrain.setControl(fieldOrientedRequest
                .withVelocityX(xVelocity)
                .withVelocityY(yVeolcity)
                .withRotationalRate(angularVeolcity));
    }

    @Override
    public void stop() {
        drivetrain.setControl(stopRequest);
    }
}
