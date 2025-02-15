package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;

@Logged
public class DrivetrainInputs {
    public int successfulDaqs = 0;
    public int failedDaqs = 0;
    public double odometryPeriod = 0.0;

    public Pose2d pose = new Pose2d();
    public Rotation2d yaw= new Rotation2d();
    public ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    public SwerveModuleState[] moduleStates = new SwerveModuleState[0];
    public SwerveModuleState[] moduleTargets = new SwerveModuleState[0];

}
