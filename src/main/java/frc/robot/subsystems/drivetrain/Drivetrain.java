package frc.robot.subsystems.drivetrain;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

@Logged
public class Drivetrain extends SubsystemBase {
    @NotLogged
    private final DrivetrainIO io;
    private final DrivetrainInputs inputs = new DrivetrainInputs();
    private final Constants constants;
    private final AutoFactory autoFactory;
    private final SysIdRoutine sysIdTranslationRoutine;

    private Pose2d targetPathPose = new Pose2d();

    public Drivetrain(DrivetrainIO io) {
        this.io = io;
        this.constants = io.getConstants();

        autoFactory = new AutoFactory(
                this::getPose, // returns current robot pose
                io::resetOdometry, // resets current robot pose to provided pose 2d
                (SwerveSample sample) -> {
                    targetPathPose = sample.getPose();

                    io.followTrajectory(sample);
                }, // trajectory follower
                true,
                this
        );

        // Creating a SysId Routine
        sysIdTranslationRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(0.25).per(Second),
                        Volts.of(4),
                        null,
                        (state) -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism(
                        io::driveSysIdTranslation,
                        null,
                        this));
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                () -> inputs.pose, // Robot pose supplier
                io::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                () -> inputs.chassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                io::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                constants.robotConfig, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    } else {
                        return false;
                    }
                },
                this // Reference to this subsystem to set requirements
        );
    }

    public Constants getConstants() {
        return constants;
    }

    public Pose2d getPose() {
        return inputs.pose;
    }

    public Rotation2d robotDirection(){
        return new Translation2d(inputs.pose.getX(), inputs.pose.getY()).getAngle();
        //return Math.atan2(inputs.pose.getY(), inputs.pose.getX());
    }

    public double getMaxDriveVelocityMPS() {
        return constants.maxLinearVelocity;
    }

    public AutoFactory getAutoFactory() {
        return autoFactory;
    }


    @Override
    public void periodic() {
        // Updates every 20 milliseconds
        io.updateInputs(inputs);
    }

    public Command resetFieldOriented() {
        return runOnce(io::resetFieldOriented);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdTranslationRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdTranslationRoutine.dynamic(direction);
    }

    /**
     * Drives field Oriented with the ability to specify X, Y, and Angular Velocities
     *
     * @param xVelocity       The target X (forwards) velocity in meters  per second.
     * @param yVelocity       The target Y (towards the left side of the robot) velocity in meters per second.
     * @param angularVelocity The target angular (counter-clockwise positive) velocity in radians per second.
     */
    public Command drive(DoubleSupplier xVelocity, DoubleSupplier yVelocity, DoubleSupplier angularVelocity) {
        return runEnd(
                // execute()
                () -> {
                    io.driveFieldOriented(
                            xVelocity.getAsDouble(),
                            yVelocity.getAsDouble(),
                            angularVelocity.getAsDouble());
                },
                // end()
                io::stop);
    }

    public record Constants(
            double maxLinearVelocity,
            double maxAngularVelocity,
            RobotConfig robotConfig
    ) {
    }
}
