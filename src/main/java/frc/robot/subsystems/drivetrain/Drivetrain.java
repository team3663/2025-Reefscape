package frc.robot.subsystems.drivetrain;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.vision.VisionMeasurement;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

@Logged
public class Drivetrain extends SubsystemBase {
    private static final double DISTANCE_THRESHOLD = Units.inchesToMeters(1.5);

    @NotLogged
    private final DrivetrainIO io;
    private final DrivetrainInputs inputs = new DrivetrainInputs();
    private final Constants constants;
    private final AutoFactory autoFactory;
    private final SysIdRoutine sysIdTranslationRoutine;

    private Pose2d targetAutoPose = new Pose2d();
    private Pose2d targetTelePose = new Pose2d();

    public Drivetrain(DrivetrainIO io) {
        this.io = io;
        this.constants = io.getConstants();

        autoFactory = new AutoFactory(
                this::getPose, // returns current robot pose
                io::resetOdometry, // resets current robot pose to provided pose 2d
                (SwerveSample sample) -> {
                    targetAutoPose = sample.getPose();

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
                        new PIDConstants(20.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(12.0, 0.0, 0.0) // Rotation PID constants
                ),
                constants.robotConfig, // The robot configuration
                () -> false,
                this // Reference to this subsystem to set requirements
        );
    }

    public Constants getConstants() {
        return constants;
    }

    public Pose2d getPose() {
        return inputs.pose;
    }

    public Rotation2d getYaw() {
        return inputs.yaw;
    }

    public AutoFactory getAutoFactory() {
        return autoFactory;
    }

    public Pose2d getTargetAutoAlignPose() {
        return targetTelePose;
    }

    public void addVisionMeasurements(List<VisionMeasurement> measurements) {
        for (VisionMeasurement measurement : measurements) {
            io.addVisionMeasurement(measurement.timestamp, measurement.estimatedPose, measurement.stdDevs);
        }
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
                () -> io.driveFieldOriented(
                        xVelocity.getAsDouble(),
                        yVelocity.getAsDouble(),
                        angularVelocity.getAsDouble()),
                // end()
                io::stop);
    }

    public Command stop() {
        return runOnce(io::stop);
    }

    /**
     * Drives the robot to a given pose fromm the robot's current position using a pathplanner path
     *
     * @param targetPose of where you want the robot to go
     * @return follows a pathplanner path command
     */
    public Command goToPosition(Supplier<Pose2d> targetPose, BooleanSupplier slowAccel, Supplier<Double> maxVelocity) {
        PIDController controller = new PIDController(7.0, 0.0, 1.0);
        PIDController rotationController = new PIDController(10.0, 0.0, 0.0);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        return startRun(
                () -> {
                    controller.reset();
                    controller.setP(slowAccel.getAsBoolean() ? 5.0 : 5.0);
                    rotationController.reset();
                    controller.setSetpoint(0.0);
                    rotationController.setSetpoint(targetPose.get().getRotation().getRadians());
                },
                () -> {
                    var target = targetPose.get();
                    var current = inputs.pose;
                    var error = target.getTranslation().minus(current.getTranslation());
                    var linearVelocity = MathUtil.clamp(
                            controller.calculate(error.getNorm()),
                            -maxVelocity.get(),
                            maxVelocity.get());
                    var velocity = new Translation2d(-linearVelocity, error.getAngle());
                    var angularVel = rotationController.calculate(
                            current.getRotation().getRadians(), target.getRotation().getRadians());

                    targetTelePose = target;

                    io.driveBlueAllianceOriented(velocity.getX(), velocity.getY(), angularVel);
                })
                .finallyDo(io::stop);
    }

    public Command goToPosition(Supplier<Pose2d> targetPose, BooleanSupplier slowAccel) {
        return goToPosition(targetPose, slowAccel, ()-> io.getConstants().maxLinearVelocity);
    }

    public Command resetOdometry(Pose2d targetPose) {
        return runOnce(()-> io.resetOdometry(targetPose));
    }
    public boolean atTargetPosition() {
        return atPosition(targetTelePose.getTranslation());
    }

    public boolean atPosition(Translation2d target, double threshold) {
        return target.minus(inputs.pose.getTranslation()).getNorm() < threshold;
    }

    public boolean atPosition(Translation2d target) {
        return atPosition(target, DISTANCE_THRESHOLD);
    }

    public record Constants(
            double maxLinearVelocity,
            double maxAngularVelocity,
            RobotConfig robotConfig
    ) {
    }
}
