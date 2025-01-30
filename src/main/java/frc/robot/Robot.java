// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.config.C2024RobotFactory;
import frc.robot.config.C2025RobotFactory;
import frc.robot.config.RobotFactory;
import frc.robot.utility.MacAddressUtils;
import frc.robot.utility.RobotId;

import java.util.Arrays;
import java.util.Set;

@Logged
public class Robot extends TimedRobot {
    /**
     * The detected MAC addresses on the system.
     */
    private final Set<String> macAddresses = MacAddressUtils.getMacAddresses();

    /**
     * The ID of the robot that was detected based on the system's MAC addresses.
     * <p>
     * May be {@code null} if the system did not have a recognized MAC address.
     */
    private final RobotId detectedId = Arrays.stream(RobotId.values())
            .filter(id -> macAddresses.contains(id.getMacAddress()))
            .findFirst().orElse(RobotId.UNKNOWN);

    @NotLogged
    private Command autonomousCommand;

    private final RobotContainer robotContainer;

    public Robot() {
        RobotFactory robotFactory = switch (detectedId) {
            case C2024 -> new C2024RobotFactory();
            case C2025 -> new C2025RobotFactory();
            default -> new C2024RobotFactory();
        };

        robotContainer = new RobotContainer(robotFactory);

        Epilogue.configure(config -> {
        });
        Epilogue.bind(this);

        SignalLogger.setPath("/media/sda1/");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
