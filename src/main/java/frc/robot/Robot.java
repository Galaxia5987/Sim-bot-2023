// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.utils.TunableNumber;
import frc.robot.utils.math.differential.BooleanTrigger;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    public static boolean debug = false;
    private static final BooleanTrigger enabledTrigger = new BooleanTrigger(false, false);
    private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    private final Timer timer = new Timer();
    private RobotContainer robotContainer;
    private Command autonomousCommand;
    private BooleanTrigger encoderTrigger = new BooleanTrigger(false, false);
    private boolean updated = false;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
//        compressor.disable();
        PathPlannerServer.startServer(5811);
        robotContainer = RobotContainer.getInstance();

        Logger.getInstance().recordMetadata("ProjectName", "Sim-bot-2023"); // Set a metadata value

        if (isReal()) {
            Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            Logger.getInstance().addDataReceiver(new WPILOGWriter("home/lvuser")); // Publish data to NetworkTables
            new PowerDistribution(1, PowerDistribution.ModuleType.kRev); // Enables power distribution logging
        } else {
            Logger.getInstance().addDataReceiver(new NT4Publisher());
        }

        Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

        timer.start();
        timer.reset();

        Logger.getInstance().recordOutput("BottomArmPose", new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(0))));
        Logger.getInstance().recordOutput("TopArmPose", new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(0))));
        Logger.getInstance().recordOutput("IntakePose", new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(Math.toRadians(-90), Math.toRadians(0), Math.toRadians(0))));
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        TunableNumber.INSTANCES.forEach(TunableNumber::update);
        CommandScheduler.getInstance().run();

        enabledTrigger.update(isEnabled());
        encoderTrigger.update(SwerveDrive.getInstance().encodersConnected());

        if (encoderTrigger.triggered()) {
            SwerveDrive.getInstance().updateOffsets(SwerveConstants.OFFSETS);
//            updated = true;
        }
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select between different
     * autonomous modes using the dashboard. The sendable chooser code works with the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
     * uncomment the getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to the switch structure
     * below with additional strings. If using the SendableChooser make sure to add them to the
     * chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {

    }

    /**
     * This function is called once when teleop is enabled.
     */
    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
    }

    /**
     * This function is called once when the robot is disabled.
     */
    @Override
    public void disabledInit() {
    }

    /**
     * This function is called periodically when disabled.
     */
    @Override
    public void disabledPeriodic() {
    }

    /**
     * This function is called once when test mode is enabled.
     */
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

    public static boolean justEnabled() {
        return enabledTrigger.triggered();
    }
}
