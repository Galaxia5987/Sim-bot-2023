package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.commands.ArmKeyboardControl;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.KeyboardDriveSim;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeKeyboardControl;
import frc.robot.subsystems.leds.Leds;
import frc.robot.utils.Utils;

public class RobotContainer {
    private static final Arm arm = Arm.getINSTANCE();
    private static RobotContainer INSTANCE = null;
    private final Leds leds = Leds.getInstance();
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Gripper gripper = Gripper.getInstance();
    private final XboxController xboxController = new XboxController(0);
    private final Joystick leftJoystick = new Joystick(1);
    private final Joystick rightJoystick = new Joystick(2);
    private final JoystickButton a = new JoystickButton(xboxController, XboxController.Button.kA.value);
    private final JoystickButton b = new JoystickButton(xboxController, XboxController.Button.kB.value);
    private final JoystickButton y = new JoystickButton(xboxController, XboxController.Button.kY.value);
    private final JoystickButton x = new JoystickButton(xboxController, XboxController.Button.kX.value);
    private final JoystickButton back = new JoystickButton(xboxController, XboxController.Button.kBack.value);
    private final JoystickButton rb = new JoystickButton(xboxController, XboxController.Button.kRightBumper.value);
    private final JoystickButton lb = new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value);
    private final Trigger xboxRightTrigger = new Trigger(() -> xboxController.getRightTriggerAxis() > 0.2);
    private final Trigger xboxLeftTrigger = new Trigger(() -> xboxController.getLeftTriggerAxis() > 0.2);
    private final JoystickButton leftJoystickTrigger = new JoystickButton(leftJoystick, Ports.UI.JOYSTICK_TRIGGER);
    private final JoystickButton leftJoystickTopBottom = new JoystickButton(leftJoystick, Ports.UI.JOYSTICK_TOP_BOTTOM_BUTTON);
    private final JoystickButton leftJoystickTopRight = new JoystickButton(leftJoystick, Ports.UI.JOYSTICK_TOP_RIGHT_BUTTON);
    private final JoystickButton leftJoystickTopLeft = new JoystickButton(leftJoystick, Ports.UI.JOYSTICK_TOP_LEFT_BUTTON);
    private final JoystickButton rightJoystickTrigger = new JoystickButton(rightJoystick, Ports.UI.JOYSTICK_TRIGGER);
    private final JoystickButton rightJoystickTopBottom = new JoystickButton(rightJoystick, Ports.UI.JOYSTICK_TOP_BOTTOM_BUTTON);
    private final JoystickButton rightJoystickTopRight = new JoystickButton(rightJoystick, Ports.UI.JOYSTICK_TOP_RIGHT_BUTTON);
    private final JoystickButton rightJoystickTopLeft = new JoystickButton(rightJoystick, Ports.UI.JOYSTICK_TOP_LEFT_BUTTON);
    private final Trigger leftPOV = new Trigger(() -> xboxController.getPOV() == 270);
    private final Trigger rightPOV = new Trigger(() -> xboxController.getPOV() == 90);
    private final Trigger upPOV = new Trigger(() -> Utils.epsilonEquals(xboxController.getPOV(), 0));
    private final Trigger downPOV = new Trigger(() -> Utils.epsilonEquals(xboxController.getPOV(), 180));
    private final JoystickButton start = new JoystickButton(xboxController, XboxController.Button.kStart.value);

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    private RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
    }

    public static RobotContainer getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new RobotContainer();
        }
        return INSTANCE;
    }

    private void configureDefaultCommands() {
        arm.setDefaultCommand(new ArmKeyboardControl());
        intake.setDefaultCommand(new IntakeKeyboardControl());
        swerveDrive.setDefaultCommand(new KeyboardDriveSim());
    }

    private void configureButtonBindings() {

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
