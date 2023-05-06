package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drive;

public class JoystickDrive extends CommandBase {

    private final Drive drive = Drive.getInstance();

    private final Joystick leftJoystick;
    private final Joystick rightJoystick;

    public JoystickDrive(Joystick leftJoystick, Joystick rightJoystick) {
        this.leftJoystick = leftJoystick;
        this.rightJoystick = rightJoystick;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.drive(
                -leftJoystick.getY(),
                leftJoystick.getX(),
                rightJoystick.getX(),
                true
        );
    }
}
