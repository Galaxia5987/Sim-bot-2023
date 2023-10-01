package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.leds.Leds;
import frc.robot.utils.math.Vector2;
import frc.robot.utils.math.spline.CubicBezierSpline;
import frc.robot.utils.math.spline.QuinticBezierSpline;

public class ArmWithSpline extends CommandBase {

    private final Arm arm = Arm.getInstance();

    private QuinticBezierSpline spline;
    private final ArmPosition desiredPosition;

    private Vector2 startPosition;
    private Vector2 middlePositions;
    private Vector2 endPosition;

    private final Timer timer = new Timer();

    public ArmWithSpline(ArmPosition desiredPosition) {
        this.desiredPosition = desiredPosition;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setDesiredPosition(desiredPosition);

        spline = desiredPosition.getSpline(arm.getInputs(), desiredPosition);

        timer.start();
        timer.reset();
    }

    @Override
    public void execute() {
        var position = spline.getPoint(MathUtil.clamp(timer.get() / 1.2, 0, 1));

        arm.setShoulderJointAngle(position.x);
        arm.setElbowJointAngle(position.y);
    }
}
