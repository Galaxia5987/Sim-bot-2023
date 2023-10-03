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

    private final Timer timer = new Timer();

    private double shoulderDistance = 90;
    private double elbowDistance = 180;

    public ArmWithSpline(ArmPosition desiredPosition) {
        this.desiredPosition = desiredPosition;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setDesiredPosition(desiredPosition);

        spline = desiredPosition.getSpline(arm.getInputs(), desiredPosition);
        Vector2[] controlPoints = spline.getControlPoints();
        shoulderDistance = Math.abs(controlPoints[0].x - controlPoints[controlPoints.length - 1].x);
        elbowDistance = Math.abs(controlPoints[0].y - controlPoints[controlPoints.length - 1].y);

        timer.start();
        timer.reset();
    }

    @Override
    public void execute() {
        double multiplier = Math.min(90 / shoulderDistance, 180 / elbowDistance);
        var positionShoulder = spline.getPoint(MathUtil.clamp(timer.get() * multiplier, 0, 1)).x;
        var positionElbow = spline.getPoint(MathUtil.clamp(timer.get() * multiplier, 0, 1)).y;

        arm.setShoulderJointAngle(positionShoulder);
        arm.setElbowJointAngle(positionElbow);
    }
}
