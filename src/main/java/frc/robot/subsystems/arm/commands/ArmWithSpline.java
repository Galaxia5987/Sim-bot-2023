package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPosition;
import lib.math.Vector2;
import lib.math.spline.QuinticBezierSpline;

public class ArmWithSpline extends Command {

    private final Arm arm = Arm.getINSTANCE();
    private final ArmPosition desiredPosition;
    private final Timer timer = new Timer();
    private QuinticBezierSpline spline;
    private double shoulderDistance = 90;
    private double elbowDistance = 180;

    public ArmWithSpline(ArmPosition desiredPosition) {
        this.desiredPosition = desiredPosition;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
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

        arm.setShoulderAngle(positionShoulder);
        arm.setElbowAngleRelative(positionElbow);
    }
}
