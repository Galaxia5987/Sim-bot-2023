package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.leds.Leds;
import utils.math.Vector2;
import utils.math.spline.QuinticBezierSpline;

public enum ArmPosition {

    PICKUP(new Translation2d(-0.34, -0.13), new Translation2d(-0.34, -0.13)),
    FEEDER(new Translation2d(0.2, 0.705), new Translation2d(0.2, 0.705)),
    NEUTRAL(new Translation2d(-0.3508, 0.3976), new Translation2d(-0.4508, 0.3976)),

    TOP_SCORING(new Translation2d(1.195, 0.741), new Translation2d(1.13, 0.92)),
    MIDDLE_SCORING(new Translation2d(0.8, 0.481), new Translation2d(0.929, 0.59)),
    BOTTOM_SCORING(new Translation2d(0.2, 0.775), new Translation2d(0.2, 0.775)),

    FEEDER_CONE(new Translation2d(0.42, 0.67), new Translation2d(0.42, 0.67));

    public final Translation2d cubePose;
    public final Translation2d conePose;

    ArmPosition(Translation2d cubePose, Translation2d conePose) {
        this.cubePose = cubePose;
        this.conePose = conePose;
    }

    public ArmPosition nextState(ArmInputsLogged armInputs, ArmPosition desiredPosition) {
        switch (desiredPosition) {
            case FEEDER:
                if (armInputs.endEffectorPose.getY() < 0.2 && armInputs.endEffectorPose.getX() < 0) {
                    return NEUTRAL;
                }
                return FEEDER;
            case PICKUP:
                if (armInputs.endEffectorPose.getX() < 0 && armInputs.endEffectorPose.getY() < 0.5) {
                    return PICKUP;
                }
                return NEUTRAL;

            case TOP_SCORING:
                if (armInputs.endEffectorPose.getY() < 0.2 && armInputs.endEffectorPose.getX() < 0) {
                    return NEUTRAL;
                }
                return TOP_SCORING;
            case MIDDLE_SCORING:
                if (armInputs.endEffectorPose.getY() < 0.2 && armInputs.endEffectorPose.getX() < 0) {
                    return NEUTRAL;
                }
                return MIDDLE_SCORING;
            case BOTTOM_SCORING:
                if (armInputs.endEffectorPose.getY() < 0.2 && armInputs.endEffectorPose.getX() < 0) {
                    return NEUTRAL;
                }
                return BOTTOM_SCORING;
            case FEEDER_CONE:
                if (armInputs.endEffectorPose.getY() < 0.2 && armInputs.endEffectorPose.getX() < 0) {
                    return NEUTRAL;
                }
                return FEEDER_CONE;
            default:
                return NEUTRAL;
        }
    }

    public QuinticBezierSpline getSpline(ArmInputsLogged inputs, ArmPosition desiredPosition) {
        var middleMan = desiredPosition.nextState(inputs, desiredPosition);
        Translation2d middlePose2, middlePose1;
        if (middleMan == NEUTRAL && desiredPosition != NEUTRAL) {
            if (desiredPosition == PICKUP) {
                middlePose1 = ArmConstants.IN_ROBOT1;
                middlePose2 = Leds.getInstance().inConeMode() ? middleMan.conePose : middleMan.cubePose;
            } else {
                middlePose2 = ArmConstants.IN_ROBOT1;
                middlePose1 = Leds.getInstance().inConeMode() ? middleMan.conePose : middleMan.cubePose;
            }
        } else {
            middlePose1 = Leds.getInstance().inConeMode() ? middleMan.conePose : middleMan.cubePose;
            middlePose2 = Leds.getInstance().inConeMode() ? middleMan.conePose : middleMan.cubePose;
        }

        var middleSolution1 = Arm.getINSTANCE().getKinematics().inverseKinematics(middlePose1);
        var middleSolution2 = Arm.getINSTANCE().getKinematics().inverseKinematics(middlePose2);
        var endSolution = Arm.getINSTANCE().getKinematics().inverseKinematics(
                Leds.getInstance().inConeMode() ? desiredPosition.conePose : desiredPosition.cubePose);

        var startPosition = new Vector2(inputs.shoulderAngle, inputs.elbowAngleAbsolute); //may not be true could be relative
        var middlePosition1 = new Vector2(
                middleSolution1.shoulderAngle, middleSolution1.elbowAngle);
        var middlePosition2 = new Vector2(
                middleSolution2.shoulderAngle, middleSolution2.elbowAngle);
        var endPosition = new Vector2(
                endSolution.shoulderAngle, endSolution.elbowAngle);

//        System.out.println(middlePosition1);
//        System.out.println(middlePosition2);
        return new QuinticBezierSpline(startPosition, middlePosition1, middlePosition1, middlePosition2, middlePosition2, endPosition);
    }
}
