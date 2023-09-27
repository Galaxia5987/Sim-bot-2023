package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.SystemState;

public enum ArmPosition implements SystemState<ArmPosition, ArmInputs> {

    PICKUP(new Translation2d(-0.34, -0.13), new Translation2d(-0.34, -0.13)),
    FEEDER(new Translation2d(0.2, 0.775), new Translation2d(0.2, 0.775)),
    NEUTRAL(new Translation2d(-0.3508, 0.3976), new Translation2d(-0.4508, 0.3976)),

    TOP_SCORING(new Translation2d(1.195, 0.741), new Translation2d(1.13, 0.92)),
    MIDDLE_SCORING(new Translation2d(0.8, 0.481), new Translation2d(0.929, 0.59)),
    BOTTOM_SCORING(new Translation2d(0.2, 0.775), new Translation2d(0.2, 0.775));

    public final Translation2d cubePose;
    public final Translation2d conePose;

    ArmPosition(Translation2d cubePose, Translation2d conePose) {
        this.cubePose = cubePose;
        this.conePose = conePose;
    }

    @Override
    public ArmPosition nextState(ArmInputs armInputs) {
        switch (armInputs.desiredArmPosition) {
            case FEEDER:
                if (armInputs.armPosition[1] < 0.2 && armInputs.armPosition[0] < 0) {
                    return NEUTRAL;
                }
                return FEEDER;
            case PICKUP:
                if (armInputs.armPosition[0] < 0 && armInputs.armPosition[1] < 0.5) {
                    return PICKUP;
                }
                return NEUTRAL;

            case TOP_SCORING:
                if (armInputs.armPosition[1] < 0.2 && armInputs.armPosition[0] < 0) {
                    return NEUTRAL;
                }
                return TOP_SCORING;
            case MIDDLE_SCORING:
                if (armInputs.armPosition[1] < 0.2 && armInputs.armPosition[0] < 0) {
                    return NEUTRAL;
                }
                return MIDDLE_SCORING;
            case BOTTOM_SCORING:
                if (armInputs.armPosition[1] < 0.2 && armInputs.armPosition[0] < 0) {
                    return NEUTRAL;
                }
                return BOTTOM_SCORING;
            case NEUTRAL:
                return NEUTRAL;
            default:
                return NEUTRAL;
        }
    }
}
