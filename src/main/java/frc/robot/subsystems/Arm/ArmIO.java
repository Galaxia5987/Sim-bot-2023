package frc.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    default void setShoulderPower(double power) {
    }

    default void setElbowPower(double power) {
    }

    default void setShoulderAngle(double angleRads) {
    }

    default void setElbowAngle(double angleRads) {
    }

    void updateInputs(ArmInputsAutoLogged inputs);

    @AutoLog
    class ArmInputs {
        double shoulderAngleRelative;
        double shoulderAngleAbsolute;
        double elbowVoltage = 0;
        double elbowCurrent = 0;
        double elbowAngleRelative = 0;
        double elbowAngleAbsolute = 0;
        double shoulderVoltage = 0;
        double shoulderCurrent = 0;
        double shoulderAngle = 0;
        Translation2d tipPosition = new Translation2d(0, 0);
    }
}
