package frc.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    default void setShoulderPower(double power){};
    default void setElbowPower(double power){}
    default void setShoulderAngle(double angleRads){}
    default void setElbowAngle(double angleRads){}
    default void setArmIntoPosition(Translation2d position){}

    @AutoLog
    class ArmInputs{
        double elbowPower = 0;
        double elbowAngle = 0;
        double shoulderPower = 0;
        double shoulderAngle = 0;
        Translation2d position  = new Translation2d(0,0);
    }
}
