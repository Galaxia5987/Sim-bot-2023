package frc.robot.subsystems.Arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
    private final SingleJointedArmSim elbow;
    private final SingleJointedArmSim shoulder;
    private final PIDController angleController;

    private ArmIOSim() {
        angleController = new PIDController(0, 0, 0, 0.02);
        elbow = new SingleJointedArmSim(DCMotor.getFalcon500(1), ArmConstants.ELBOW_GEARING, ArmConstants.ELBOW_J_KG_MPOW2, ArmConstants.ELBOW_LENGTH, ArmConstants.ELBOW_MIN_ANGLE, ArmConstants.ELBOW_MAX_ANGLE, true);
        shoulder = new SingleJointedArmSim(DCMotor.getFalcon500(1), ArmConstants.SHOULDER_GEARING, ArmConstants.SHOULDER_J_KG_MPOW2, ArmConstants.SHOULDER_J_KG_MPOW2, ArmConstants.SHOULDER_MIN_ANGLE, ArmConstants.SHOULDER_MAX_ANGLE, true);
    }

    @Override
    public void setShoulderPower(double power) {
        shoulder.setInputVoltage(power);
    }

    @Override
    public void setElbowPower(double power) {
        elbow.setInputVoltage(power);
    }

    @Override
    public void setShoulderAngle(double angleRads) {
        double angleVoltage = angleController.calculate(shoulder.getAngleRads(), angleRads);
        shoulder.setInputVoltage(angleVoltage);
    }

    @Override
    public void setElbowAngle(double angleRads) {
        double angleVoltage = angleController.calculate(elbow.getAngleRads(), angleRads);
        elbow.setInputVoltage(angleVoltage);
    }

    @Override
    public void setTipPosition(Pose2d tIpPosition) {

    }

    @Override
    public void setTipPosition(Translation2d position) {
        ArmIO.super.setTipPosition(position);
    }
}
