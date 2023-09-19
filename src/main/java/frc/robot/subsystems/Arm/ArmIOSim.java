package frc.robot.subsystems.Arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
    private final SingleJointedArmSim elbow;
    private final SingleJointedArmSim shoulder;
    private final PIDController elbowController;
    private final PIDController shoulderController;

    ArmIOSim() {
        shoulderController = new PIDController(ArmConstants.SHOULDER_P, ArmConstants.SHOULDER_I, ArmConstants.SHOULDER_D, 0.02);
        elbowController = new PIDController(ArmConstants.ELBOW_P, ArmConstants.ELBOW_I, ArmConstants.ELBOW_D, 0.02);
        elbow = new SingleJointedArmSim(DCMotor.getFalcon500(1), ArmConstants.ELBOW_GEARING, ArmConstants.ELBOW_J_KG_MPOW2, ArmConstants.ELBOW_LENGTH, ArmConstants.ELBOW_MIN_ANGLE, ArmConstants.ELBOW_MAX_ANGLE, true);
        shoulder = new SingleJointedArmSim(DCMotor.getFalcon500(1), ArmConstants.SHOULDER_GEARING, ArmConstants.SHOULDER_J_KG_MPOW2, ArmConstants.SHOULDER_LENGTH, ArmConstants.SHOULDER_MIN_ANGLE, ArmConstants.SHOULDER_MAX_ANGLE, true);
    }

    @Override
    public void setShoulderPower(double power) {
        shoulder.setInputVoltage(power * 12);
    }

    @Override
    public void setElbowPower(double power) {
        elbow.setInputVoltage(power * 12);
    }

    @Override
    public void setShoulderAngle(double angleRads) {
        double angleVoltage = shoulderController.calculate(shoulder.getAngleRads(), angleRads);
        shoulder.setInputVoltage(angleVoltage);
    }

    @Override
    public void setElbowAngle(double angleRads) {
        double angleVoltage = elbowController.calculate(elbow.getAngleRads(), angleRads);
        elbow.setInputVoltage(angleVoltage);
    }

    @Override
    public void updateInputs(ArmInputsAutoLogged inputs) {
        inputs.elbowVoltage = elbow.getOutput(0);
        inputs.elbowCurrent = elbow.getCurrentDrawAmps();
        inputs.elbowAngleAbsolute = elbow.getAngleRads();
        inputs.elbowAngleRelative = elbow.getAngleRads();
        inputs.shoulderAngleAbsolute = elbow.getAngleRads();
        inputs.shoulderAngleRelative = elbow.getAngleRads();
        inputs.shoulderCurrent = shoulder.getOutput(0);
        inputs.shoulderVoltage = shoulder.getCurrentDrawAmps();
    }
}
