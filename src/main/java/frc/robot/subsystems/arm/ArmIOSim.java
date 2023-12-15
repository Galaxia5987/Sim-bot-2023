package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
    private final ArmInputsLogged inputs;
    private final SingleJointedArmSim shoulder;
    private final SingleJointedArmSim elbow;
    private final PIDController shoulderController = new PIDController(ArmConstants.shoulderP, ArmConstants.shoulderI, ArmConstants.shoulderD, 0.02);
    private final PIDController elbowController = new PIDController(ArmConstants.elbowP, ArmConstants.elbowI, ArmConstants.elbowD, 0.02);
    private final double angleCalculated = 0;

    public ArmIOSim(ArmInputsLogged inputs) {
        this.inputs = inputs;
        shoulder = new SingleJointedArmSim(LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getFalcon500(2), ArmConstants.SHOULDER_MOMENT_OF_INERTIA, ArmConstants.SHOULDER_GEARING
        ), DCMotor.getFalcon500(2), ArmConstants.SHOULDER_GEARING, ArmConstants.SHOULDER_ARM_LENGTH, Math.toRadians(-60), Math.toRadians(150), false, 0);
        elbow = new SingleJointedArmSim(LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getFalcon500(2), ArmConstants.ELBOW_MOMENT_OF_INERTIA, ArmConstants.ELBOW_GEARING
        ), DCMotor.getFalcon500(2), ArmConstants.ELBOW_GEARING, ArmConstants.ELBOW_ARM_LENGTH, Math.toRadians(-360), Math.toRadians(360), false, 0);
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
    public void setShoulderAngle(double angle) {
        inputs.shoulderAppliedVoltage = shoulderController.calculate(inputs.shoulderAngle, angle);
        shoulder.setInputVoltage(inputs.shoulderAppliedVoltage);

    }

    @Override

    public void setElbowAngle(double angle) {
        inputs.elbowAppliedVoltage = elbowController.calculate(inputs.elbowAngleRelative, angle);
        elbow.setInputVoltage(inputs.elbowAppliedVoltage);
    }

    @Override
    public void setEndEffectorPosition(Translation2d position, ArmKinematics armKinematics) {
        setElbowAngle(armKinematics.inverseKinematics(position).elbowAngle);
        setShoulderAngle(armKinematics.inverseKinematics(position).shoulderAngle);
    }

    public void setElbowP(double kP) {
        elbowController.setP(kP);
    }


    @Override
    public String getSubsystemName() {
        return "arm";
    }

    @Override
    public void updateInputs(ArmInputsLogged inputs) {
        shoulder.update(0.02);
        elbow.update(0.02);
        inputs.shoulderAngle = shoulder.getAngleRads();
        inputs.shoulderTipPose = new double[]{Math.cos(inputs.shoulderAngle) * ArmConstants.SHOULDER_LENGTH, Math.sin(inputs.shoulderAngle) * ArmConstants.SHOULDER_LENGTH};
        inputs.elbowAngleRelative = elbow.getAngleRads();
        inputs.elbowAngleAbsolute = inputs.elbowAngleRelative + inputs.shoulderAngle;
        inputs.elbowAppliedCurrent = elbow.getCurrentDrawAmps();
        inputs.shoulderAppliedCurrent = shoulder.getCurrentDrawAmps();
        inputs.endEffectorPose = armKinematics.forwardKinematics(inputs.shoulderAngle, inputs.elbowAngleRelative);
    }
}
