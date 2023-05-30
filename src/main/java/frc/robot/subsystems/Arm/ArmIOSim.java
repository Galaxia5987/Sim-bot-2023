package frc.robot.subsystems.Arm;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
    private final SingleJointedArmSim elbow;
    private final SingleJointedArmSim shoulder;

    private ArmIOSim() {
        elbow = new SingleJointedArmSim(DCMotor.getFalcon500());
        shoulder = new SingleJointedArmSim(DCMotor.getFalcon500());
    }

    @Override
    public void setShoulderPower(double power) {
        
    }

    @Override
    public void setElbowPower(double power) {
        ArmIO.super.setElbowPower(power);
    }

    @Override
    public void setShoulderAngle(double angleRads) {
        ArmIO.super.setShoulderAngle(angleRads);
    }

    @Override
    public void setElbowAngle(double angleRads) {
        ArmIO.super.setElbowAngle(angleRads);
    }

    @Override
    public void setArmIntoPosition(Translation2d position) {
        ArmIO.super.setArmIntoPosition(position);
    }
}
