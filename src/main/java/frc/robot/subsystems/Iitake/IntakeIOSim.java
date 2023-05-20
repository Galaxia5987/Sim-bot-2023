package frc.robot.subsystems.Iitake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim implements IntakeIO{
    private final SingleJointedArmSim angleMotor;
    private final FlywheelSim powerMotor;
    private double angleVoltage = 0;
    private final PIDController angleController;
    public IntakeIOSim() {
        angleController = new PIDController(0,0,0, 0.02);
        angleMotor = new SingleJointedArmSim(DCMotor.getNEO(1));
        powerMotor = new FlywheelSim(DCMotor.getFalcon500(1));
    }

    public void updateInputs(IntakeInputs inputs) {
        inputs.motorPower = powerMotor.getAngularVelocityRPM();
        inputs.motorAngle = angleMotor.getAngleRads();
    }

    public double getPowerMotor() {
        return powerMotor.getAngularVelocityRadPerSec();
    }

    public double getAngleMotor(){
        return angleMotor.getAngleRads();
    }

    public void setMotorPower(double power) {
        powerMotor.setInputVoltage(power);
    }

    public void setAngleMotor(double desiredAngleRads) {
        angleVoltage = angleController.calculate(getAngleMotor(), desiredAngleRads);
        angleMotor.setInputVoltage(angleVoltage);
    }


}
