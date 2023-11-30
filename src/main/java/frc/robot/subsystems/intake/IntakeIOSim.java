package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim implements IntakeIO {
    private final FlywheelSim spinMotor;
    private final SingleJointedArmSim angleMotor;
    private final PIDController angleFeedback;

    public IntakeIOSim() {
        spinMotor = new FlywheelSim(DCMotor.getNEO(1), IntakeConstants.SPIN_GEAR_RATIO, 1);
        angleMotor = new SingleJointedArmSim(DCMotor.getFalcon500(1), IntakeConstants.ANGLE_GEAR_RATIO, 1.7, 42, -Math.PI * 2, Math.PI * 2, false); //could be wrong type of motor or max/min angle
        angleFeedback = new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);
    }

    @Override
    public void updateInputs(IntakeLoggedInputs inputs) {
        inputs.spinMotorPower = inputs.setpointSpinMotorPower;
        inputs.spinMotorCurrent = spinMotor.getCurrentDrawAmps();
        inputs.angleMotorAngle = angleMotor.getAngleRads();
        inputs.angleMotorcurrent = angleMotor.getCurrentDrawAmps();

        spinMotor.update(0.02);
        angleMotor.update(0.02);
    }

    @Override
    public void setSpinMotorPower(double power) {
        spinMotor.setInputVoltage(power * 12);
    }

    @Override
    public void setAngleMotorAngle(double angle) {
        double angleMotorAppliedVoltage = angleFeedback.calculate(angleMotor.getAngleRads(), angle);
        angleMotor.setInputVoltage(angleMotorAppliedVoltage);
    }

    @Override
    public void setAngleMotorPower(double power) {
        angleMotor.setInputVoltage(power * 12);
    }

    @Override
    public void resetEncoder(double angle) {

    }
}
