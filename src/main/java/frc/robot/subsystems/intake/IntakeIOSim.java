package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.utils.math.differential.Integral;

public class IntakeIOSim implements IntakeIO{

    private final FlywheelSim spinMotor;
    private final FlywheelSim angleMotor;

    private final PIDController angleFeedback;
    private double angleVoltage = 0;
    private double currentAngle = 0;

    private final Integral angle = new Integral(0, 0);

    public IntakeIOSim() {
        this.spinMotor = new FlywheelSim(
                DCMotor.getNEO(1), 1/IntakeConstants.SPIN_MOTOR_GEARING, IntakeConstants.SPIN_MOTOR_MOMENT_OF_INERTIA);
        this.angleMotor = new FlywheelSim(
                DCMotor.getFalcon500(1), 1/IntakeConstants.ANGLE_MOTOR_GEARING, IntakeConstants.ANGLE_MOTOR_MOMENT_OF_INERTIA
        );
        this.angleFeedback = new PIDController(0, 0, 0);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        spinMotor.update(0.02);
        angleMotor.update(0.02);

        angle.update(angleMotor.getAngularVelocityRadPerSec());
        inputs.angle = angle.get();
        currentAngle = angle.get();
        inputs.angleMotorVelocity = angleMotor.getAngularVelocityRadPerSec();
        inputs.spinMotorVelocity = spinMotor.getAngularVelocityRadPerSec();
        inputs.angleVoltage = angleVoltage;
    }

    @Override
    public void setAngleMotorPower(double volts) {
        angleMotor.setInputVoltage(volts);
    }

    @Override
    public void setSpinMotorPower(double volts) {
        spinMotor.setInputVoltage(volts);
    }

    @Override
    public void setAngle(double angle) {
        angleVoltage = angleFeedback.calculate(currentAngle, angle);
        angleMotor.setInputVoltage(angleVoltage);
    }
}
