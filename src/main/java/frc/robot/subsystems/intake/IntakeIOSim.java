package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim implements IntakeIO{

    private final FlywheelSim spinMotor;
    private final FlywheelSim angleMotor;

    public IntakeIOSim() {
        this.spinMotor = new FlywheelSim(
                DCMotor.getFalcon500(1), 1/IntakeConstants.GEARING, IntakeConstants.SPIN_MOTOR_MOMENT_OF_INERTIA);
        this.angleMotor = new FlywheelSim(
                DCMotor.getFalcon500(1), 1/IntakeConstants.GEARING, IntakeConstants.ANGLE_MOTOR_MOMENT_OF_INERTIA
        );
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        spinMotor.update(0.02);
        angleMotor.update(0.02);

        inputs.angleMotorVelocity = angleMotor.getAngularVelocityRadPerSec();
        inputs.spinMotorVelocity = spinMotor.getAngularVelocityRadPerSec();
    }

    @Override
    public void setAngleMotorVelocity(double velocity) {
        IntakeIO.super.setAngleMotorVelocity(velocity);
    }

    @Override
    public void setSpinMotorVelocity(double velocity) {
        IntakeIO.super.setSpinMotorVelocity(velocity);
    }

    @Override
    public void setAngle(double angle) {
        IntakeIO.super.setAngle(angle);
    }
}
