package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import utils.motors.TalonFXSim;

public class IntakeIOSim implements IntakeIO {
    private final TalonFXSim angleMotorSim;
    private final TalonFXSim powerMotorSim;
    private final TalonFXConfiguration angleConfiguration;
    private final PIDController angleController = new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);

    public IntakeIOSim() {
        angleConfiguration = new TalonFXConfiguration();

        angleMotorSim = new TalonFXSim(1, IntakeConstants.ANGLE_GEAR_RATIO, 0.00001);
        powerMotorSim = new TalonFXSim(1, 1, 0.00001);
        angleMotorSim.setController(angleController);

        angleConfiguration.Slot0 = IntakeConstants.PIDGains;
//        angleMotorSim.configure(angleConfiguration);
    }


    @Override
    public void setSpinMotorPower(double power) {
        powerMotorSim.setControl(new DutyCycleOut(power));
    }

    @Override
    public void setAngleMotorAngle(Rotation2d angle) {
        angleMotorSim.setControl(new PositionVoltage(angle.getRotations()));
    }

    @Override
    public void setAngleMotorPower(double power) {
        angleMotorSim.setControl(new DutyCycleOut(power));
    }


    @Override
    public void resetEncoder(double angle) {

    }

    @Override
    public void updateInputs(IntakeLoggedInputs inputs) {
        //var batterySim = new BatterySim();
        angleMotorSim.update(Timer.getFPGATimestamp());
        powerMotorSim.update(Timer.getFPGATimestamp());

        inputs.angleMotorPower = angleMotorSim.getAppliedVoltage() / 12;
        inputs.angleMotorAppliedCurrent = angleMotorSim.getAppliedCurrent();
        inputs.angleMotorAppliedVoltage = angleMotorSim.getAppliedVoltage();
        inputs.angleMotorVelocity = Rotation2d.fromRotations(angleMotorSim.getRotorVelocity());
        inputs.angleMotorAngle = Rotation2d.fromRotations(angleMotorSim.getRotorPosition());
        inputs.spinMotorAppliedCurrent = powerMotorSim.getAppliedVoltage();
        inputs.spinMotorPower = powerMotorSim.getAppliedVoltage() / 12;

    }

}
