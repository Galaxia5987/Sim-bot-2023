package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final IntakeIO intakeIO;

    private IntakeIO.IntakeInputs inputs;

    public Intake (IntakeIO intakeIO){
        this.intakeIO = intakeIO;
    }

    public void setAngleMotorPower(double power){
        intakeIO.setAngleMotorPower(power);
    }

    public void setAngle(double angle){
        intakeIO.setAngle(angle);
    }

    public void setSpinMotorPower(double power){
        intakeIO.setSpinMotorPower(power);
    }

    @Override
    public void periodic() {
        intakeIO.updateInputs(inputs);
    }


}
