package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final IntakeIO intakeIO;

    public void setAngleMotorPower(double power){
        intakeIO.setAngleMotorPower(power);
    }

    public void setAngle(double angle){
        intakeIO.setAngle(angle);
    }

    public void setSpinMotorPower(double power){
        intakeIO.setSpinMotorPower(power);
    }
}
