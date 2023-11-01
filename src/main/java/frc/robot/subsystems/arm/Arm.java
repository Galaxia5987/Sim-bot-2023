package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    static Arm INSTANCE;
    private ArmIO io;
    private ArmInputsAutoLogged inputs;

    private Arm(ArmIO io) {
        this.io = io;
    }

    public static Arm getINSTANCE() {
        if (INSTANCE == null) {
            INSTANCE = new Arm(new ArmIO());
        }

        return INSTANCE;

    }

    private void setElbowPower(double power) {
        inputs.elbowAppliedVoltage = power * 12;
    }

    private void setElbowAngleRelative(double angle) {
        inputs.elbowAngleRelative = angle;
    }

    private void setShoulderPower(double power) {
        inputs.elbowAppliedVoltage = power * 12;
    }

    private void setShoulderAngle(double angle) {
        inputs.elbowAngleRelative = angle;
    }

    @Override
    public void periodic() {
        io.setElbowAngle(inputs.elbowAngleRelative);
        io.setElbowPower(inputs.elbowAppliedVoltage);
        io.setShoulderAngle(inputs.shoulderAngle);
        io.setShoulderPower(inputs.shoulderAppliedVoltage);
        io.updateInputs();
    }
}
