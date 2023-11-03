package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Arm extends SubsystemBase {
    static Arm INSTANCE;
    private final ArmIO io;
    private ArmInputsAutoLogged inputs;

    private Arm(ArmIO io) {
        this.io = io;
    }

    public static Arm getINSTANCE() {
        if (INSTANCE == null) {
            if (Robot.isReal()) {
                INSTANCE = new Arm(new ArmIOSim(new ArmInputsAutoLogged()));

            } else {
                INSTANCE = new Arm(new ArmIOSim(new ArmInputsAutoLogged()));
            }
        }

        return INSTANCE;

    }

    public void setElbowPower(double power) {
        inputs.elbowAppliedVoltage = power * 12;
    }

    public void setElbowAngleRelative(double angle) {
        inputs.elbowAngleRelative = angle;
    }

    public void setShoulderPower(double power) {
        inputs.elbowAppliedVoltage = power * 12;
    }

    public void setShoulderAngle(double angle) {
        inputs.elbowAngleRelative = angle;
    }

    public void setEndEffectorPosition(Translation2d endEffectorPosition, ArmKinematics armKinematics){
        inputs.endEffectorPosition = new double[]{endEffectorPosition.getX(), endEffectorPosition.getY()};
    }

    @Override
    public void periodic() {
        io.setElbowAngle(inputs.elbowAngleRelative);
        io.setElbowPower(inputs.elbowAppliedVoltage);
        io.setShoulderAngle(inputs.shoulderAngle);
        io.setShoulderPower(inputs.shoulderAppliedVoltage);
        io.setEndEffectorPosition(new Translation2d(inputs.endEffectorPosition[0],inputs.endEffectorPosition[1] ) , ArmIO.armKenematics);
        io.updateInputs();
    }
}
