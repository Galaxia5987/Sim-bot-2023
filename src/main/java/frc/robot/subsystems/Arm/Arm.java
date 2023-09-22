package frc.robot.subsystems.Arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    private static Arm INSTANCE;
    private final ArmInputsAutoLogged inputs = new ArmInputsAutoLogged();
    private ArmKinematics armKinematics = new ArmKinematics(ArmConstants.SHOULDER_LENGTH, ArmConstants.ELBOW_LENGTH);
    private ControlMode elbowControlMode;
    private ControlMode shoulderControlMode;
    private ArmIO io;

    private Arm() {
        if (Robot.isReal()) {
            // io = new ArmIOReal();
        } else {
            io = new ArmIOSim();
        }
    }

    public static Arm getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Arm();
        }
        return INSTANCE;
    }

    public void setShoulderPower(double power) {
        io.setShoulderPower(power);
        inputs.elbowVoltage = power * 12;
    }

    public void setElbowPower(double power) {
        io.setElbowPower(power);
        inputs.elbowVoltage = power * 12;
    }

    public void setShoulderAngle(double angle) {
        shoulderControlMode = ControlMode.Position;
        inputs.shoulderAngleAbsolute = angle;
    }

    public void setElbowAngle(double angle) {
        elbowControlMode = ControlMode.Position;
        inputs.elbowAngleAbsolute = angle;
    }

    public void setArmPosition(double x, double y) {
        var solution = armKinematics.inverseKinematics(new Translation2d(x, y));
        setShoulderAngle(solution.shoulderAngle);
        setElbowAngle(solution.elbowAngle);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Arm", inputs);

        if (elbowControlMode == ControlMode.Position) {
            io.setElbowAngle(inputs.elbowAngleAbsolute);
        } else if (elbowControlMode == ControlMode.PercentOutput) {
            io.setElbowPower(inputs.elbowVoltage);
        }

        if (shoulderControlMode == ControlMode.Position) {
            io.setShoulderAngle(inputs.shoulderAngle);
        } else if (shoulderControlMode == ControlMode.PercentOutput) {
            io.setShoulderPower(inputs.shoulderVoltage);
        }
    }
}
