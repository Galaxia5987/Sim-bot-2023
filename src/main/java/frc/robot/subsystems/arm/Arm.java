package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    static Arm INSTANCE;
    private final ArmIO io;
    private static final ArmInputsAutoLogged inputs = new ArmInputsAutoLogged();

    private Arm(ArmIO io) {
        this.io = io;
    }

    public static Arm getINSTANCE() {
        if (INSTANCE == null) {
            if (Robot.isReal()) {
                INSTANCE = new Arm(new ArmIOSim(inputs));

            } else {
                INSTANCE = new Arm(new ArmIOSim(inputs));
            }
        }
        return INSTANCE;
    }

    public void setElbowPower(double power) {
        inputs.elbowAppliedVoltage = power * 12;
        inputs.elbowControlMode = ArmIO.ControlMode.PRECENT_OUTPUT;
    }

    public void setElbowAngleRelative(double angle) {
        inputs.elbowAngleSetpoint = angle;
        inputs.elbowControlMode = ArmIO.ControlMode.POSITION;
    }

    public void setShoulderPower(double power) {
        inputs.shoulderAppliedVoltage = power * 12;
        inputs.shoulderControlMode = ArmIO.ControlMode.PRECENT_OUTPUT;
    }

    public void setShoulderAngle(double angle) {
        inputs.shoulderAngleSetPoint = angle;
        inputs.shoulderControlMode = ArmIO.ControlMode.POSITION;
    }

    public void setEndEffectorPosition(Translation2d endEffectorPosition, ArmKinematics armKinematics) {
        inputs.endEffectorPosition = new double[]{endEffectorPosition.getX(), endEffectorPosition.getY()};
    }

    @Override
    public void periodic() {
        io.updateInputs();
        Logger.getInstance().processInputs("Arm", inputs);
        if(inputs.elbowControlMode == ArmIO.ControlMode.POSITION){
            io.setElbowAngle(inputs.elbowAngleSetpoint);
        }
        else if(inputs.elbowControlMode == ArmIO.ControlMode.PRECENT_OUTPUT){
            io.setElbowPower(inputs.elbowAppliedVoltage);
        }
        if (inputs.shoulderControlMode == ArmIO.ControlMode.POSITION){
            io.setShoulderAngle(inputs.shoulderAngleSetPoint);
        }
        else if(inputs.shoulderControlMode == ArmIO.ControlMode.PRECENT_OUTPUT){
            io.setShoulderPower(inputs.shoulderAppliedVoltage);
        }
        Logger.getInstance().recordOutput("BottomArmPose", new Pose3d(new Translation3d(-0.29, 0, 0.37), new Rotation3d(Math.toRadians(0), inputs.shoulderAngle, Math.toRadians(0))));

        Logger.getInstance().recordOutput("TopArmPose", new Pose3d(new Translation3d(-0.29, 0, 0.37).plus(new Translation3d(-inputs.shoulderTipPose[0], 0,inputs.shoulderTipPose[1])), new Rotation3d(Math.toRadians(0), inputs.elbowAngleAbsolute, Math.toRadians(0)))); //TODO: check how to chnage the spin of the origin


    }
}
