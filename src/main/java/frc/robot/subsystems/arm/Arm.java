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
    private ArmInputsAutoLogged inputs;

    private Arm(ArmIO io) {
        this.io = io;
        this.inputs = new ArmInputsAutoLogged();
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

    public void setEndEffectorPosition(Translation2d endEffectorPosition, ArmKinematics armKinematics) {
        inputs.endEffectorPosition = new double[]{endEffectorPosition.getX(), endEffectorPosition.getY()};
    }

    @Override
    public void periodic() {
        io.setElbowAngle(inputs.elbowAngleRelative);
        io.setElbowPower(inputs.elbowAppliedVoltage);
        io.setShoulderAngle(inputs.shoulderAngle);
        io.setShoulderPower(inputs.shoulderAppliedVoltage);
        io.setEndEffectorPosition(new Translation2d(inputs.endEffectorPosition[0], inputs.endEffectorPosition[1]), ArmIO.armKenematics);
        io.updateInputs();
        Logger.getInstance().recordOutput("BottomArmPose", new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(0))));

        Logger.getInstance().recordOutput("TopArmPose", new Pose3d(new Translation3d(inputs.shoulderTipPose[0], 0, inputs.shoulderTipPose[1]), new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(0)))); //TODO: check how to chnage the spin of the origin

        Logger.getInstance().recordOutput("IntakePose", new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(0))));

    }
}
