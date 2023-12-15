package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.arm.commands.ArmXboxControl;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    private static Arm INSTANCE;
    private final ArmInputsLogged inputs = new ArmInputsLogged();
    private final ArmIO io;
    private final Mechanism2d mechanism = new Mechanism2d(
            3, 3
    );
    private final MechanismRoot2d root = mechanism.getRoot("Arm", 1, 1);
    private final MechanismLigament2d shoulder = root.append(
            new MechanismLigament2d("Shoulder", ArmConstants.SHOULDER_LENGTH, 0)
    );
    private final MechanismLigament2d elbow = shoulder.append(
            new MechanismLigament2d("Elbow", ArmConstants.ELBOW_LENGTH, 0)
    );
    private Command lastCommand = null;
    private Command currentCommand = null;
    private boolean changedToDefaultCommand = false;


    private Arm() {
        if (Robot.isReal()) {
            io = new ArmIOReal();
        } else {
            io = new ArmIOReal();
        }
    }

    public static Arm getINSTANCE() {
        if (INSTANCE == null) {
            INSTANCE = new Arm();
        }
        return INSTANCE;
    }

    public void setElbowPower(double power) {
        inputs.elbowAppliedVoltage = power * 12;
        inputs.elbowControlMode = ArmIO.ControlMode.PRECENT_OUTPUT;
    }

    public void setShoulderPower(double power) {
        inputs.shoulderAppliedVoltage = power * 12;
        inputs.shoulderControlMode = ArmIO.ControlMode.PRECENT_OUTPUT;
    }

    public void setEndEffectorPosition(Translation2d endEffectorPosition) {
        inputs.endEffectorPositionSetPoint = new double[]{endEffectorPosition.getX(), endEffectorPosition.getY()};
        setShoulderAngle((ArmIO.armKinematics.inverseKinematics(endEffectorPosition)).shoulderAngle);
        setElbowAngleRelative(((ArmIO.armKinematics.inverseKinematics(endEffectorPosition)).elbowAngle));
    }

    public void setElbowP(double kP) {
        io.setElbowP(kP);
    }

    public double getShoulderAngle() {
        return inputs.shoulderAngle;
    }

    public void setShoulderAngle(double angle) {
        inputs.shoulderAngleSetPoint = angle;
        inputs.shoulderControlMode = ArmIO.ControlMode.POSITION;
    }

    public double getElbowAngleRelative() {
        return inputs.elbowAngleRelative;
    }

    public void setElbowAngleRelative(double angle) {
        inputs.elbowAngleSetpoint = angle;
        inputs.elbowControlMode = ArmIO.ControlMode.POSITION;
    }

    public Translation2d getEndPosition() {
        double shoulderAngle = getShoulderAngle();
        double elbowAngle = getElbowAngleRelative();
        return ArmIO.armKinematics.forwardKinematics(shoulderAngle, shoulderAngle + elbowAngle - Math.PI);
    }

    public ArmInputsLogged getInputs() {
        return inputs;
    }

    public ArmKinematics getKinematics() {
        return ArmIO.armKinematics;
    }

    public boolean armIsOutOfFrame() {
        return !(inputs.shoulderTipPose[0] < 0) || !(inputs.endEffectorPose.getX() < 0);
    }

    public boolean armIsInRobot() {
        return inputs.shoulderTipPose[1] < 0 && inputs.shoulderAngle > 90;
    }

    public boolean changedToDefaultCommand() {
        return changedToDefaultCommand;
    }

    @Override
    public void periodic() {
        io.updateInputs();
        Logger.processInputs("Arm", inputs);
        currentCommand = getCurrentCommand();

        if (currentCommand != null) {
            Logger.recordOutput("ArmCommand", currentCommand.getName());
        }
        changedToDefaultCommand = !(lastCommand instanceof ArmXboxControl) && (currentCommand instanceof ArmXboxControl);
        lastCommand = currentCommand;

        shoulder.setAngle(Math.toDegrees(inputs.shoulderAngle));
        elbow.setAngle(Math.toDegrees(inputs.elbowAngleRelative) - 180);
        SmartDashboard.putData("Arm Mechanism", mechanism);

        if (inputs.elbowControlMode == ArmIO.ControlMode.POSITION) {
            io.setElbowAngle(inputs.elbowAngleSetpoint);

        } else if (inputs.elbowControlMode == ArmIO.ControlMode.PRECENT_OUTPUT) {
            io.setElbowPower(inputs.elbowAppliedVoltage);
        }
        if (inputs.shoulderControlMode == ArmIO.ControlMode.POSITION) {
            io.setShoulderAngle(inputs.shoulderAngleSetPoint);
        } else if (inputs.shoulderControlMode == ArmIO.ControlMode.PRECENT_OUTPUT) {
            io.setShoulderPower(inputs.shoulderAppliedVoltage);
        }
        Logger.recordOutput("BottomArmPose", new Pose3d(ArmConstants.shoulderOrigin, new Rotation3d(Math.toRadians(0), inputs.shoulderAngle, Math.toRadians(0))));
        Logger.recordOutput("TopArmPose", new Pose3d(ArmConstants.shoulderOrigin.plus(new Translation3d(-inputs.shoulderTipPose[0], 0, inputs.shoulderTipPose[1])), new Rotation3d(Math.toRadians(0), inputs.elbowAngleAbsolute - Math.PI, Math.toRadians(0)))); //TODO: check how to chnage the spin of the origin
    }

    public void stop() {
        inputs.shoulderControlMode = null;
        inputs.elbowControlMode = null;
    }

}
