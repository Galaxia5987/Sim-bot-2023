package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.leds.Leds;

public class ArmWithStateMachine extends CommandBase {

    private final Arm arm = Arm.getInstance();

    private final frc.robot.subsystems.arm.ArmPosition desiredPosition;
    private final Timer timer = new Timer();
    private frc.robot.subsystems.arm.ArmPosition nextPosition;
    private frc.robot.subsystems.arm.ArmPosition lastPosition;
    private TrapezoidProfile shoulderProfile;
    private TrapezoidProfile elbowProfile;

    public ArmWithStateMachine(frc.robot.subsystems.arm.ArmPosition desiredPosition) {
        this.desiredPosition = desiredPosition;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setDesiredPosition(desiredPosition);

        nextPosition = desiredPosition.nextState(arm.getInputs());
        lastPosition = nextPosition;
        var pose = Leds.getInstance().inConeMode() ? nextPosition.conePose : nextPosition.cubePose;
        var angles = arm.getKinematics().inverseKinematics(pose);

        shoulderProfile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(180, 540),
                new TrapezoidProfile.State(Math.toDegrees(angles.shoulderAngle), 0),
                new TrapezoidProfile.State(arm.getShoulderJointAngle().getDegrees(), arm.getShoulderMotorVelocity())
        );
        elbowProfile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(360, 2080),
                new TrapezoidProfile.State(Math.toDegrees(angles.elbowAngle), 0),
                new TrapezoidProfile.State(arm.getElbowJointAngle().getDegrees(), arm.getElbowMotorVelocity())
        );

        timer.start();
        timer.reset();
    }

    /**
     *
     */
    @Override
    public void execute() {
        nextPosition = desiredPosition.nextState(arm.getInputs());

        if (nextPosition != lastPosition) {
            updateTrapezoidProfiles();
        }

        arm.setShoulderJointAngle(shoulderProfile.calculate(timer.get()).position);
        arm.setElbowJointAngle(elbowProfile.calculate(timer.get()).position);

        lastPosition = nextPosition;
    }

    private void updateTrapezoidProfiles() {
        var pose = Leds.getInstance().inConeMode() ? desiredPosition.conePose : desiredPosition.cubePose;
        var angles = arm.getKinematics().inverseKinematics(pose);

        shoulderProfile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(180, 540),
                new TrapezoidProfile.State(Math.toDegrees(angles.shoulderAngle), 0),
                new TrapezoidProfile.State(arm.getShoulderJointAngle().getDegrees(), arm.getShoulderMotorVelocity())
        );
        elbowProfile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(360, 2080),
                new TrapezoidProfile.State(Math.toDegrees(angles.elbowAngle), 0),
                new TrapezoidProfile.State(arm.getElbowJointAngle().getDegrees(), arm.getElbowMotorVelocity())
        );

        timer.reset();
    }
}
