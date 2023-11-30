package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmKinematics;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SetArmsPositionAngular extends Command {
    private final Arm arm = Arm.getINSTANCE();
    private final DoubleSupplier shoulderAngle;
    private final DoubleSupplier elbowAngle;
    private final double deadBand;
    private final Timer timer = new Timer();
    private final Supplier<Translation2d> positionSupplier;
    private TrapezoidProfile shoulderProfile;
    private TrapezoidProfile elbowProfile;

    private double finalShoulderVelocity = 0;
    private double finalElbowVelocity = 0;

    public SetArmsPositionAngular(Supplier<Translation2d> positionSupplier, double deadBand) {
        Supplier<ArmKinematics.InverseKinematicsSolution> solution =
                () -> arm.getKinematics().inverseKinematics(positionSupplier.get());
        this.shoulderAngle = () -> Math.toDegrees(solution.get().shoulderAngle);
        this.elbowAngle = () -> Math.toDegrees(solution.get().elbowAngle);
        this.deadBand = deadBand;
        this.positionSupplier = positionSupplier;
        addRequirements(arm);
    }

    public SetArmsPositionAngular(Supplier<Translation2d> positionSupplier) {
        this(positionSupplier, 0.05);
    }

    public SetArmsPositionAngular(Supplier<Translation2d> positionSupplier, double deadBand, double finalShoulderVelocity, double finalElbowVelocity) {
        this(positionSupplier, deadBand);
        this.finalShoulderVelocity = finalShoulderVelocity;
        this.finalElbowVelocity = finalElbowVelocity;
    }

    @Override
    public void initialize() {
        double currentShoulderAngle = arm.getShoulderAngle();
        double currentElbowAngle = arm.getElbowAngleRelative();
        shoulderProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(180, 540));
        elbowProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(360, 2080));

        timer.start();
        timer.reset();
    }

    @Override
    public void execute() {
        double currentShoulderAngle = arm.getShoulderAngle();
        double currentElbowAngle = arm.getElbowAngleRelative();
        double time = timer.get();
        double shoulderSetpoint = shoulderProfile.calculate(time,
                new TrapezoidProfile.State(shoulderAngle.getAsDouble(), finalShoulderVelocity),
                new TrapezoidProfile.State(currentShoulderAngle, 0)).position;
        double elbowSetpoint = elbowProfile.calculate(time,
                new TrapezoidProfile.State(elbowAngle.getAsDouble(), finalElbowVelocity),
                new TrapezoidProfile.State(currentElbowAngle, 0)).position;

        arm.setShoulderAngle(shoulderSetpoint);
        arm.setElbowAngleRelative(elbowSetpoint);

    }

    @Override
    public boolean isFinished() {
        return arm.getEndPosition().minus(positionSupplier.get()).getNorm() < deadBand
                || positionSupplier.get().getNorm() > (ArmConstants.SHOULDER_ARM_LENGTH + ArmConstants.SHOULDER_LENGTH);
    }
}
