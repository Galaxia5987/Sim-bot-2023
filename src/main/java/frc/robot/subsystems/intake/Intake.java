package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.intake.commands.HoldIntakeInPlace;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private static Intake INSTANCE;
    private ControlMode angleMode;
    private final IntakeLoggedInputs inputs = new IntakeLoggedInputs();
    private Command lastCommand = null;
    private boolean switchedToDefaultCommand = false;
    private final Mechanism2d mech = new Mechanism2d(3, 3);
    private final MechanismRoot2d root = mech.getRoot("Intake", 1, 1);
    private final MechanismLigament2d intakeP1 = root.append(new MechanismLigament2d("IntakeP1", 0.3, 0));
    private final MechanismLigament2d intakeP2 = intakeP1.append(new MechanismLigament2d("IntakeP2", 0.3, -45));

    private Intake(){
        if (Robot.isReal()){
            io = new IntakeIOReal();
        }
        else{
            io = new IntakeIOSim();
        }
    }

    /**
     * @return the INSTANCE of the Intake.
     */
    public static Intake getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Intake();
        }
        return INSTANCE;
    }

    /**
     * @return the relative output.
     * Return the power that the motor applies. [%]
     */
    private double getSpinMotorPower() {
        return inputs.spinMotorPower;
    }
    public double getAngleMotorAngle() {
        return inputs.angleMotorAngle;
    }

    private double getAngleMotorVelocity() {
        return inputs.angleMotorVelocity;
    }
    public double getCurrent() {
        return inputs.angleMotorcurrent;
    }

    /**
     * Set the motors' relative output.
     * @param power is the power that the motor applies. [%]
     */
    public void setSpinMotorPower(double power) {
        inputs.setpointSpinMotorPower = power;
    }

    /**
     * Sets the angles position.
     *
     * @param angle is the angle of the retractor. [degrees]
     */
    public void setAngleMotorAngle(double angle) {
        inputs.setpointAngleMotorAngle = Math.toRadians(angle);
        angleMode = ControlMode.Position;
    }

    public void setAnglePower(double power) {
        inputs.setpointAngleMotorPower = power;
        angleMode = ControlMode.PercentOutput;
    }
    public void resetEncoder(double angle) {
        io.resetEncoder(angle);
    }

    public Command lowerIntake() {
        return new InstantCommand(() -> resetEncoder(IntakeConstants.ANGLE_UP), this)
                .andThen(new RunCommand(() -> setAngleMotorAngle(-40), this));
    }
    public Command run(double power) {
        return new RunCommand(() -> this.setSpinMotorPower(power));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        io.setSpinMotorPower(inputs.setpointSpinMotorPower);

        if (angleMode == ControlMode.Position){
            io.setAngleMotorAngle(inputs.setpointAngleMotorAngle);
        }
        else {;
            io.setAngleMotorPower(inputs.setpointAngleMotorPower);
        }


        var currentCommand = getCurrentCommand();
        switchedToDefaultCommand = (currentCommand instanceof HoldIntakeInPlace) &&
                !(lastCommand instanceof HoldIntakeInPlace);
        lastCommand = currentCommand;

        intakeP1.setAngle(Math.toDegrees(getAngleMotorAngle()) + 90);

        Logger.getInstance().processInputs("Intake", inputs);
        SmartDashboard.putData("IntakeMech", mech);
    }

    public boolean switchedToDefaultCommand() {
        return switchedToDefaultCommand;
    }
}
