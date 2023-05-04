package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

import java.util.ArrayList;

public class TunableNumber {

    public static final ArrayList<TunableNumber> INSTANCES = new ArrayList<>();

    private final String key;
    private double value;

    public TunableNumber(String key, double value) {
        this.key = key;
        this.value = value;

        if (Robot.isReal()) {
            SmartDashboard.putNumber(key, value);
        }

        INSTANCES.add(this);
    }

    public double get() {
        return value;
    }

    public void update() {
        if (Robot.isReal()) {
            value = SmartDashboard.getNumber(key, value);
        }
    }
}
