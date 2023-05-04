package frc.robot.utils.math.differential;

public class Integral {

    private double value;
    private double lastValue;

    private double sum;

    private double lastTimestamp = 0;

    private Integral integral = null;

    public Integral(double initialValue, double initialLastValue) {
        this.value = initialValue;
        this.lastValue = initialLastValue;
    }

    public Integral integrate() {
        if (integral == null) {
            integral = new Integral(sum, sum);
        }
        return integral;
    }

    public void update(double newValue) {
        double timestamp = System.currentTimeMillis();

        lastValue = value;
        value = newValue;

        sum += (value + lastValue) * (timestamp - lastTimestamp) / 2;

        if (integral != null) {
            integral.update(sum);
        }

        lastTimestamp = timestamp;
    }

    public double get() {
        return sum;
    }
}
