package frc.robot.utils;

import frc.robot.utils.math.differential.Integral;
import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

@RunWith(JUnit4.class)
public class DifferentialTest {

    @Test
    public void testIntegral() {
        Integral integral = new Integral(0, 0);

        for (int i = 0; i < 10000; i++) {
            integral.update(i);
        }

        Assert.assertTrue(
                "Integral is nonzero",
                integral.get() > 0
        );
    }
}
