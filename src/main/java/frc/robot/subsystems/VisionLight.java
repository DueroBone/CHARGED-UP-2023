package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PWM.PeriodMultiplier;
import frc.robot.Constants;

public class VisionLight {


    static PWM pwm = new PWM(Constants.DeviceConstants.LightPWM);
    public static void toggle() {
    // Set the PWM pulse width in milliseconds
    pwm.setPeriodMultiplier(PeriodMultiplier.k1X);
    if (pwm.getSpeed() > 0.1) {
        pwm.setDisabled();
    } else {
        pwm.setSpeed(1);
    }
    System.out.println("**LET THERE BE LIGHT** | " + pwm.getSpeed());
    }
}