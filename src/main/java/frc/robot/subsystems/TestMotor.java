package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class TestMotor {
  static CANSparkMax motor = new CANSparkMax(12, MotorType.kBrushless);
  public static void SetSpeed(double speed) {
    motor.set(speed);
  }
}
