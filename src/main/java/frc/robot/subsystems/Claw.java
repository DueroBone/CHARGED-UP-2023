package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.DeviceConstants;

public class Claw {
  static CANSparkMax Motor = new CANSparkMax(DeviceConstants.clawMotorId, MotorType.kBrushless);

  public static void openClawMotor(double speed) {
    Motor.set(speed);
  }

  public static void closeClawMotor(double speed) {
    Motor.set(-speed);
  }
}