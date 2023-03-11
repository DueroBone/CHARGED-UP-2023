package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.DeviceConstants;

public class Claw {
  static CANSparkMax clawMotor = new CANSparkMax(DeviceConstants.clawMotorId, MotorType.kBrushless);
  public static void setup() {
    clawMotor.setIdleMode(IdleMode.kBrake);
    clawMotor.setSmartCurrentLimit(10);
    clawMotor.burnFlash();
  }

  public static void open(double speed) {
    clawMotor.set(speed);
  }

  public static void close(double speed) {
    clawMotor.set(-speed);
  }

  public static void stop() {
    clawMotor.stopMotor();
  }
}