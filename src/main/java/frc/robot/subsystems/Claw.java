package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.DeviceConstants;

public class Claw {
  static CANSparkMax clawMotor = new CANSparkMax(DeviceConstants.clawMotorId, MotorType.kBrushless);
  public static void setup() {
    clawMotor.setIdleMode(IdleMode.kBrake);
    clawMotor.setSmartCurrentLimit(DeviceConstants.clawAmpsMax);
    clawMotor.burnFlash();
  }

  public static void open() {
    clawMotor.set(0.15);
  }

  public static void close() {
    clawMotor.set(-0.25);
  }

  public static void stop() {
    clawMotor.stopMotor();
  }
}