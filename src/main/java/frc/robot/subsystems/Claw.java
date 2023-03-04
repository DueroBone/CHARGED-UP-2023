package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants.DeviceConstants;

public class Claw {
  static CANSparkMax Motor = new CANSparkMax(DeviceConstants.clawMotor, MotorType.kBrushless);
  static DoubleSolenoid Piston = new DoubleSolenoid(PneumaticsModuleType.REVPH, DeviceConstants.clawPnumaticsIn, DeviceConstants.clawPnumaticsOut);

  public static void openClawPiston() {
    Piston.set(Value.kForward);
  }

  public static void closeClawPiston() {
    Piston.set(Value.kReverse);
  }

  public static void openClawMotor() {
    Motor.set(0.5);
  }

  public static void closeClawMotor() {
    Motor.set(-0.5);
  }
}