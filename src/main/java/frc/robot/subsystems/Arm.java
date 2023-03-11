package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DeviceConstants;
import frc.robot.commands.resetArm;

public class Arm {
  static String preset = "starting";
  static double actualHeight;
  static double actualLength;
  static double toleranceHeight = 5;
  static double toleranceLength = 5;
  static double armUpSpeed = DeviceConstants.armUpMax;
  static double armDownSpeed = DeviceConstants.armDownMax;
  static double armInSpeed = DeviceConstants.armInMax;
  static double armOutSpeed = DeviceConstants.armOutMax;
  static CANSparkMax lifterMotor = new CANSparkMax(DeviceConstants.armLifterId, MotorType.kBrushless);
  static CANSparkMax extenderMotor = new CANSparkMax(DeviceConstants.armExtenderId, MotorType.kBrushless);
  static RelativeEncoder lifterEncoder;
  static RelativeEncoder extenderEncoder;
  static SparkMaxLimitSwitch lifterLimitUp;
  static SparkMaxLimitSwitch lifterLimitDown;
  static SparkMaxLimitSwitch extenderLimitIn;
  static SparkMaxLimitSwitch extenderLimitOut;

  public static void setup() { // unfinished
    System.out.print("Setting up arm motors");
    lifterMotor.setIdleMode(IdleMode.kBrake);
    extenderMotor.setIdleMode(IdleMode.kBrake);

    lifterMotor.setInverted(true);

    lifterMotor.setSmartCurrentLimit(DeviceConstants.armAmpsMax);
    extenderMotor.setSmartCurrentLimit(DeviceConstants.armAmpsMax);

    lifterEncoder = lifterMotor.getEncoder();
    extenderEncoder = extenderMotor.getEncoder();

    lifterEncoder.setPositionConversionFactor(0); // degrees // need to set
    extenderEncoder.setPositionConversionFactor(0); // inches

    lifterLimitUp = lifterMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    lifterLimitDown = lifterMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    extenderLimitIn = extenderMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    extenderLimitOut = extenderMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);

    lifterMotor.burnFlash();
    extenderMotor.burnFlash();

    System.out.println(" ... Done");
  }

  public static void moveToPreset() {
    double desiredHeight;
    double desiredLength;
    getPositions();

    if (preset == "driving") {
      desiredHeight = positions.drivingHeight;
      desiredLength = positions.drivingLength;
    } else if (preset == "bottom") {
      desiredHeight = positions.bottomHeight;
      desiredLength = positions.bottomLength;
    } else if (preset == "scoring") {
      desiredHeight = positions.scoringHeight;
      desiredLength = positions.scoringLength;
    } else {
      desiredHeight = positions.startingHeight;
      desiredLength = positions.startingLength;
    }

    if (actualHeight < desiredHeight) {
      if (Math.abs(actualHeight - desiredHeight) > toleranceHeight) {
        moveLifter(false);
      } else {
        stopLifter();
      }
    } else if (actualHeight > desiredHeight) {
      if (Math.abs(actualHeight - desiredHeight) > toleranceHeight) {
        moveLifter(true);
      } else {
        stopLifter();
      }
    } else {
      stopLifter();
    }

    if (actualLength < desiredLength) {
      if (Math.abs(actualLength - desiredLength) > toleranceLength) {
        moveExtender(true);
      } else {
        stopExtender();
      }
    } else if (actualLength > desiredLength) {
      if (Math.abs(actualLength - desiredLength) > toleranceLength) {
        moveExtender(false);
      } else {
        stopExtender();
      }
    } else {
      stopExtender();
    }
  }

  public static void moveLifter(boolean up) {
    if (up) {
      setLifter(armUpSpeed);
    } else {
      setLifter(armDownSpeed);
    }
  }

  public static void moveExtender(boolean out) {
    if (out) {
      setExtender(armOutSpeed);
    } else {
      setExtender(armInSpeed);
    }
  }

  public static void setLifter(double speed) {
    lifterMotor.set(speed);
  }

  public static void setExtender(double speed) {
    extenderMotor.set(speed);
  }

  public static void stopLifter() {
    lifterMotor.set(0.05);
  }

  public static void stopExtender() {
    extenderMotor.stopMotor();
  }

  public static void stopArm() {
    lifterMotor.stopMotor();
    extenderMotor.stopMotor();
  }

  public static void startingPosition() {
    preset = "starting";
    System.out.println("Moving arm to preset: " + preset);
  }

  public static void drivingPosition() {
    preset = "driving";
    System.out.println("Moving arm to preset: " + preset);
  }

  public static void bottomPosition() {
    preset = "bottom";
    System.out.println("Moving arm to preset: " + preset);
  }

  public static void scoringPosition() {
    preset = "scoring";
    System.out.println("Moving arm to preset: " + preset);
  }

  public static void moveToStartingReset() {
    System.out.println("**ARM POSITION IS BEING RESET**");
    CommandScheduler.getInstance().schedule(new resetArm());
  }

  public static class info {
    public static double getLifterSpeed() {
      return lifterMotor.get();
    }

    public static double getExtenderSpeed() {
      return extenderMotor.get();
    }

    public static double getLifterVelocity() {
      return lifterEncoder.getVelocity();
    }

    public static double getExtenderVelocity() {
      return extenderEncoder.getVelocity();
    }

    public static boolean getLifterLimitUp() {
      return lifterLimitUp.isPressed();
    }

    public static boolean getLifterLimitDown() {
      return lifterLimitDown.isPressed();
    }

    public static boolean getExtenderLimitIn() {
      return extenderLimitIn.isPressed();
    }

    public static boolean getExtenderLimitOut() {
      return extenderLimitOut.isPressed();
    }

    public static void resetEncoders() {
      lifterEncoder.setPosition(0);
      extenderEncoder.setPosition(0);
    }
  }

  static void getPositions() {
    actualHeight = lifterEncoder.getPosition();
    actualLength = extenderEncoder.getPosition();
  }
}

class positions {
  public static double startingHeight = 0;
  public static double startingLength = 0;

  public static double drivingHeight = 0;
  public static double drivingLength = 0;

  public static double bottomHeight = 0;
  public static double bottomLength = 0;

  public static double scoringHeight = 0;
  public static double scoringLength = 0;
}