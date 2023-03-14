package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DeviceConstants;
import frc.robot.commands.resetArm;

public class Arm {
  static String preset = "starting";
  static double actualHeight;
  static double actualLength;
  static double toleranceHeight = 2;
  static double toleranceLength = 1;
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

    lifterEncoder.setPositionConversionFactor(0.6262); // degrees
    extenderEncoder.setPositionConversionFactor(0.4323); // inches | not correct

    lifterLimitUp = lifterMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    lifterLimitDown = lifterMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    extenderLimitIn = extenderMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    extenderLimitOut = extenderMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    lifterLimitUp.enableLimitSwitch(false);
    lifterLimitDown.enableLimitSwitch(false);
    extenderLimitIn.enableLimitSwitch(false);
    extenderLimitOut.enableLimitSwitch(false);
    // lifterMotor.burnFlash();
    // extenderMotor.burnFlash();

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
        moveLifter(true);
      } else {
        stopLifter();
      }
    } else if (actualHeight > desiredHeight) {
      if (Math.abs(actualHeight - desiredHeight) > toleranceHeight) {
        moveLifter(false);
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
    // System.out.println("Arm at height: " + actualHeight + " Going to: " +
    // desiredHeight + " At speed: " + lifterMotor.get());
  }

  public static void moveLifter(boolean up) {
    if (up) {
      setLifter(armUpSpeed);
    } else {
      setLifter(armDownSpeed);
    }
  }

  public static void moveExtender(boolean out) {
    if (lifterEncoder.getPosition() > -40) {
      if (out) {
        setExtender(armOutSpeed);
      } else {
        setExtender(armInSpeed);
      }
    } else {
      DriverStation.reportWarning("ARM TOO LOW TO EXTEND!!", false);
    }
  }

  public static void setLifter(double speed) {
    lifterMotor.set(speed);
    // System.out.println("Lifter at: " + lifterMotor.get());
  }

  public static void setExtender(double speed) {
    extenderMotor.set(speed);
  }

  public static void stopLifter() {
    if (info.getLifterVelocity() < -0.005) {
      if (info.getLifterVelocity() < 0) {
        lifterMotor.set(-10 * info.getLifterVelocity());
      } else {
        lifterMotor.stopMotor();
      }
    }
    // lifterMotor.set(0.1);
    // System.out.println("STOPPING LIFTER");
  }

  public static void stopExtender() {
    // extenderMotor.set(0.02);
    extenderMotor.stopMotor();
  }

  public static void stopArm() {
    stopLifter();
    stopExtender();
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
    public static boolean isLifterStopped;

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

    public static double getLifterPosition() {
      return lifterEncoder.getPosition();
    }

    public static double getExtenderPosition() {
      return extenderEncoder.getPosition();
    }

    public static void resetEncoders() {
      lifterEncoder.setPosition(0);
      extenderEncoder.setPosition(0);
      DriverStation.reportWarning("Encoders reset", false);
    }
  }

  static void getPositions() {
    actualHeight = lifterEncoder.getPosition();
    actualLength = extenderEncoder.getPosition();
  }
}

class positions {
  public static final double startingHeight = 0;
  public static final double startingLength = 0;

  public static final double drivingHeight = -15;
  public static final double drivingLength = 0;

  public static final double bottomHeight = -50;
  public static final double bottomLength = 0;

  public static final double scoringHeight = -25;
  public static final double scoringLength = 225;
}