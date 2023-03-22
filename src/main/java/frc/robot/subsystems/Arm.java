package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.Constants.DeviceConstants;
import frc.robot.commands.resetArm;

public class Arm {
  static String preset = "starting";
  static double actualHeight;
  static double actualLength;
  static double toleranceHeight = 2;
  static double toleranceLength = 1;
  static double holdLifter;
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
  static PIDController pid;

  public static void setup() { // unfinished
    System.out.print("Setting up arm motors");
    lifterMotor.setIdleMode(IdleMode.kBrake);
    extenderMotor.setIdleMode(IdleMode.kBrake);

    lifterMotor.setInverted(true);

    lifterMotor.setSmartCurrentLimit(DeviceConstants.armAmpsMax);
    extenderMotor.setSmartCurrentLimit(DeviceConstants.armAmpsMax);

    lifterEncoder = lifterMotor.getEncoder();
    extenderEncoder = extenderMotor.getEncoder();

    lifterEncoder.setPositionConversionFactor(0.3925); // degrees // 0.6262 157.93 == 62
    extenderEncoder.setPositionConversionFactor(0.4323); // inches | not correct

    lifterLimitUp = lifterMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    lifterLimitDown = lifterMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    extenderLimitIn = extenderMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    extenderLimitOut = extenderMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    pid = new PIDController(0.05, 0, 0.5);
    pid.setTolerance(toleranceHeight);
    pid.enableContinuousInput(-70, 70);

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
    if (actualHeight == desiredHeight) {
      holdLifter = info.getLifterPosition();
    }

    if (actualHeight < desiredHeight) {
      if (Math.abs(actualHeight - desiredHeight) > toleranceHeight) {
        moveLifter(true);
      } else {
        holdLifter();
      }
    } else if (actualHeight > desiredHeight) {
      if (Math.abs(actualHeight - desiredHeight) > toleranceHeight) {
        moveLifter(false);
      } else {
        holdLifter();
      }
    } else {
      holdLifter();
    }

    if (actualLength < desiredLength) {
      if (Math.abs(actualLength - desiredLength) > toleranceLength) {
        moveExtender(true, (actualLength - desiredLength));
      } else {
        stopExtender();
      }
    } else if (actualLength > desiredLength) {
      if (Math.abs(actualLength - desiredLength) > toleranceLength) {
        moveExtender(false, (actualLength - desiredLength));
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
    if (RobotContainer.safteyEnabled) {
      if (lifterEncoder.getPosition() < -80) {
        DriverStation.reportWarning("ARM TOO LOW TO EXTEND!!", false);
      } else {
        if (out) {
          setExtender(armOutSpeed);
        } else {
          setExtender(armInSpeed);
        }
      }
    }
  }

  public static void moveExtender(boolean out, double distance) {
    if (RobotContainer.safteyEnabled) {
      if (lifterEncoder.getPosition() < -80) {
        DriverStation.reportWarning("ARM TOO LOW TO EXTEND!!", false);
      }
      if (lifterEncoder.getPosition() > -20) {
        DriverStation.reportWarning("ARM TOO HIGH TO EXTEND!!", false);
      } else {
        if (out) {
          if (Math.abs(distance) < 5) {
            setExtender(armOutSpeed / 3);
          } else {
            setExtender(armOutSpeed);
          }
        } else {
          if (Math.abs(distance) < 5) {
            setExtender(armInSpeed / 3);
          } else {
            setExtender(armInSpeed);
          }
        }
      }
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
    holdLifter = info.getLifterPosition();
    holdLifter();
    System.out.println("Stop L");
    // lifterMotor.set(0.1);
  }

  public static void holdLifter() {
    // System.out.println(info.getLifterPosition() + " " + holdLifter);
    // setLifter(pid.calculate(info.getLifterPosition(), holdLifter));
    setLifter(0.02);
  }

  public static void stopExtender() {
    // extenderMotor.set(0.02);
    extenderMotor.stopMotor();
  }

  public static void stopArm() {
    stopLifter();
    stopExtender();
    System.out.println("Stop arm");
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
  public static final double drivingLength = 50;

  public static final double bottomHeight = -100;
  public static final double bottomLength = 10;

  public static final double scoringHeight = -55;
  public static final double scoringLength = 225;
}