// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {

  //// ----- Motor Controllers ----- /////
  // There are 4 separate motor controllers with 1 pwm channel per controller
  public static final CANSparkMax motorDriveLeft1 = new CANSparkMax(DriveConstants.leftDrive1Id, MotorType.kBrushless);
  public static final CANSparkMax motorDriveLeft2 = new CANSparkMax(DriveConstants.leftDrive2Id, MotorType.kBrushless);
  public static final CANSparkMax motorDriveLeft3 = new CANSparkMax(DriveConstants.leftDrive3Id, MotorType.kBrushless);
  public static final CANSparkMax motorDriveRight1 = new CANSparkMax(DriveConstants.rightDrive1Id, MotorType.kBrushless);
  public static final CANSparkMax motorDriveRight2 = new CANSparkMax(DriveConstants.rightDrive2Id, MotorType.kBrushless);
  public static final CANSparkMax motorDriveRight3 = new CANSparkMax(DriveConstants.rightDrive3Id, MotorType.kBrushless);

  // define Speed Controller Groups and Differential Drive for use in drive train
  private static final MotorControllerGroup driveGroupLeft = new MotorControllerGroup(motorDriveLeft1, motorDriveLeft2, motorDriveLeft3);
  private static final MotorControllerGroup driveGroupRight = new MotorControllerGroup(motorDriveRight1, motorDriveRight2, motorDriveRight3);
  private static final DifferentialDrive differentialDrive = new DifferentialDrive(driveGroupLeft, driveGroupRight);

  // Pneumatic solenoids for Hammer
  // private Solenoid solenoidHammerRaise; // The solenoids we use have two
  // channels, one for each output
  // private Solenoid solenoidHammerDown;
  private boolean highGear = false;

  // navX Gyro on RoboRIO 2.0
  public static AHRS m_Gyro;

  private static final boolean kSquareInputs = true;
  private static final boolean kSkipGyro = false;
  private static int counter = 0; // for limiting display

  static DoubleSolenoid gearChanger;

  /*
   * Creates a new DriveTrain.
   */
  public DriveTrain() {
    System.out.println("Instatntiating drivetrain");

    motorDriveLeft1.restoreFactoryDefaults(); // Clear any non default configuration/settings
    motorDriveLeft2.restoreFactoryDefaults();
    motorDriveLeft3.restoreFactoryDefaults();
    motorDriveRight1.restoreFactoryDefaults();
    motorDriveRight2.restoreFactoryDefaults();
    motorDriveRight3.restoreFactoryDefaults();

    // SupplyCurrentLimitConfiguration supplyLimit = new
    // SupplyCurrentLimitConfiguration(true, 30, 35, 1.0);
    int ampsMax = 20;
    motorDriveLeft1.setSmartCurrentLimit(ampsMax); // Set the current limist
    motorDriveLeft2.setSmartCurrentLimit(ampsMax);
    motorDriveLeft3.setSmartCurrentLimit(ampsMax);
    motorDriveRight1.setSmartCurrentLimit(ampsMax);
    motorDriveRight2.setSmartCurrentLimit(ampsMax);
    motorDriveRight3.setSmartCurrentLimit(ampsMax);

    motorDriveLeft1.setClosedLoopRampRate(5);
    motorDriveLeft2.setClosedLoopRampRate(5);
    motorDriveLeft3.setClosedLoopRampRate(5);
    motorDriveRight1.setClosedLoopRampRate(5);
    motorDriveRight2.setClosedLoopRampRate(5);
    motorDriveRight3.setClosedLoopRampRate(5);

    // DifferentialDrive inverts right side by default, so no need to setInvert()

    // Invert 1 side of robot so will drive forward    
    driveGroupRight.setInverted(true);

    motorDriveLeft1.setIdleMode(IdleMode.kCoast); // set brake mode
    motorDriveLeft2.setIdleMode(IdleMode.kCoast);
    motorDriveLeft3.setIdleMode(IdleMode.kCoast);
    motorDriveRight1.setIdleMode(IdleMode.kCoast);
    motorDriveRight2.setIdleMode(IdleMode.kCoast);
    motorDriveRight3.setIdleMode(IdleMode.kCoast);

    // driveStraightControl.setTolerance(0.02); // set tolerance around setpoint

    // Initialize the solenoids
    gearChanger = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.DriveConstants.GearChangeUp, Constants.DriveConstants.GearChangeDown);

    if (kSkipGyro) {
      m_Gyro = null;

    } else {
      // navX-MXP Gyro instantiation
      try {
        // Instantiate Gyro - communicate w/navX-MXP via the MXP SPI Bus
        // Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB
        // See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for
        // details
        m_Gyro = new AHRS(SPI.Port.kMXP);

      } catch (RuntimeException ex) {
        DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
      }
      while (m_Gyro.isCalibrating()) {
        try {
          Thread.sleep(500);
        } catch (Exception e) {
          System.out.println(e);
        } // sleep in milliseconds
        System.out.println("**gyro isCalibrating . . .");
      }
      SmartDashboard.putBoolean("gyro connected", m_Gyro.isConnected());
      System.out.println("gyro connected " + m_Gyro.isConnected());
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Tank style driving for the DriveTrain.
   *
   * @param left  Speed in range [-1,1]
   * @param right Speed in range [-1,1]
   */
  public static void doTankDrive(double leftDrivePercent, double rightDrivePercent) {

    if (counter++ % 100 == 0) {
      System.out.println("**driveTrain power L/R: " + leftDrivePercent + " | " + rightDrivePercent);
    }
    // SquareInputs adjust inputs at low speeds so better control - note 1.0 * 1.0
    // is still 1
    // differentialDrive.tankDrive(leftDrivePercent, rightDrivePercent,
    // kSquareInputs); // Send output to drive train
    // System.out.println("Left: " + leftDrivePercent + " Right: " +
    // rightDrivePercent);
    // motorDriveLeft1.set(leftDrivePercent);
    // motorDriveLeft2.set(leftDrivePercent);
    // motorDriveRight1.set(rightDrivePercent);
    // motorDriveRight2.set(rightDrivePercent);
    if (Math.abs(leftDrivePercent) > 0.01) {
      driveGroupLeft.set(leftDrivePercent);
    }
    if (Math.abs(rightDrivePercent) > 0.01) {
      driveGroupRight.set(rightDrivePercent);
    }
  }

  public void doTankDriveDefault(double leftDrivePercent, double rightDrivePercent) {

    // if (counter++ % 100 == 0) { System.out.println("**default driveTrain power: "
    // + leftDrivePercent+"-"+rightDrivePercent); }
    // SquareInputs adjust inputs at low speeds so better control - note 1.0 * 1.0
    // is still 1
    differentialDrive.tankDrive(leftDrivePercent, rightDrivePercent, kSquareInputs); // send output to drive train
  }

  /**
   * Arcade style driving for the DriveTrain.
   *
   * @param speed    Speed in range [-1,1]
   * @param rotation Rotation in range [-1,1]
   */
  public static void doArcadeDrive(double speed, double rotation) {
    // if (counter++ % 100 == 0) { System.out.println("**arcadeDrive power
    // speed/rotation: " + speed+"-"rotation); }
    // SquareInputs adjust inputs at low speeds so better control - note 1.0 * 1.0
    // is still 1
    differentialDrive.arcadeDrive(speed, rotation, kSquareInputs);
  }

  // http://pdocs.kauailabs.com/navx-mxp/guidance/terminology (for pitch, roll,
  // yaw, IMU terminology)
  public double getHeadingAngle() {
    return m_Gyro.getAngle(); // get current heading
    // return Math.IEEEremainder(m_Gyro.getAngle(), 360.0);
    // return 0.0;
  }

  public double getYaw() {
    return m_Gyro.getYaw(); // get rotation around Z axis for current heading
    // return 0.0;
  }

  public static void resetGyro() {
    m_Gyro.reset();
    // "Zero" yaw (whatever direction sensor is pointing now becomes new "Zero"
    // degrees
    m_Gyro.zeroYaw();
  }

  // public void resetEncoder() {
  public void resetEncoders() {
    // m_leftEncoder.reset();
    // m_rightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    // return m_leftEncoder.get();
    return 0;
  }

  public int getRightEncoderCount() {
    // return m_rightEncoder.get();
    return 0;
  }

  // get current distance since last encoder reset
  public double getLeftDistance() {
    // return m_leftEncoder.getDistance();
    return 0.0;
  }

  public double getLeftDistanceInch() {
    // return Math.PI * DriveConstants.WHEEL_DIAMETER * (getLeftEncoderCount() /
    // DriveConstants.PULSES_PER_REVOLUTION);
    return 0.0;
  }

  public double getRightDistanceInch() {
    // return Math.PI * DriveConstants.WHEEL_DIAMETER * (getRightEncoderCount() /
    // DriveConstants.PULSES_PER_REVOLUTION);
    return 0.0;
  }

  public double getAveDistanceInch() {
    // return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
    return 0.0;
  }

  // Function to set the solenoids
  public void doHighGear(boolean fast) {

    highGear = fast;
    if (fast) {
      gearChanger.set(DoubleSolenoid.Value.kForward);
      System.out.println("Gear shifter set to Low Torque Mode");
    } else {
      gearChanger.set(DoubleSolenoid.Value.kReverse);
      System.out.println("Gear shifter set to High Torque Mode");
    }
  }

  // public double clampValue(double value, double min, double max) {
  // return Math.max(min, Math.min(value, max)); // Make sure we are within range
  // }

  public static void stop() {
    System.out.println("in drivetrain stop");
    doTankDrive(0.0, 0.0);
  }

}
