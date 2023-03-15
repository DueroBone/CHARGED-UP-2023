package frc.robot;

import java.util.function.BooleanSupplier;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.RobotContainer.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.I2C;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  public static int CamWidth = 640;
  public static int CamHeight = 480;
  private ColorSensorV3 colorSensor;
  int counter1;
  boolean isBraked = true;

  /*
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    try {
      UsbCamera visionCamera = CameraServer.startAutomaticCapture();
      visionCamera.setResolution(CamWidth, CamHeight);
      visionCamera.setBrightness(15);
    } catch (VideoException e) {
      System.out.println("NO CAMERA DETECTED");
    }
    DriverStation.silenceJoystickConnectionWarning(true);

    // Instantiate the REV Color Sensor V2 object
    colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    Arm.setup();
    Claw.setup();
    RobotContainer.RemapControllers();
    RobotContainer.configureButtonBindings();
    //Arm.moveToStartingReset();
  }

  /*
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    /*
     * Runs the Scheduler. This is responsible for polling buttons, adding
     * newly-scheduled
     * commands, running already-scheduled commands, removing finished or
     * interrupted commands,
     * and running subsystem periodic() methods. This must be called from the
     * robot's periodic
     * block in order for anything in the Command-based framework to work.
     */
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("Left Drive Speed", DriveTrain.getLeftSpeed());
    SmartDashboard.putNumber("Right Drive Speed", DriveTrain.getRightSpeed());
    SmartDashboard.putNumber("Arm Lifter Speed", Arm.info.getLifterSpeed());
    SmartDashboard.putNumber("Arm Extender Speed", Arm.info.getExtenderSpeed());

  }

  // This function is called once each time the robot is Disabled
  @Override
  public void disabledInit() {
    System.out.println("Disabled");
  }

  @Override
  public void disabledPeriodic() {

  }

  // This autonomous runs the autonomous command selected by your {@link
  // RobotContainer} class
  @Override
  public void autonomousInit() {
    System.out.println("Autonomous enabled");
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // Schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    System.out.println("Teleop Enabled");
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    counter1 = 0;
    CommandScheduler.getInstance().clearButtons();
    RobotContainer.RemapControllers();
    RobotContainer.configureButtonBindings();

    CommandScheduler.getInstance().clearButtons();
    RobotContainer.RemapControllers();
    RobotContainer.configureButtonBindings();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("IR", colorSensor.getIR());

    BooleanSupplier LT = () -> (dynamicXbox.object.getLeftTriggerAxis() > 0.5);
    BooleanSupplier RT = () -> (dynamicXbox.object.getRightTriggerAxis() > 0.5);
    dynamicXbox.LeftTrigger = new Button(LT);
    dynamicXbox.RightTrigger = new Button(RT);
    counter1++;
    if (counter1 % 3 == 0) {
      double trigger = dynamicXbox.object.getRightTriggerAxis();
      if ((trigger < 0.5) != isBraked) {
        System.out.println("Switched DriveTrain brake mode to " + isBraked);
        if (trigger < 0.5) {
          DriveTrain.motorDriveLeft1.setIdleMode(IdleMode.kCoast);
          DriveTrain.motorDriveLeft2.setIdleMode(IdleMode.kCoast);
          DriveTrain.motorDriveLeft3.setIdleMode(IdleMode.kCoast);
          DriveTrain.motorDriveRight1.setIdleMode(IdleMode.kCoast);
          DriveTrain.motorDriveRight2.setIdleMode(IdleMode.kCoast);
          DriveTrain.motorDriveRight3.setIdleMode(IdleMode.kCoast);
        } else {
          DriveTrain.motorDriveLeft1.setIdleMode(IdleMode.kBrake);
          DriveTrain.motorDriveLeft2.setIdleMode(IdleMode.kBrake);
          DriveTrain.motorDriveLeft3.setIdleMode(IdleMode.kBrake);
          DriveTrain.motorDriveRight1.setIdleMode(IdleMode.kBrake);
          DriveTrain.motorDriveRight2.setIdleMode(IdleMode.kBrake);
          DriveTrain.motorDriveRight3.setIdleMode(IdleMode.kBrake);
        }
      }

      if (counter1 % 100 == 0) {
        System.out.println("Lifter: " + Arm.info.getLifterSpeed() + " Extender: " + Arm.info.getExtenderSpeed());
      }

      if (trigger < 0.5) {
        isBraked = true;
      } else {
        isBraked = false;
      }
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    teleopInit();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    teleopPeriodic();
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
