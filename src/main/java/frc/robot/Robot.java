package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.RunInTeleop;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  public static int CamWidth = 640;
  public static int CamHeight = 480;

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
    if (!Robot.isSimulation()) {
      UsbCamera visionCamera = CameraServer.startAutomaticCapture();
      visionCamera.setResolution(CamWidth, CamHeight);
      visionCamera.setBrightness(15);
    }
    DriverStation.silenceJoystickConnectionWarning(true);

    m_robotContainer = new RobotContainer();
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
  }

  // This function is called once each time the robot is Disabled
  @Override
  public void disabledInit() {
    // Hammer.hammer.set(DoubleSolenoid.Value.kReverse);
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

    CommandScheduler.getInstance().clearButtons();
    if (!CommandScheduler.getInstance().isScheduled(new RunInTeleop())) {
      System.out.println("Calling RunInTeleOp");
      CommandScheduler.getInstance().schedule(new RunInTeleop());
    }

    RobotContainer.configureButtonBindings();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
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
