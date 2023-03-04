package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunInDisable extends CommandBase {
  //Variables init

  public RunInDisable() {} //Declare major variables

  @Override
  public void initialize() {} //Declare rest of variables

  @Override
  public void execute() {
    
  }

  @Override
  public void end(boolean interrupted) {} //Reset everything

  @Override
  public boolean isFinished() {
    return !RobotState.isDisabled(); 
  }
}
