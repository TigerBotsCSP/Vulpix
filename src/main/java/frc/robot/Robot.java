package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Swerve.Drive;

public class Robot extends TimedRobot {
  public Drive m_drive = new Drive();

  @Override
  public void autonomousPeriodic() {
    // ? When running paths, don't forget this
    m_drive.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    m_drive.swerve(RobotContainer.m_controller.getLeftY(), RobotContainer.m_controller.getLeftX(),
        RobotContainer.m_controller.getRightX());
  }
}
