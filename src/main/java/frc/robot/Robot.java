// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package frc.robot;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.TimedRobot;
// ---------------------------------------------------------------[Robot Class]-------------------------------------------------------------//
public final class Robot extends TimedRobot {
  // ---------------------------------------------------------------[Robot]-----------------------------------------------------------------//
  @Override
  public void robotInit() {
    //TODO: Multi-Limelight Enabled Port Forwarding
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  // ------------------------------------------------------------[Simulation]---------------------------------------------------------------//
  @Override
  public void simulationInit() {

  }

  @Override
  public void simulationPeriodic() {

  }

  // -------------------------------------------------------------[Disabled]----------------------------------------------------------------//
  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {

  }


  @Override
  public void disabledExit() {

  }  

  // ------------------------------------------------------------[Autonomous]---------------------------------------------------------------//  
  @Override
  public void autonomousInit() {

  }


  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void autonomousExit() {

  }

  // --------------------------------------------------------------[Teleop]-----------------------------------------------------------------//
  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopExit() {

  }   

  // ---------------------------------------------------------------[Test]------------------------------------------------------------------//
  @Override
  public void testPeriodic() {

  } 

  @Override
  public void testInit() {

  }

  @Override
  public void testExit() {

  }
}