// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package frc.robot;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.Threads;

import java.util.function.BiConsumer;
import java.util.stream.Stream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import static frc.robot.Constants.*;

// ---------------------------------------------------------------[Robot Class]-------------------------------------------------------------//
public final class Robot extends LoggedRobot  {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//

  // ---------------------------------------------------------------[Robot]-----------------------------------------------------------------//
  @Override
  public void robotInit() {
    LOGGER.recordMetadata(("ProjectName"), ("PROJECT-LIQUORICE"));
    if (isReal()) {
      LOGGER.addDataReceiver(new WPILOGWriter(("/media/sda1/")));
      LOGGER.addDataReceiver(new NT4Publisher());
      LoggedPowerDistribution.getInstance((0), ModuleType.kAutomatic);
    } else {
      if(Constants.AdvantageKit.REPLAY_FROM_LOG) {
        setUseTiming(TURBO_MODE);
        String logPath = LogFileUtil.findReplayLog();
        LOGGER.setReplaySource(new WPILOGReader(logPath));
        LOGGER.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
      } else {
        LOGGER.addDataReceiver(new WPILOGWriter("src\\main\\java\\frc\\deploy\\logs"));
        LOGGER.addDataReceiver(new NT4Publisher());        
      }
    }
    HashMap<String,Integer> CommandInstanceCount = new HashMap<>();
    BiConsumer<Command, Boolean> CommandFunctionLogger = (Command Operation, Boolean Active) -> {
      String OperationName = Operation.getName();
      int Count = CommandInstanceCount.getOrDefault(OperationName, (0)) + ((Active)? (1): (-1));
      CommandInstanceCount.put(OperationName,Count);
      LOGGER.recordOutput("UniqueOperations/" + OperationName + "_" + Integer.toHexString(Operation.hashCode()), Active);
      LOGGER.recordOutput("Operations/" + OperationName, Count > 0);
    };
    CommandScheduler.getInstance().onCommandInitialize((Command Command) -> CommandFunctionLogger.accept(Command, (true)));
    CommandScheduler.getInstance().onCommandInterrupt((Command Command) -> CommandFunctionLogger.accept(Command, (false)));
    CommandScheduler.getInstance().onCommandFinish((Command Command) -> CommandFunctionLogger.accept(Command, (false)));    
    Logger.getInstance().start();
    for (int ForwardingPort = (5800); ForwardingPort <= (5805); ForwardingPort++) {
      PortForwarder.add(ForwardingPort, ("limelight.local"), ForwardingPort);
    }    
    RobotContainer.getInstance();
  }

  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority((true), (99));
    CommandScheduler.getInstance().run();
    List<String> ClientNames, ClientAddresses;
    ClientNames = new ArrayList<>(); ClientAddresses = new ArrayList<>();
    Stream.of(NetworkTableInstance.getDefault().getConnections()).forEach((Connection) -> {
      ClientNames.add(Connection.remote_id);
      ClientAddresses.add(Connection.remote_ip);
    });
    LOGGER.recordOutput(("NTClient/Names"), ClientNames.toArray(new String[0]));
    LOGGER.recordOutput(("NTClient/Addresses"), ClientAddresses.toArray(new String[0]));
    Threads.setCurrentThreadPriority((true), (20));
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
    Shuffleboard.startRecording();
  }


  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void autonomousExit() {
    Shuffleboard.stopRecording();
  }

  // -----------------------------------------------------------[Teleoperated]--------------------------------------------------------------//
  @Override
  public void teleopInit() {
    Shuffleboard.startRecording();
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopExit() {
    Shuffleboard.stopRecording();
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