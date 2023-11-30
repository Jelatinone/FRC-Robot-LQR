// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package frc.robot;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//S
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.net.PortForwarder;
import java.util.function.BiConsumer;
import edu.wpi.first.wpilibj.Threads;
import java.util.stream.Stream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import static frc.robot.Constants.*;
import static frc.robot.Constants.AdvantageKit.*;
// ---------------------------------------------------------------[Robot Class]-------------------------------------------------------------//
public final class Robot extends LoggedRobot  {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private static final RepeatCommand COMMAND_LOGGER = new RepeatCommand(new InstantCommand(() -> {
    if(LOGGING_ENABLED) {
      Threads.setCurrentThreadPriority((true), (99));
      List<String> ClientNames, ClientAddresses;
      ClientNames = new ArrayList<>(); ClientAddresses = new ArrayList<>();
      Stream.of(NetworkTableInstance.getDefault().getConnections()).forEach((Connection) -> {
        ClientNames.add(Connection.remote_id);
        ClientAddresses.add(Connection.remote_ip);
      });
      LOGGER.recordOutput(("NTClient/Names"), ClientNames.toArray(String[]::new));
      LOGGER.recordOutput(("NTClient/Addresses"), ClientAddresses.toArray(String[]::new));
      Threads.setCurrentThreadPriority((true), (20));      
    }
  }));
  // ---------------------------------------------------------------[Robot]-----------------------------------------------------------------//
  @SuppressWarnings("ExtractMethodRecommender")
  @Override
  public void robotInit() {
    LOGGER.recordMetadata(("ProjectName"), ("PROJECT-LIQUORICE"));
    if (IS_REAL_ROBOT) {
      if(LOGGING_ENABLED) {
        LOGGER.addDataReceiver(new WPILOGWriter(("/media/sda1/")));
      }
      LOGGER.addDataReceiver(new NT4Publisher());
      LoggedPowerDistribution.getInstance((POWER_DISTRIBUTION_ID), ModuleType.kRev);
    } else {
      if(REPLAY_FROM_LOG) {
        setUseTiming(TURBO_MODE);
        String logPath = LogFileUtil.findReplayLog();
        LOGGER.setReplaySource(new WPILOGReader(logPath));
        if(LOGGING_ENABLED) {
          LOGGER.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, ("_sim"))));
        }
      } else {
        if(LOGGING_ENABLED) {
          LOGGER.addDataReceiver(new WPILOGWriter(LOGGING_FOLDER));
        }
        LOGGER.addDataReceiver(new NT4Publisher());        
      }
    }
    HashMap<String,Integer> CommandInstanceCount = new HashMap<>();
    BiConsumer<Command, Boolean> CommandFunctionLogger = (Command Operation, Boolean Active) -> new Thread(() -> {
    String OperationName = Operation.getName();
    int Count = CommandInstanceCount.getOrDefault(OperationName, (0)) + ((Active)? (1): (-1));
    CommandInstanceCount.put(OperationName,Count);
    LOGGER.recordOutput("UniqueOperations/" + OperationName + "_" + Integer.toHexString(Operation.hashCode()), Active);
    LOGGER.recordOutput("Operations/" + OperationName, Count > (0));
    });
    CommandScheduler.getInstance().onCommandInitialize((Command) -> CommandFunctionLogger.accept(Command, (true)));
    CommandScheduler.getInstance().onCommandInterrupt((Command) -> CommandFunctionLogger.accept(Command, (false)));
    CommandScheduler.getInstance().onCommandFinish((Command) -> CommandFunctionLogger.accept(Command, (false)));    
    LOGGER.start();
    for (int ForwardingPort = (5800); ForwardingPort <= (5805); ForwardingPort++) {
      PortForwarder.add(ForwardingPort, ("limelight.local"), ForwardingPort);
    }
    RobotContainer.getInstance();
    COMMAND_LOGGER.schedule();    
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();    
    SmartDashboard.updateValues();    
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
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledPeriodic() {

  }


  @Override
  public void disabledExit() {
    super.disabledExit();
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

  // ----------------------------------------------------------------[Test]------------------------------------------------------------------//
  @Override
  public void testPeriodic() {

  } 

  @Override
  public void testInit() {
    super.testInit();
  }

  @Override
  public void testExit() {
    super.testExit();
  }
}