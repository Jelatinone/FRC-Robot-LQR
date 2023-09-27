// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package frc.robot.subsystems.drivebase;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.lib.DrivebaseModule;

import java.util.function.Supplier;

// ------------------------------------------------------------[Constants Class]------------------------------------------------------------//
public final class Constants {
    
  public static final class Values {

    public static final class ComponentData {
      public static final Double ENCODER_SENSITIVITY = (2048.0);    
      public static final Double SCALE_FACTOR = (1.0/50.0);   
      public static final Double AZIMUTH_DEADBAND = (0.06);
      public static final StatorCurrentLimitConfiguration DRIVE_CURRENT_LIMIT = new StatorCurrentLimitConfiguration((true), (60), (60), (0));      
      public static final StatorCurrentLimitConfiguration AZIMUTH_CURRENT_LIMIT = new StatorCurrentLimitConfiguration((true), (20), (20), (0));
    }

    public static final class PathPlanner {
      public static final Double TRANSLATION_KP = (0.2); // {15, 3.25, 2.75, 2.5, 2.1, 2, 0.018, 0.03, 0.004, 0.001}
      public static final Double TRANSLATION_KI = (0.0); 
      public static final Double TRANSLATION_KD = (0.0);
      public static final Double ROTATION_KP = (0.008); // {6.25, 12.5, 15}
      public static final Double ROTATION_KI = (0.0);
      public static final Double ROTATION_KD = (0.0);
    }

    public static final class Chassis {
      public static final Double WHEEL_DIAMETER = (0.1016);          
      public static final Double WHEEL_PERIMETER = (WHEEL_DIAMETER) * Math.PI;      
      public static final Double DRIVETRAIN_GEAR_RATIO = (6.75);    
      public static final Double ROBOT_WIDTH = (0.6858);         
    }

    public static final class Limit {   
      public static final Double ROBOT_MAXIMUM_VELOCITY = (5.4);    
    }

    public static final class Port {
      public static final Integer PATHPLANNER_SERVER_PORT = (6969);     
      public static final Integer GYROSCOPE_ID = (3);   

    }
  }

  public static final class Hardware {
    public static final WPI_Pigeon2 GYROSCOPE = new WPI_Pigeon2(Values.Port.GYROSCOPE_ID);

    public static final class Modules {
        public static final class FL_Module {

          public static final class Values {
            public static final Double MAXIMUM_AZIMUTH_ACCELERATION = (Math.PI*2);    
            public static final Double AZIMUTH_ENCODER_POSITION_OFFSET = (-313.006);
            public static final Double MAXIMUM_AZIMUTH_VOLTAGE = (12.0);            
            public static final Double MAXIMUM_AZIMUTH_VELOCITY = (5.4);
            public static final Double MAXIMUM_LINEAR_VELOCITY = (5.4);
            public static final Double ACCELERATION_GAIN = (0.035); //TODO <---- Find Acceleration Gain (kA) of Linear Position System, Volts/(Units/Sec^2), DO NOT DEPLOY THIS UNTIL FOUND
            public static final Double VELOCITY_GAIN = (0.035); //TODO <---- Find Velocity Gain (kV) of Linear Position System, Volts/(Units/Sec), DO NOT DEPLOY THIS UNTIL FOUND

            public static final Double LINEAR_ENCODER_SENSITIVITY = (2048.0);     
            public static final Double ENCODER_SENSITIVITY = (4096.0);   

            public static final Double MODEL_VELOCITY_ACCURACY_DEGREES = (7e-5);
            public static final Double MODEL_POSITION_ACCURACY_RPM = (2.5e-4);
            public static final Double DISCRETIZATION_TIMESTEP = (1/50.0);

            public static final Integer AZIMUTH_ID = (21);                       
            public static final Integer LINEAR_ID = (11);    
            public static final Integer SENSOR_ID = (4);
          }

          public static final class Components {
            public static final WPI_CANCoder AZIMUTH_SENSOR = DrivebaseModule.configureRotationEncoder((new WPI_CANCoder(Values.SENSOR_ID)), Values.AZIMUTH_ENCODER_POSITION_OFFSET);
            public static final WPI_TalonFX LINEAR_CONTROLLER = DrivebaseModule.configureTranslationController(new WPI_TalonFX(Values.LINEAR_ID), Constants.Values.ComponentData.DRIVE_CURRENT_LIMIT);            
            public static final WPI_TalonFX AZIMUTH_CONTROLLER = DrivebaseModule.configureRotationController(new WPI_TalonFX(Values.AZIMUTH_ID), AZIMUTH_SENSOR, Constants.Values.ComponentData.AZIMUTH_CURRENT_LIMIT, Constants.Values.ComponentData.AZIMUTH_DEADBAND);                                 
            public static final Supplier<SwerveModuleState> STATE_SENSOR = () -> new SwerveModuleState(
              (2.0)*(((LINEAR_CONTROLLER.getSelectedSensorVelocity() / Values.LINEAR_ENCODER_SENSITIVITY) * (10)) / Constants.Values.Chassis.DRIVETRAIN_GEAR_RATIO) * Math.PI * Constants.Values.Chassis.WHEEL_DIAMETER,
              new Rotation2d(AZIMUTH_SENSOR.getAbsolutePosition()));            
            public static final DrivebaseModule MODULE = new DrivebaseModule(new MotorControllerGroup(AZIMUTH_CONTROLLER), new MotorControllerGroup(AZIMUTH_CONTROLLER), STATE_SENSOR, () -> Values.MAXIMUM_LINEAR_VELOCITY, () -> Control.CONSTRAINTS, Control.CONTROL_LOOP);
          }
      

          public static final class Control {
            public static final Double AZIMUTH_EPSILON_POSITION = Components.AZIMUTH_SENSOR.getAbsolutePosition();
            public static final Double AZIMUTH_EPSILON_VELOCITY = Components.AZIMUTH_SENSOR.getVelocity();
            public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(Values.MAXIMUM_AZIMUTH_VELOCITY, Values.MAXIMUM_AZIMUTH_ACCELERATION);
            public static final LinearSystem<N2,N1,N1> PLANT = LinearSystemId.identifyPositionSystem(Values.VELOCITY_GAIN, Values.ACCELERATION_GAIN);        
            public static final LinearSystemLoop<N2,N1,N1> CONTROL_LOOP = new LinearSystemLoop<>(
              PLANT,
              new LinearQuadraticRegulator<>(
                PLANT,
                VecBuilder.fill(Units.degreesToRadians((AZIMUTH_EPSILON_POSITION > 0)? (AZIMUTH_EPSILON_POSITION): (1.0)), Units.rotationsPerMinuteToRadiansPerSecond((AZIMUTH_EPSILON_VELOCITY > 0)? (AZIMUTH_EPSILON_VELOCITY): (1.0))), //<---- Controller error weight, how much we want to conserve controller error, or how accurate we want to be, increased time to target
                VecBuilder.fill(Values.MAXIMUM_AZIMUTH_VOLTAGE), //<---- Controller effort weight, how much we want to conserve controller expenditure or how fast we want to be; increased state error
                Values.DISCRETIZATION_TIMESTEP),
              new KalmanFilter<>(
                  Nat.N2(),
                  Nat.N1(),
                  PLANT,
                  VecBuilder.fill(Units.degreesToRadians(Values.MODEL_VELOCITY_ACCURACY_DEGREES), Units.rotationsPerMinuteToRadiansPerSecond(Values.MODEL_POSITION_ACCURACY_RPM)), //<---- How accurate we think our models are
                  VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(1 / Values.ENCODER_SENSITIVITY)), //<---- How accurate we think our encoders are 
                  Values.DISCRETIZATION_TIMESTEP),
              Values.MAXIMUM_AZIMUTH_VOLTAGE,
              Values.DISCRETIZATION_TIMESTEP);
          }
        }
        public static final class FR_Module {

          public static final class Values {
            public static final Double MAXIMUM_AZIMUTH_ACCELERATION = (Math.PI*2);    
            public static final Double AZIMUTH_ENCODER_POSITION_OFFSET = (-68.582);
            public static final Double MAXIMUM_AZIMUTH_VOLTAGE = (12.0);            
            public static final Double MAXIMUM_AZIMUTH_VELOCITY = (5.4);
            public static final Double MAXIMUM_LINEAR_VELOCITY = (5.4);
            public static final Double ACCELERATION_GAIN = (0.035);
            public static final Double VELOCITY_GAIN = (0.035);

            public static final Double LINEAR_ENCODER_SENSITIVITY = (2048.0);     
            public static final Double ENCODER_SENSITIVITY = (4096.0);   

            public static final Double MODEL_VELOCITY_ACCURACY_DEGREES = (7e-5);
            public static final Double MODEL_POSITION_ACCURACY_RPM = (2.5e-4);
            public static final Double DISCRETIZATION_TIMESTEP = (1/50.0);

            public static final Integer AZIMUTH_ID = (22);                       
            public static final Integer LINEAR_ID = (12);    
            public static final Integer SENSOR_ID = (5);
          }

          public static final class Components {
            public static final WPI_CANCoder AZIMUTH_SENSOR = DrivebaseModule.configureRotationEncoder((new WPI_CANCoder(Values.SENSOR_ID)), Values.AZIMUTH_ENCODER_POSITION_OFFSET);
            public static final WPI_TalonFX LINEAR_CONTROLLER = DrivebaseModule.configureTranslationController(new WPI_TalonFX(Values.LINEAR_ID), Constants.Values.ComponentData.DRIVE_CURRENT_LIMIT);            
            public static final WPI_TalonFX AZIMUTH_CONTROLLER = DrivebaseModule.configureRotationController(new WPI_TalonFX(Values.AZIMUTH_ID), AZIMUTH_SENSOR, Constants.Values.ComponentData.AZIMUTH_CURRENT_LIMIT, Constants.Values.ComponentData.AZIMUTH_DEADBAND);                              
            public static final Supplier<SwerveModuleState> STATE_SENSOR = () -> new SwerveModuleState(
              (2.0)*(((LINEAR_CONTROLLER.getSelectedSensorVelocity() / Values.LINEAR_ENCODER_SENSITIVITY) * (10)) / Constants.Values.Chassis.DRIVETRAIN_GEAR_RATIO) * Math.PI * Constants.Values.Chassis.WHEEL_DIAMETER,
              new Rotation2d(AZIMUTH_SENSOR.getAbsolutePosition()));            
            public static final DrivebaseModule MODULE = new DrivebaseModule(new MotorControllerGroup(AZIMUTH_CONTROLLER), new MotorControllerGroup(AZIMUTH_CONTROLLER), STATE_SENSOR, () -> Values.MAXIMUM_LINEAR_VELOCITY, () -> Control.CONSTRAINTS, Control.CONTROL_LOOP);
          }
      

          public static final class Control {
            public static final Double AZIMUTH_EPSILON_POSITION = Components.AZIMUTH_SENSOR.getAbsolutePosition();
            public static final Double AZIMUTH_EPSILON_VELOCITY = Components.AZIMUTH_SENSOR.getVelocity();
            public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(Values.MAXIMUM_AZIMUTH_VELOCITY, Values.MAXIMUM_AZIMUTH_ACCELERATION);
            public static final LinearSystem<N2,N1,N1> PLANT = LinearSystemId.identifyPositionSystem(Values.VELOCITY_GAIN, Values.ACCELERATION_GAIN);        
            public static final LinearSystemLoop<N2,N1,N1> CONTROL_LOOP = new LinearSystemLoop<>(
              PLANT,
              new LinearQuadraticRegulator<>(
                PLANT,
                VecBuilder.fill(Units.degreesToRadians((AZIMUTH_EPSILON_POSITION > 0)? (AZIMUTH_EPSILON_POSITION): (1.0)), Units.rotationsPerMinuteToRadiansPerSecond((AZIMUTH_EPSILON_VELOCITY > 0)? (AZIMUTH_EPSILON_VELOCITY): (1.0))),
                VecBuilder.fill(Values.MAXIMUM_AZIMUTH_VOLTAGE), 
                Values.DISCRETIZATION_TIMESTEP),
              new KalmanFilter<>(
                  Nat.N2(),
                  Nat.N1(),
                  PLANT,
                  VecBuilder.fill(Units.degreesToRadians(Values.MODEL_VELOCITY_ACCURACY_DEGREES), Units.rotationsPerMinuteToRadiansPerSecond(Values.MODEL_POSITION_ACCURACY_RPM)),
                  VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(1 / Values.ENCODER_SENSITIVITY)), 
                  Values.DISCRETIZATION_TIMESTEP),
              Values.MAXIMUM_AZIMUTH_VOLTAGE,
              Values.DISCRETIZATION_TIMESTEP);
          }
        }
        public static final class RL_Module {

          public static final class Values {
            public static final Double MAXIMUM_AZIMUTH_ACCELERATION = (Math.PI*2);    
            public static final Double AZIMUTH_ENCODER_POSITION_OFFSET = (134.209);
            public static final Double MAXIMUM_AZIMUTH_VOLTAGE = (12.0);            
            public static final Double MAXIMUM_AZIMUTH_VELOCITY = (5.4);
            public static final Double MAXIMUM_LINEAR_VELOCITY = (5.4);
            public static final Double ACCELERATION_GAIN = (0.035);
            public static final Double VELOCITY_GAIN = (0.035);

            public static final Double LINEAR_ENCODER_SENSITIVITY = (2048.0);     
            public static final Double ENCODER_SENSITIVITY = (4096.0);   

            public static final Double MODEL_VELOCITY_ACCURACY_DEGREES = (7e-5);
            public static final Double MODEL_POSITION_ACCURACY_RPM = (2.5e-4);
            public static final Double DISCRETIZATION_TIMESTEP = (1/50.0);

            public static final Integer AZIMUTH_ID = (23);                       
            public static final Integer LINEAR_ID = (13);    
            public static final Integer SENSOR_ID = (6);
          }

          public static final class Components {
            public static final WPI_CANCoder AZIMUTH_SENSOR = DrivebaseModule.configureRotationEncoder((new WPI_CANCoder(Values.SENSOR_ID)), Values.AZIMUTH_ENCODER_POSITION_OFFSET);
            public static final WPI_TalonFX LINEAR_CONTROLLER = DrivebaseModule.configureTranslationController(new WPI_TalonFX(Values.LINEAR_ID), Constants.Values.ComponentData.DRIVE_CURRENT_LIMIT);            
            public static final WPI_TalonFX AZIMUTH_CONTROLLER = DrivebaseModule.configureRotationController(new WPI_TalonFX(Values.AZIMUTH_ID), AZIMUTH_SENSOR, Constants.Values.ComponentData.AZIMUTH_CURRENT_LIMIT, Constants.Values.ComponentData.AZIMUTH_DEADBAND);                             
            public static final Supplier<SwerveModuleState> STATE_SENSOR = () -> new SwerveModuleState(
              (2.0)*(((LINEAR_CONTROLLER.getSelectedSensorVelocity() / Values.LINEAR_ENCODER_SENSITIVITY) * (10)) / Constants.Values.Chassis.DRIVETRAIN_GEAR_RATIO) * Math.PI * Constants.Values.Chassis.WHEEL_DIAMETER,
              new Rotation2d(AZIMUTH_SENSOR.getAbsolutePosition()));            
            public static final DrivebaseModule MODULE = new DrivebaseModule(new MotorControllerGroup(AZIMUTH_CONTROLLER), new MotorControllerGroup(AZIMUTH_CONTROLLER), STATE_SENSOR, () -> Values.MAXIMUM_LINEAR_VELOCITY, () -> Control.CONSTRAINTS, Control.CONTROL_LOOP);
          }
      

          public static final class Control {
            public static final Double AZIMUTH_EPSILON_POSITION = Components.AZIMUTH_SENSOR.getAbsolutePosition();
            public static final Double AZIMUTH_EPSILON_VELOCITY = Components.AZIMUTH_SENSOR.getVelocity();
            public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(Values.MAXIMUM_AZIMUTH_VELOCITY, Values.MAXIMUM_AZIMUTH_ACCELERATION);
            public static final LinearSystem<N2,N1,N1> PLANT = LinearSystemId.identifyPositionSystem(Values.VELOCITY_GAIN, Values.ACCELERATION_GAIN);        
            public static final LinearSystemLoop<N2,N1,N1> CONTROL_LOOP = new LinearSystemLoop<>(
              PLANT,
              new LinearQuadraticRegulator<>(
                PLANT,
                VecBuilder.fill(Units.degreesToRadians((AZIMUTH_EPSILON_POSITION > 0)? (AZIMUTH_EPSILON_POSITION): (1.0)), Units.rotationsPerMinuteToRadiansPerSecond((AZIMUTH_EPSILON_VELOCITY > 0)? (AZIMUTH_EPSILON_VELOCITY): (1.0))),
                VecBuilder.fill(Values.MAXIMUM_AZIMUTH_VOLTAGE), 
                Values.DISCRETIZATION_TIMESTEP),
              new KalmanFilter<>(
                  Nat.N2(),
                  Nat.N1(),
                  PLANT,
                  VecBuilder.fill(Units.degreesToRadians(Values.MODEL_VELOCITY_ACCURACY_DEGREES), Units.rotationsPerMinuteToRadiansPerSecond(Values.MODEL_POSITION_ACCURACY_RPM)),
                  VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(1 / Values.ENCODER_SENSITIVITY)), 
                  Values.DISCRETIZATION_TIMESTEP),
              Values.MAXIMUM_AZIMUTH_VOLTAGE,
              Values.DISCRETIZATION_TIMESTEP);
          }
        }
        public static final class RR_Module {

          public static final class Values {
            public static final Double MAXIMUM_AZIMUTH_ACCELERATION = (Math.PI*2);    
            public static final Double AZIMUTH_ENCODER_POSITION_OFFSET = (-257.783);
            public static final Double MAXIMUM_AZIMUTH_VOLTAGE = (12.0);            
            public static final Double MAXIMUM_AZIMUTH_VELOCITY = (5.4);
            public static final Double MAXIMUM_LINEAR_VELOCITY = (5.4);
            public static final Double ACCELERATION_GAIN = (0.035);
            public static final Double VELOCITY_GAIN = (0.035);

            public static final Double LINEAR_ENCODER_SENSITIVITY = (2048.0);     
            public static final Double ENCODER_SENSITIVITY = (4096.0);   

            public static final Double MODEL_VELOCITY_ACCURACY_DEGREES = (7e-5);
            public static final Double MODEL_POSITION_ACCURACY_RPM = (2.5e-4);
            public static final Double DISCRETIZATION_TIMESTEP = (1/50.0);

            public static final Integer AZIMUTH_ID = (24);                       
            public static final Integer LINEAR_ID = (14);    
            public static final Integer SENSOR_ID = (7);
          }

          public static final class Components {
            public static final WPI_CANCoder AZIMUTH_SENSOR = DrivebaseModule.configureRotationEncoder((new WPI_CANCoder(Values.SENSOR_ID)), Values.AZIMUTH_ENCODER_POSITION_OFFSET);
            public static final WPI_TalonFX LINEAR_CONTROLLER = DrivebaseModule.configureTranslationController(new WPI_TalonFX(Values.LINEAR_ID), Constants.Values.ComponentData.DRIVE_CURRENT_LIMIT);            
            public static final WPI_TalonFX AZIMUTH_CONTROLLER = DrivebaseModule.configureRotationController(new WPI_TalonFX(Values.AZIMUTH_ID), AZIMUTH_SENSOR, Constants.Values.ComponentData.AZIMUTH_CURRENT_LIMIT, Constants.Values.ComponentData.AZIMUTH_DEADBAND);                         
            public static final Supplier<SwerveModuleState> STATE_SENSOR = () -> new SwerveModuleState(
              (2.0)*(((LINEAR_CONTROLLER.getSelectedSensorVelocity() / Values.LINEAR_ENCODER_SENSITIVITY) * (10)) / Constants.Values.Chassis.DRIVETRAIN_GEAR_RATIO) * Math.PI * Constants.Values.Chassis.WHEEL_DIAMETER,
              new Rotation2d(AZIMUTH_SENSOR.getAbsolutePosition()));            
            public static final DrivebaseModule MODULE = new DrivebaseModule(new MotorControllerGroup(AZIMUTH_CONTROLLER), new MotorControllerGroup(AZIMUTH_CONTROLLER), STATE_SENSOR, () -> Values.MAXIMUM_LINEAR_VELOCITY, () -> Control.CONSTRAINTS, Control.CONTROL_LOOP);
          }
      

          public static final class Control {
            public static final Double AZIMUTH_EPSILON_POSITION = Components.AZIMUTH_SENSOR.getAbsolutePosition();
            public static final Double AZIMUTH_EPSILON_VELOCITY = Components.AZIMUTH_SENSOR.getVelocity();
            public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(Values.MAXIMUM_AZIMUTH_VELOCITY, Values.MAXIMUM_AZIMUTH_ACCELERATION);
            public static final LinearSystem<N2,N1,N1> PLANT = LinearSystemId.identifyPositionSystem(Values.VELOCITY_GAIN, Values.ACCELERATION_GAIN);        
            public static final LinearSystemLoop<N2,N1,N1> CONTROL_LOOP = new LinearSystemLoop<>(
              PLANT,
              new LinearQuadraticRegulator<>(
                PLANT,
                VecBuilder.fill(Units.degreesToRadians((AZIMUTH_EPSILON_POSITION > 0)? (AZIMUTH_EPSILON_POSITION): (1.0)), Units.rotationsPerMinuteToRadiansPerSecond((AZIMUTH_EPSILON_VELOCITY > 0)? (AZIMUTH_EPSILON_VELOCITY): (1.0))),
                VecBuilder.fill(Values.MAXIMUM_AZIMUTH_VOLTAGE), 
                Values.DISCRETIZATION_TIMESTEP),
              new KalmanFilter<>(
                  Nat.N2(),
                  Nat.N1(),
                  PLANT,
                  VecBuilder.fill(Units.degreesToRadians(Values.MODEL_VELOCITY_ACCURACY_DEGREES), Units.rotationsPerMinuteToRadiansPerSecond(Values.MODEL_POSITION_ACCURACY_RPM)),
                  VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(1 / Values.ENCODER_SENSITIVITY)), 
                  Values.DISCRETIZATION_TIMESTEP),
              Values.MAXIMUM_AZIMUTH_VOLTAGE,
              Values.DISCRETIZATION_TIMESTEP);
          }
        }    
    }    
  }
}
