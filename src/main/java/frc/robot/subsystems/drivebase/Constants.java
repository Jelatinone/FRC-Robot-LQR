// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package frc.robot.subsystems.drivebase;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.LinearSystem;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.numbers.*;
import java.util.function.Supplier;
import edu.wpi.first.math.Nat;
import frc.lib.SwerveModule;

// ------------------------------------------------------------[Constants Class]------------------------------------------------------------//
public final class Constants {
    

  public static final class Values {
    public static final Integer PATHPLANNER_SERVER_PORT = (6969); 
    public static final Double DRIVETRAIN_GEAR_RATIO = (6.75);    
    public static final Double WHEEL_DIAMETER = (0.1016);    
    public static final Double WHEEL_PERIMETER = (WHEEL_DIAMETER) * Math.PI;
    public static final Double BUILTIN_ENCODER_SENSITIVITY = (2048.0);
    public static final Double ROBOT_WIDTH = (0.6858);    
    public static final Double SCALE_FACTOR = (1.0/50.0);

    public static final Integer GYROSCOPE_ID = (3);

  }

  public static final class Hardware {
    public static final WPI_Pigeon2 GYROSCOPE = new WPI_Pigeon2(Values.GYROSCOPE_ID);


    public static final class Modules {
        public static final class FL_Module {

          public static final class Values {
            public static final Double MAXIMUM_AZIMUTH_ACCELERATION = (Math.PI*2);    
            public static final Double MAXIMUM_AZIMUTH_VOLTAGE = (12.0);            
            public static final Double MAXIMM_AZIMUTH_VELOCITY = (5.4);              
            public static final Double MAXIMUM_LINEAR_VELOCITY = (5.4);
            public static final Double ACCELERATION_GAIN = (1.0); //<---- Acceleration Gain (kA) of Linear Position System, Volts/(Units/Sec^2)
            public static final Double VELOCITY_GAIN = (1.0); //<---- Velocity Gain (kV) of Linear Position System, Volts/(Units/Sec)

            public static final Double ENCODER_POSITION_ACCURACY_DEGREES = (1/4096.0); 
            public static final Double MODEL_VELOCITY_ACCURACY_DEGREES = (7e-5);
            public static final Double MODEL_POSITION_ACCURACY_RPM = (2.5e-4);
            public static final Double DISCRETIZATION_TIMESTEP = (1/50.0);

            public static final Integer AZIMUTH_ID = (21);                       
            public static final Integer LINEAR_ID = (11);    
            public static final Integer SENSOR_ID = (4);
          }

          public static final class Components {
            public static final WPI_CANCoder AZIMUTH_SENSOR = new WPI_CANCoder(Values.SENSOR_ID);
            public static final WPI_TalonFX AZIMUTH_CONTROLLER = new WPI_TalonFX(Values.AZIMUTH_ID);                                 
            public static final WPI_TalonFX LINEAR_CONTROLLER = new WPI_TalonFX(Values.LINEAR_ID);
            public static final Supplier<SwerveModuleState> STATE_SENSOR = () -> new SwerveModuleState(
              2.0*(((LINEAR_CONTROLLER.getSelectedSensorVelocity() / 2048) * 10) / Constants.Values.DRIVETRAIN_GEAR_RATIO) * Math.PI * Constants.Values.WHEEL_DIAMETER,
              new Rotation2d(AZIMUTH_SENSOR.getAbsolutePosition()));            
            public static final SwerveModule MODULE = new SwerveModule(new MotorControllerGroup(AZIMUTH_CONTROLLER), new MotorControllerGroup(LINEAR_CONTROLLER),STATE_SENSOR,() -> Control.CONSTRAINTS,Control.CONTROL_LOOP, () -> Values.MAXIMUM_LINEAR_VELOCITY);
          }
      

          public static final class Control {
            public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(Values.MAXIMM_AZIMUTH_VELOCITY, Values.MAXIMUM_AZIMUTH_ACCELERATION);
            public static final LinearSystem<N2,N1,N1> PLANT = LinearSystemId.identifyPositionSystem(Values.VELOCITY_GAIN, Values.ACCELERATION_GAIN);        
            public static final LinearSystemLoop<N2,N1,N1> CONTROL_LOOP = new LinearSystemLoop<N2,N1,N1>(
              PLANT,
              new LinearQuadraticRegulator<>(
                PLANT,
                VecBuilder.fill(Units.degreesToRadians(Components.AZIMUTH_SENSOR.getAbsolutePosition()),Units.rotationsPerMinuteToRadiansPerSecond(Components.AZIMUTH_SENSOR.getVelocity())), //<---- Controller error weight, how much we want to conserve controller error, or how accurate we want to be
                VecBuilder.fill(Values.MAXIMUM_AZIMUTH_VOLTAGE), //<---- Controller effort weight, how much we want to conserve controller expenditure or how fast we want to be; increased state error
                Values.DISCRETIZATION_TIMESTEP),
              new KalmanFilter<>(
                Nat.N2(),
                Nat.N1(),
                PLANT, 
                VecBuilder.fill(Units.degreesToRadians(Values.MODEL_VELOCITY_ACCURACY_DEGREES),Units.rotationsPerMinuteToRadiansPerSecond(Values.MODEL_POSITION_ACCURACY_RPM)), //<---- How accurate we think our models are
                VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(Values.ENCODER_POSITION_ACCURACY_DEGREES)), //<---- How accurate we think our encoders are
                Values.DISCRETIZATION_TIMESTEP),
              Values.MAXIMUM_AZIMUTH_VOLTAGE,
              Values.DISCRETIZATION_TIMESTEP);        
          }

        }
        public static final class FR_Module {

          public static final class Values {
            public static final Double MAXIMUM_AZIMUTH_ACCELERATION = (Math.PI*2);    
            public static final Double MAXIMUM_AZIMUTH_VOLTAGE = (12.0);            
            public static final Double MAXIMM_AZIMUTH_VELOCITY = (5.4);              
            public static final Double MAXIMUM_LINEAR_VELOCITY = (5.4);
            public static final Double ACCELERATION_GAIN = (1.0); //<---- Acceleration Gain (kA) of Linear Position System, Volts/(Units/Sec^2)
            public static final Double VELOCITY_GAIN = (1.0); //<---- Velocity Gain (kV) of Linear Position System, Volts/(Units/Sec)

            public static final Double ENCODER_POSITION_ACCURACY_DEGREES = (1/4096.0); 
            public static final Double MODEL_VELOCITY_ACCURACY_DEGREES = (7e-5);
            public static final Double MODEL_POSITION_ACCURACY_RPM = (2.5e-4);
            public static final Double DISCRETIZATION_TIMESTEP = (1/50.0);

            public static final Integer AZIMUTH_ID = (22);                       
            public static final Integer LINEAR_ID = (12);    
            public static final Integer SENSOR_ID = (5);
          }

          public static final class Components {
            public static final WPI_CANCoder AZIMUTH_SENSOR = new WPI_CANCoder(Values.SENSOR_ID);
            public static final WPI_TalonFX AZIMUTH_CONTROLLER = new WPI_TalonFX(Values.AZIMUTH_ID);                                 
            public static final WPI_TalonFX LINEAR_CONTROLLER = new WPI_TalonFX(Values.LINEAR_ID);
            public static final Supplier<SwerveModuleState> STATE_SENSOR = () -> new SwerveModuleState(
              2.0*(((LINEAR_CONTROLLER.getSelectedSensorVelocity() / 2048) * 10) / Constants.Values.DRIVETRAIN_GEAR_RATIO) * Math.PI * Constants.Values.WHEEL_DIAMETER,
              new Rotation2d(AZIMUTH_SENSOR.getAbsolutePosition()));            
            public static final SwerveModule MODULE = new SwerveModule(new MotorControllerGroup(AZIMUTH_CONTROLLER), new MotorControllerGroup(LINEAR_CONTROLLER),STATE_SENSOR,() -> Control.CONSTRAINTS,Control.CONTROL_LOOP, () -> Values.MAXIMUM_LINEAR_VELOCITY);
          }
      

          public static final class Control {
            public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(Values.MAXIMM_AZIMUTH_VELOCITY, Values.MAXIMUM_AZIMUTH_ACCELERATION);
            public static final LinearSystem<N2,N1,N1> PLANT = LinearSystemId.identifyPositionSystem(Values.VELOCITY_GAIN, Values.ACCELERATION_GAIN);        
            public static final LinearSystemLoop<N2,N1,N1> CONTROL_LOOP = new LinearSystemLoop<N2,N1,N1>(
              PLANT,
              new LinearQuadraticRegulator<>(
                PLANT,
                VecBuilder.fill(Units.degreesToRadians(Components.AZIMUTH_SENSOR.getAbsolutePosition()),Units.rotationsPerMinuteToRadiansPerSecond(Components.AZIMUTH_SENSOR.getVelocity())), //<---- Controller error weight, how much we want to conserve controller error, or how accurate we want to be
                VecBuilder.fill(Values.MAXIMUM_AZIMUTH_VOLTAGE), //<---- Controller effort weight, how much we want to conserve controller expenditure or how fast we want to be; increased state error
                Values.DISCRETIZATION_TIMESTEP),
              new KalmanFilter<>(
                Nat.N2(),
                Nat.N1(),
                PLANT, 
                VecBuilder.fill(Units.degreesToRadians(Values.MODEL_VELOCITY_ACCURACY_DEGREES),Units.rotationsPerMinuteToRadiansPerSecond(Values.MODEL_POSITION_ACCURACY_RPM)), //<---- How accurate we think our models are
                VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(Values.ENCODER_POSITION_ACCURACY_DEGREES)), //<---- How accurate we think our encoders are
                Values.DISCRETIZATION_TIMESTEP),
              Values.MAXIMUM_AZIMUTH_VOLTAGE,
              Values.DISCRETIZATION_TIMESTEP);             
          }

        }
        public static final class RL_Module {


          public static final class Values {
            public static final Double MAXIMUM_AZIMUTH_ACCELERATION = (Math.PI*2);    
            public static final Double MAXIMUM_AZIMUTH_VOLTAGE = (12.0);            
            public static final Double MAXIMM_AZIMUTH_VELOCITY = (5.4);              
            public static final Double MAXIMUM_LINEAR_VELOCITY = (5.4);
            public static final Double ACCELERATION_GAIN = (1.0); //<---- Acceleration Gain (kA) of Linear Position System, Volts/(Units/Sec^2)
            public static final Double VELOCITY_GAIN = (1.0); //<---- Velocity Gain (kV) of Linear Position System, Volts/(Units/Sec)

            public static final Double ENCODER_POSITION_ACCURACY_DEGREES = (1/4096.0); 
            public static final Double MODEL_VELOCITY_ACCURACY_DEGREES = (7e-5);
            public static final Double MODEL_POSITION_ACCURACY_RPM = (2.5e-4);
            public static final Double DISCRETIZATION_TIMESTEP = (1/50.0);

            public static final Integer AZIMUTH_ID = (23);                       
            public static final Integer LINEAR_ID = (13);    
            public static final Integer SENSOR_ID = (6);
          }

          public static final class Components {
            public static final WPI_CANCoder AZIMUTH_SENSOR = new WPI_CANCoder(Values.SENSOR_ID);
            public static final WPI_TalonFX AZIMUTH_CONTROLLER = new WPI_TalonFX(Values.AZIMUTH_ID);                                 
            public static final WPI_TalonFX LINEAR_CONTROLLER = new WPI_TalonFX(Values.LINEAR_ID);
            public static final Supplier<SwerveModuleState> STATE_SENSOR = () -> new SwerveModuleState(
              2.0*(((LINEAR_CONTROLLER.getSelectedSensorVelocity() / 2048) * 10) / Constants.Values.DRIVETRAIN_GEAR_RATIO) * Math.PI * Constants.Values.WHEEL_DIAMETER,
              new Rotation2d(AZIMUTH_SENSOR.getAbsolutePosition()));            
            public static final SwerveModule MODULE = new SwerveModule(new MotorControllerGroup(AZIMUTH_CONTROLLER), new MotorControllerGroup(LINEAR_CONTROLLER),STATE_SENSOR,() -> Control.CONSTRAINTS,Control.CONTROL_LOOP, () -> Values.MAXIMUM_LINEAR_VELOCITY);
          }
      

          public static final class Control {
            public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(Values.MAXIMM_AZIMUTH_VELOCITY, Values.MAXIMUM_AZIMUTH_ACCELERATION);
            public static final LinearSystem<N2,N1,N1> PLANT = LinearSystemId.identifyPositionSystem(Values.VELOCITY_GAIN, Values.ACCELERATION_GAIN);        
            public static final LinearSystemLoop<N2,N1,N1> CONTROL_LOOP = new LinearSystemLoop<N2,N1,N1>(
              PLANT,
              new LinearQuadraticRegulator<>(
                PLANT,
                VecBuilder.fill(Units.degreesToRadians(Components.AZIMUTH_SENSOR.getAbsolutePosition()),Units.rotationsPerMinuteToRadiansPerSecond(Components.AZIMUTH_SENSOR.getVelocity())), //<---- Controller error weight, how much we want to conserve controller error, or how accurate we want to be
                VecBuilder.fill(Values.MAXIMUM_AZIMUTH_VOLTAGE), //<---- Controller effort weight, how much we want to conserve controller expenditure or how fast we want to be; increased state error
                Values.DISCRETIZATION_TIMESTEP),
              new KalmanFilter<>(
                Nat.N2(),
                Nat.N1(),
                PLANT, 
                VecBuilder.fill(Units.degreesToRadians(Values.MODEL_VELOCITY_ACCURACY_DEGREES),Units.rotationsPerMinuteToRadiansPerSecond(Values.MODEL_POSITION_ACCURACY_RPM)), //<---- How accurate we think our models are
                VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(Values.ENCODER_POSITION_ACCURACY_DEGREES)), //<---- How accurate we think our encoders are
                Values.DISCRETIZATION_TIMESTEP),
              Values.MAXIMUM_AZIMUTH_VOLTAGE,
              Values.DISCRETIZATION_TIMESTEP);          
          }

        }
        public static final class RR_Module {

          public static final class Values {
            public static final Double MAXIMUM_AZIMUTH_ACCELERATION = (Math.PI*2);    
            public static final Double MAXIMUM_AZIMUTH_VOLTAGE = (12.0);            
            public static final Double MAXIMM_AZIMUTH_VELOCITY = (5.4);              
            public static final Double MAXIMUM_LINEAR_VELOCITY = (5.4);
            public static final Double ACCELERATION_GAIN = (1.0); //<---- Acceleration Gain (kA) of Linear Position System, Volts/(Units/Sec^2)
            public static final Double VELOCITY_GAIN = (1.0); //<---- Velocity Gain (kV) of Linear Position System, Volts/(Units/Sec)

            public static final Double ENCODER_POSITION_ACCURACY_DEGREES = (1/4096.0); 
            public static final Double MODEL_VELOCITY_ACCURACY_DEGREES = (7e-5);
            public static final Double MODEL_POSITION_ACCURACY_RPM = (2.5e-4);
            public static final Double DISCRETIZATION_TIMESTEP = (1/50.0);

            public static final Integer AZIMUTH_ID = (24);                       
            public static final Integer LINEAR_ID = (14);    
            public static final Integer SENSOR_ID = (7);
          }

          public static final class Components {
            public static final WPI_CANCoder AZIMUTH_SENSOR = new WPI_CANCoder(Values.SENSOR_ID);
            public static final WPI_TalonFX AZIMUTH_CONTROLLER = new WPI_TalonFX(Values.AZIMUTH_ID);                                 
            public static final WPI_TalonFX LINEAR_CONTROLLER = new WPI_TalonFX(Values.LINEAR_ID);
            public static final Supplier<SwerveModuleState> STATE_SENSOR = () -> new SwerveModuleState(
              2.0*(((LINEAR_CONTROLLER.getSelectedSensorVelocity() / 2048) * 10) / Constants.Values.DRIVETRAIN_GEAR_RATIO) * Math.PI * Constants.Values.WHEEL_DIAMETER,
              new Rotation2d(AZIMUTH_SENSOR.getAbsolutePosition()));            
            public static final SwerveModule MODULE = new SwerveModule(new MotorControllerGroup(AZIMUTH_CONTROLLER), new MotorControllerGroup(LINEAR_CONTROLLER),STATE_SENSOR,() -> Control.CONSTRAINTS,Control.CONTROL_LOOP, () -> Values.MAXIMUM_LINEAR_VELOCITY);
          }
      

          public static final class Control {
            public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(Values.MAXIMM_AZIMUTH_VELOCITY, Values.MAXIMUM_AZIMUTH_ACCELERATION);
            public static final LinearSystem<N2,N1,N1> PLANT = LinearSystemId.identifyPositionSystem(Values.VELOCITY_GAIN, Values.ACCELERATION_GAIN);        
            public static final LinearSystemLoop<N2,N1,N1> CONTROL_LOOP = new LinearSystemLoop<N2,N1,N1>(
              PLANT,
              new LinearQuadraticRegulator<>(
                PLANT,
                VecBuilder.fill(Units.degreesToRadians(Components.AZIMUTH_SENSOR.getAbsolutePosition()),Units.rotationsPerMinuteToRadiansPerSecond(Components.AZIMUTH_SENSOR.getVelocity())), //<---- Controller error weight, how much we want to conserve controller error, or how accurate we want to be
                VecBuilder.fill(Values.MAXIMUM_AZIMUTH_VOLTAGE), //<---- Controller effort weight, how much we want to conserve controller expenditure or how fast we want to be; increased state error
                Values.DISCRETIZATION_TIMESTEP),
              new KalmanFilter<>(
                Nat.N2(),
                Nat.N1(),
                PLANT, 
                VecBuilder.fill(Units.degreesToRadians(Values.MODEL_VELOCITY_ACCURACY_DEGREES),Units.rotationsPerMinuteToRadiansPerSecond(Values.MODEL_POSITION_ACCURACY_RPM)), //<---- How accurate we think our models are
                VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(Values.ENCODER_POSITION_ACCURACY_DEGREES)), //<---- How accurate we think our encoders are
                Values.DISCRETIZATION_TIMESTEP),
              Values.MAXIMUM_AZIMUTH_VOLTAGE,
              Values.DISCRETIZATION_TIMESTEP);        
          }

        }    
    }    
  }


}
