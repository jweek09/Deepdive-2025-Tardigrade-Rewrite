package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import java.util.concurrent.TimeUnit;

import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

public class ElevatorSubsystem {
    SparkMax kLeftMotorSparkMax = new SparkMax(ElevatorConstants.kLeftElevatorMotor, SparkLowLevel.MotorType.kBrushless);
    SparkMax kRightMotorSparkMax = new SparkMax(ElevatorConstants.kRightElevatorMotor, SparkLowLevel.MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    RelativeEncoder kPrimaryEncoder = kLeftMotorSparkMax.getEncoder();
     public ElevatorSubsystem() {
         config.idleMode(IdleMode.kBrake);
         config.inverted(true);
         kLeftMotorSparkMax.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
         config.inverted(false);
         kRightMotorSparkMax.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
         kPrimaryEncoder.setPosition(0);
     }
     public Command goUpALittleForTestingAndDeleteThisCommandLater() {
         return run( () -> {
             kLeftMotorSparkMax.set(0.1);
             kRightMotorSparkMax.set(0.1);
             try {
                 Thread.sleep(3000);
             } catch (InterruptedException e) {
                 throw new RuntimeException(e);
             }
             kLeftMotorSparkMax.set(0);
             kRightMotorSparkMax.set(0);
         });
     }
}
