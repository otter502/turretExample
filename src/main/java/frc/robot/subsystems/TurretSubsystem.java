package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.MotorSimulationSubsystem.PhysicalSimWrapper;
import frc.robot.subsystems.MotorSimulationSubsystem.SimSparkConstants;
import frc.robot.subsystems.MotorSimulationSubsystem.SimulatedSubsystem;
import frc.robot.util.ModeSwitchHandler.ModeSwitchInterface;

import static frc.robot.Constants.kStandardDt;
import static frc.robot.Constants.TurretConstants.*;

import java.util.Optional;

public class TurretSubsystem extends SubsystemBase implements SimulatedSubsystem, ModeSwitchInterface{
    
    //#region variables
    private final SparkMax mMotor;
    private final RelativeEncoder mEncoder;

    private final TrapezoidProfile mMotionProfileCalculator;

    private final SparkClosedLoopController mCLController;


    private Rotation2d mSetpoint;
    private State mCurrState;

    //#endregion

    //#region Utility functions
    /**
     * this function is used to ensure the conversion between the raw encoder value and the angle is consistent
     * @return a rotation2d with 0 degrees facing the front of the robot with positive being CCW
     */
    private Rotation2d getPosition(){
        return Rotation2d.fromDegrees(mEncoder.getPosition());
    }

    /**
     * this should the inverse of {@link #getPosition() getPosition()}
     * 
     * @param angle the intended angle of the system
     * @return the raw value to be used for the motor
     */
    private double convertAngleToRaw(Rotation2d angle){
        return angle.getDegrees();
    }
    //#endregion

    public TurretSubsystem() {
        super();

        mMotor = new SparkMax(kMotorID, kMotorType);
        mMotor.configure(kMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        mEncoder = mMotor.getEncoder();

        mEncoder.setPosition(convertAngleToRaw(kStartingAngle));

        mMotionProfileCalculator = new TrapezoidProfile(kConstraints);

        mCLController = mMotor.getClosedLoopController();

        resetMechanism();
    }


    public void resetMechanism(){
        var currentPosition = getPosition();
        mSetpoint = currentPosition;
        mCurrState = new State(convertAngleToRaw(currentPosition), 0.0);
    }


    //#region Command methods

    public void incrementAngle(Rotation2d delta){
        mSetpoint = mSetpoint.plus(delta);
    }

    public void setSetpoint(Rotation2d newAngle){
        mSetpoint = newAngle;
    }

    public void setMechanismAngle(Rotation2d angle){
        mEncoder.setPosition(convertAngleToRaw(angle));
        resetMechanism();
    }


    //#endregion

    @Override
    public void periodic() {
        mSetpoint = safetyZoneUpdate(mSetpoint);

        updateMotor();

        updateUserOutputs();

    }

    //#region Periodic methods

    /**
     * this function modifies the setpoint to be safe
     * for this turret it wraps the output, (this assumes the turret can rotate infinitely)
     * @param setpoint unsafe setpoint
     * @return safe setpoint
     */
    private Rotation2d safetyZoneUpdate(Rotation2d setpoint) {
        return Rotation2d.fromDegrees(
            MathUtil.inputModulus(setpoint.getDegrees(),
                0,
                360
            )
        );
    }

    /**
     * this function runs the motor's closed loop and motion profile update cycle
     */
    private void updateMotor(){
        mCurrState = mMotionProfileCalculator
            .calculate(
                kStandardDt, 
                mCurrState , 
                new State(convertAngleToRaw(mSetpoint), 0.0)
            );

        mCLController.setReference(
            mCurrState.position, 
            ControlType.kPosition, 
            ClosedLoopSlot.kSlot0, 
            kFeedForwardCalc.calculate(mCurrState.velocity)
        );
    }


    /**
     * this function updates the subsystem's logs as well as shuffleboard
     */
    private void updateUserOutputs(){

    }
    
    //#endregion

    //#region interfaces

    @Override
    public void onModeSwitch() {
        resetMechanism();
    }

    @Override
    public Optional<SimSparkConstants[]> getSparkSimConstants() {
        return Optional.of(new SimSparkConstants[]{
            new SimSparkConstants(
                mMotor, 
                new PhysicalSimWrapper(kSim),
                DCMotor.getNEO(1),
                kGearRatio,
                (s) -> {}
            )
        });
    }

    //#endregion

}
