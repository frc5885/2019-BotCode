// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Subsystems/LiftSubSystem.h"
#include "Robot.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

LiftSubSystem::LiftSubSystem() : frc::Subsystem("LiftSubSystem")
{
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    liftMotor5.reset(new WPI_TalonSRX(5));
    liftMotor6.reset(new WPI_TalonSRX(6));
    liftMotor7.reset(new WPI_TalonSRX(7));

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    this->liftMotor5->SetNeutralMode(NeutralMode::Brake);
    this->liftMotor6->SetNeutralMode(NeutralMode::Brake);
    this->liftMotor7->SetNeutralMode(NeutralMode::Brake);

    /////// closed loop mode setup for talons 5 and 7 ///////

    /* choose the sensor and sensor direction */
    this->liftMotor5->ConfigSelectedFeedbackSensor(
            FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
    this->liftMotor5->SetSensorPhase(true);

    this->liftMotor7->ConfigSelectedFeedbackSensor(
            FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
    this->liftMotor7->SetSensorPhase(true);

    // set the peak and nominal outputs, +-1 means full
    this->liftMotor5->ConfigNominalOutputForward(0, kTimeoutMs);
    this->liftMotor5->ConfigNominalOutputReverse(0, kTimeoutMs);
    this->liftMotor5->ConfigPeakOutputForward(.6, kTimeoutMs);
    this->liftMotor5->ConfigPeakOutputReverse(-.6, kTimeoutMs);

    this->liftMotor7->ConfigNominalOutputForward(0, kTimeoutMs);
    this->liftMotor7->ConfigNominalOutputReverse(0, kTimeoutMs);
    this->liftMotor7->ConfigPeakOutputForward(1.0, kTimeoutMs);
    this->liftMotor7->ConfigPeakOutputReverse(-1.0, kTimeoutMs);

    // set closed loop gains in slot0
    this->liftMotor5->Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
    this->liftMotor5->Config_kP(kPIDLoopIdx, 0.1, kTimeoutMs);
    this->liftMotor5->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
    this->liftMotor5->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);

    this->liftMotor7->Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
    this->liftMotor7->Config_kP(kPIDLoopIdx, 0.1, kTimeoutMs);
    this->liftMotor7->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
    this->liftMotor7->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);


    // our local variables
    this->frontMotorSpeed = 0.0;
    this->rearMotorSpeed = 0.0;
    this->leftJoystickY = 0.0;
    this->rightJoystickY = 0.0;
    this->targetHeight = 10.0;      /// set this to experimentally determined target height!!!
    this->logOutput = false;
    this->logStrLoops = 0;
    this->logStrDelay = 10;
}

void LiftSubSystem::InitDefaultCommand() {
    // Set the default command for a subsystem here.
    // SetDefaultCommand(new MySpecialCommand());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
}

void LiftSubSystem::Periodic()
{
    switch (Robot::liftMode)
    {
    case LiftMode::Manual:
        // drive front & rear lifts independently
        this->RunOpenLoop();
        break;

    case LiftMode::Level:
        // drive all 3 talons, keeping the robot level using left joystick
        // operator drives rear lift, front lift follows
        this->RunClosedLoop();
        break;

    case LiftMode::AutoLevel:
        // drive all 3 talons to a target height, keeping the robot level using left joystick
        this->RunAutoLevel();
        break;
 
    case LiftMode::RetractFront:
        // retract the front lift prior to driving on the platform
        this->RunRetractFrontLift();
        break;

    default:
        break;
    }
}

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


// Put methods for controlling this subsystem
// here. Call these from Commands.

void LiftSubSystem::ZeroSensors()
{
    // zero out the sensors
    this->liftMotor5->SetSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
    this->liftMotor7->SetSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
}

void LiftSubSystem::StopMotors()
{
    this->liftMotor5->Set(ControlMode::PercentOutput, 0.0);
    this->liftMotor6->Set(ControlMode::PercentOutput, 0.0);
    this->liftMotor7->Set(ControlMode::PercentOutput, 0.0);
}

void LiftSubSystem::RunOpenLoop()
{
    if (Robot::liftMode != LiftMode::Manual)
    {
        return;
    }

     this->liftMotor5->Set(ControlMode::PercentOutput, this->frontMotorSpeed);
     this->liftMotor7->Set(ControlMode::PercentOutput, this->rearMotorSpeed);
}

void LiftSubSystem::RunClosedLoop()
{
    if (Robot::liftMode != LiftMode::Level)
    {
        return;
    }

    // move the rear lift (talon5) at a speed based on joystick position
    this->liftMotor5->Set(ControlMode::PercentOutput, this->leftJoystickY);

    // set the target for talon7 to the current position of talon5
    int targetPosition = this->liftMotor5->GetSelectedSensorPosition(kPIDLoopIdx);
    this->liftMotor7->Set(ControlMode::Position, targetPosition);

    // print output, if required
    PrintOutput(targetPosition);
}

void LiftSubSystem::RunAutoLevel()
{
    if (Robot::liftMode != LiftMode::AutoLevel)
    {
        return;
    }

    // drive rear lift to target height
    int targetPosition = (int)((double)(this->targetHeight * kCountsPerInch));
    this->liftMotor5->Set(ControlMode::Position, this->targetHeight);

    // set the target for talon7 to the current position of talon5
    targetPosition = this->liftMotor5->GetSelectedSensorPosition(kPIDLoopIdx);
    this->liftMotor7->Set(ControlMode::Position, targetPosition);

    // if we are at the target height, change mode to retract front lift
    if (std::abs(this->liftMotor7->GetClosedLoopError(kPIDLoopIdx)) < 10)
    {
        Robot::liftMode = LiftMode::RetractFront;
    }

    // Print the output, if required
    PrintOutput(targetPosition);
}

void LiftSubSystem::RunRetractFrontLift()
{
     if (Robot::liftMode != LiftMode::RetractFront)
    {
        return;
    }

    // hold rear lift at target height
    int targetPosition = (int)((double)(this->targetHeight * kCountsPerInch));
    this->liftMotor5->Set(ControlMode::Position, this->targetHeight);

    // set the target for talon7 to 0 to retract it
    this->liftMotor7->Set(ControlMode::Position, 0);

    // Print the output, if required
    PrintOutput(0);
}

void LiftSubSystem::PrintOutput(int tgtPos)
{
    /* print every N loops, printing too much too fast is generally bad for performance */
    if (this->logOutput && ++this->logStrLoops >= this->logStrDelay)
    {
        /* prepare line to print */
        this->logStr.clear();
        this->logStr.append("\tcommanded posn:");
        this->logStr.append(std::to_string(tgtPos));
        this->logStr.append("\tfollowing posn:");
        this->logStr.append(std::to_string(this->liftMotor7->GetSelectedSensorPosition(kPIDLoopIdx)));

        this->logStr.append("\terrNative:");
        this->logStr.append(std::to_string(this->liftMotor7->GetClosedLoopError(kPIDLoopIdx)));

        this->logStrLoops = 0;
        printf("%s\n", this->logStr.c_str());
    }
}
