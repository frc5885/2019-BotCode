// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#ifndef LIFTSUBSYSTEM_H
#define LIFTSUBSYSTEM_H
#include "frc/commands/Subsystem.h"
#include "frc/WPILib.h"
#include "ctre/Phoenix.h"

enum Constants 
{
	/**
	 * Which PID slot to pull gains from.  Starting 2018, you can choose
	 * from 0,1,2 or 3.  Only the first two (0,1) are visible in web-based configuration.
	 */
	kSlotIdx = 0,

	/* Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops.
	 * For now we just want the primary one.
	 */
	kPIDLoopIdx = 0,

	/*
	 * set to zero to skip waiting for confirmation, set to nonzero to wait
	 * and report to DS if action fails.
	 */
	kTimeoutMs = 30,

	// encoder counts per inch of rack movement
	kCountsPerInch = 73015
};

/**
 *
 *
 * @author ExampleAuthor
 */
class LiftSubSystem: public frc::Subsystem 
{
private:
	// It's desirable that everything possible is private except
	// for methods that implement subsystem capabilities
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	std::shared_ptr<WPI_TalonSRX> liftMotor5;
	std::shared_ptr<WPI_TalonSRX> liftMotor6;
	std::shared_ptr<WPI_TalonSRX> liftMotor7;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
public:
	LiftSubSystem();
	void InitDefaultCommand() override;
	void Periodic() override;
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS


// Properties
private:
    double frontMotorSpeed, rearMotorSpeed;  // speed for lift motors
    double leftJoystickY, rightJoystickY;    // left,right joystick Y positions 
    bool logOutput;         // log closed loop output
    std::string logStr;     // string for logged output
    int logStrLoops;        // loop counter for logged strings
    int logStrDelay;        // delay in output for logged strings
	double targetHeight;	// height to lift robot for habitat
	double heightOffset;	// constant height offset between front and rear racks
// Methods
public:
    void ZeroSensors();
	void StopMotors();	

    void SetFrontMotorSpeed(const double& speed)
    {
        if (speed >= -1.0 && speed <= 1.0)
        {
            this->frontMotorSpeed = speed;
        }
    }

    void SetRearMotorSpeed(const double& speed)
    {
        if (speed >= -1.0 && speed <= 1.0)
        {
			this->rearMotorSpeed = speed;
        }
    }

    void SetLeftJoystickY(const double& yValue)
    {
        this->leftJoystickY = yValue;
    }

	void SetTargetHeight(double tgtHt)
	{
		this->targetHeight = tgtHt;
	}

private:
	void RunOpenLoop();
    void RunClosedLoop();
	void RunAutoLevel();
	void RunRetractFrontLift();
	void PrintOutput(int tgtPos);
};

#endif
