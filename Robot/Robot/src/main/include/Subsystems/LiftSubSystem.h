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

/**
 *
 *
 * @author ExampleAuthor
 */
class LiftSubSystem: public frc::Subsystem {
private:
	// It's desirable that everything possible is private except
	// for methods that implement subsystem capabilities
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	std::shared_ptr<WPI_TalonSRX> liftMotor5;
	std::shared_ptr<WPI_TalonSRX> liftMotor6;
	std::shared_ptr<frc::SpeedControllerGroup> liftDrive;
	// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
public:
	LiftSubSystem();
	void InitDefaultCommand() override;
	void Periodic() override;
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
	// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS


};

#endif
