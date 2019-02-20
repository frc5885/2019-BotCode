/**
 * Example demonstrating the Position closed-loop servo.
 * Tested with Logitech F350 USB Gamepad inserted into Driver Station]
 *
 * Be sure to select the correct feedback sensor using configSelectedFeedbackSensor() below.
 *
 * After deploying/debugging this to your RIO, first use the left Y-stick
 * to throttle the Talon manually.  This will confirm your hardware setup.
 * Be sure to confirm that when the Talon is driving forward (green) the
 * position sensor is moving in a positive direction.  If this is not the cause
 * flip the boolean input to the reverseSensor() call below.
 *
 * Once you've ensured your feedback device is in-phase with the motor,
 * use the button shortcuts to servo to target position.
 *
 * Tweak the PID gains accordingly.
 */
#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "Constants.h"

using namespace frc;

int SignOf(double value)
{
	return (value < 0.0 ? -1 : 1);
}

class Robot: public TimedRobot {
public:
	TalonSRX * _talon5 = new TalonSRX(5);
	TalonSRX * _talon7 = new TalonSRX(7);
	Joystick * _joy = new Joystick(0);
	std::string _sb;
	int _loops = 0;
	bool _lastButton1 = false;
	/** save the target position to servo to */
	int _targetPosition = 0;

	void RobotInit() 
	{
		/* choose the sensor and sensor direction */
		_talon5->ConfigSelectedFeedbackSensor(
				FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
		_talon5->SetSensorPhase(true);

		_talon7->ConfigSelectedFeedbackSensor(
				FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
		_talon7->SetSensorPhase(true);

		// set the peak and nominal outputs, +-1 means full
		_talon5->ConfigNominalOutputForward(0, kTimeoutMs);
		_talon5->ConfigNominalOutputReverse(0, kTimeoutMs);
		_talon5->ConfigPeakOutputForward(.5, kTimeoutMs);
		_talon5->ConfigPeakOutputReverse(-.5, kTimeoutMs);

		_talon7->ConfigNominalOutputForward(0, kTimeoutMs);
		_talon7->ConfigNominalOutputReverse(0, kTimeoutMs);
		_talon7->ConfigPeakOutputForward(1.0, kTimeoutMs);
		_talon7->ConfigPeakOutputReverse(-1.0, kTimeoutMs);

		// set closed loop gains in slot0
		_talon5->Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		_talon5->Config_kP(kPIDLoopIdx, 0.1, kTimeoutMs);
		_talon5->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
		_talon5->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);

		_talon7->Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		_talon7->Config_kP(kPIDLoopIdx, 0.1, kTimeoutMs);
		_talon7->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
		_talon7->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);
	}

	void TeleopInit()
	{
		// zero out the sensors
		_talon5->SetSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
		_talon7->SetSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
	}

	/**
	 * This function is called periodically during operator control
	 */
	void TeleopPeriodic() 
	{
		/* get gamepad axis */
		double leftYstick = _joy->GetY();
		_talon5->Set(ControlMode::PercentOutput, leftYstick);

		// not used right now...
		bool button1 = _joy->GetRawButton(1);
		bool button2 = _joy->GetRawButton(2);

		// set the target for talon7 to the current position of talon5
		_targetPosition = _talon5->GetSelectedSensorPosition(kPIDLoopIdx);
		_talon7->Set(ControlMode::Position, _targetPosition);

		/* prepare line to print */
		_sb.append("\tcommanded posn:");
		_sb.append(std::to_string(_targetPosition));
		_sb.append("\tfollowing posn:");
		_sb.append(std::to_string(_talon7->GetSelectedSensorPosition(kPIDLoopIdx)));

		/* append more signals to print when in speed mode. */
		_sb.append("\terrNative:");
		_sb.append(std::to_string(_talon7->GetClosedLoopError(kPIDLoopIdx)));

		/* print every ten loops, printing too much too fast is generally bad for performance */
		if (++_loops >= 10)
		{
			_loops = 0;
			printf("%s\n", _sb.c_str());
		}
		_sb.clear();

		/* save button state for on press detect */
		_lastButton1 = button1;
	}
};

int main()
{
	 return frc::StartRobot<Robot>();
}
