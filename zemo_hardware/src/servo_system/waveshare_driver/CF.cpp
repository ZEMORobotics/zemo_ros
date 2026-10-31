#include "zemo_hardware/servo_system/waveshare_driver/CF.h"
#include "zemo_hardware/servo_system/waveshare_driver/INST.h"
#include "zemo_hardware/servo_system/waveshare_driver/SyncWriteBuffer.h"

CF::CF(){
    End = 0;
}

/**
 * @brief Constructor for endian type
 * 
 * @param End 0 - little endian. 1 - big endian.
 */
CF::CF(u8 End) : SCSerial(End){}

/**
 * @brief Constructor for endian type
 *
 * @param End 0 - little endian. 1 - big endian.
 * @param Level Response level (0=no response, 1=response for read/write commands)
 */
CF::CF(u8 End, u8 Level) : SCSerial(End, Level) {}

/**
 * @brief Write position, speed, and acceleration to servo (extended command)
 *
 * Sends single-servo position command with acceleration and speed control.
 * Negative positions are converted to absolute value with direction bit set.
 *
 * @param ID Servo ID
 * @param Position Target position (-32767 - +32767, 0.087 degrees per unit)
 * @param Speed Max moving speed (0 - 32767, 0.732 RPM per unit) For CF 45RPM max is equal to 61 written here
 * @param ACC Acceleration value (0-254, 8.7 degree/s^2 per unit)
 * @return 1 on success, 0 on failure
 */
int CF::WritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC)
{
	u16 encodedPosition = ServoUtils::encodeSignedValue(Position, CF_DIRECTION_BIT_POS);

	u8 bBuf[7];
	bBuf[0] = ACC;
	Host2SCS(bBuf + 1, bBuf + 2, encodedPosition);
	Host2SCS(bBuf + 3, bBuf + 4, CF_DEFAULT_MAX_CURRENT);
	Host2SCS(bBuf + 5, bBuf + 6, Speed);

	return genWrite(ID, CF_ACC, bBuf, 7);
}

/**
 * @brief Register write position command (executes on RegWriteAction)
 *
 * Queues position/speed/acceleration command for later execution.
 * Use with RegWriteAction for synchronized multi-servo motion.
 *
 * @param ID Servo ID
 * @param Position Target position (-32767 - +32767, 0.087 degrees per unit)
 * @param Speed Max moving speed (0 - 32767, 0.732 RPM per unit) For CF 45RPM max is equal to 61 written here
 * @param ACC Acceleration value (0-254, 8.7 degree/s^2 per unit)
 * @return 1 on success, 0 on failure
 */
int CF::RegWritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC)
{
	u16 encodedPosition = ServoUtils::encodeSignedValue(Position, CF_DIRECTION_BIT_POS);

	u8 bBuf[7];
	bBuf[0] = ACC;
	Host2SCS(bBuf + 1, bBuf + 2, encodedPosition);
	Host2SCS(bBuf + 3, bBuf + 4, CF_DEFAULT_MAX_CURRENT);
	Host2SCS(bBuf + 5, bBuf + 6, Speed);

	return regWrite(ID, CF_ACC, bBuf, 7);
}

/**
 * @brief Synchronized position write for multiple servos
 *
 * Sends position/speed/acceleration commands to multiple servos simultaneously.
 * All servos receive commands in single transmission for coordinated motion.
 *
 * @param ID Array of servo IDs
 * @param IDN Number of servos
 * @param Position Array of target positions
 * @param Speed Array of speeds (NULL for 0 speed)
 * @param ACC Array of accelerations (NULL for MAX acceleration)
 */
void CF::SyncWritePosEx(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[])
{
	SyncWriteBuffer buffer(IDN, 7);
	if (!buffer.isValid()) {
		return;
	}
	for (u8 i = 0; i < IDN; i++) {
		u16 encodedPosition = ServoUtils::encodeSignedValue(Position[i], CF_DIRECTION_BIT_POS);

		u8 bBuf[7];
		u16 V = Speed ? Speed[i] : 0;
		bBuf[0] = ACC ? ACC[i] : 0;
		Host2SCS(bBuf + 1, bBuf + 2, encodedPosition);
		Host2SCS(bBuf + 3, bBuf + 4, CF_DEFAULT_MAX_CURRENT);
		Host2SCS(bBuf + 5, bBuf + 6, V);
		buffer.writeMotorData(i, bBuf, 7);
	}
	syncWrite(ID, IDN, CF_ACC, buffer.getBuffer(), 7);
}


/**
 * @brief Set servo operating mode
 *
 * The only allowed and verified mode is position mode, thank you:
 * - Mode 0: Servo position mode (default)
 *
 * @param ID Servo ID
 * @param mode Operating mode (0)
 * @return 1 on success, 0 on failure (invalid mode or communication error)
 */
int CF::Mode(u8 ID, u8 mode)
{
	if (mode != CF_MODE_SERVO){
		Err = 1;
		return 0;
	}
	Err = 0;
	return writeByte(ID, CF_MODE, mode);
}


// TODO: Add MULTITURN configuration here
/**
 * @brief Initialize motor with operating mode and torque setting
 *
 * 1. Unlocks EEPROM (to allow Operating_Mode write)
 * 2. Sets operating mode (0=position)
 * 3. Locks EEPROM (to save the mode setting)
 * 4. Optionally enables/disables torque
 *
 * @param ID Servo ID
 * @param mode Operating mode (0=servo)
 * @param enableTorque 1 to enable torque, 0 to disable (default: 1)
 * @return 1 on success, 0 on failure
 */
int CF::InitMotor(u8 ID, u8 mode, u8 enableTorque)
{
	Err = 0;

	// Unlock EEPROM for mode change.
	if(unLockEeprom(ID) == 0){
		Err = 1;
		return 0;
	}

	int mode_ret = Mode(ID, mode);

    if (mode_ret == 0) {
        LockEeprom(ID);
        Err = 1;
        return 0;
    }

    if (!writeWord(ID, CF_MIN_ANGLE_LIMIT_L, 0) || !writeWord(ID, CF_MAX_ANGLE_LIMIT_L, 0)) 
    {
        Err = 1;
        LockEeprom(ID);
        return 0;
    }

    int phase = readByte(ID, CF_PHASE);

    if (phase < 0 || !writeByte(ID, CF_PHASE, static_cast<u8>(phase | CF_PHASE_FULL_ANGLE_FEEDBACK_MASK))){
        Err = 1;
        LockEeprom(ID);
        return 0;
    }

	// We have to try to lock the eeprom either way.
	int lock_ret = LockEeprom(ID);

	if(lock_ret == 0){
		Err = 1;
		return 0;
	}

	if(EnableTorque(ID, enableTorque) == 0){
		Err = 1;
		return 0;
	}

	Err = 0;
	return 1;
}

/**
 * @brief Enable or disable servo motor torque
 *
 * @param ID Servo ID
 * @param Enable 1 to enable torque, 0 to disable (free-moving)
 * @return 1 on success, 0 on failure
 */
int CF::EnableTorque(u8 ID, u8 Enable)
{
	return writeByte(ID, CF_TORQUE_ENABLE, Enable);
}

/**
 * @brief Unlock EEPROM for writing
 *
 * @param ID Servo ID
 * @return 1 on success, 0 on failure
 */
int CF::unLockEeprom(u8 ID)
{
	return writeByte(ID, CF_LOCK, 0);
}

/**
 * @brief Lock EEPROM to prevent accidental changes
 *
 * @param ID Servo ID
 * @return 1 on success, 0 on failure
 */
int CF::LockEeprom(u8 ID)
{
	return writeByte(ID, CF_LOCK, 1);
}

int CF::WriteAcc(u8 ID, u8 value)
{
	return writeByte(ID, CF_ACC, value);
}

int CF::ReadAcc(int ID)
{
	return readByte(ID, CF_ACC);
}

/**
 * @brief Read all feedback data from servo into memory buffer
 *
 * Reads position, speed, load, voltage, temperature, movement status, and current.
 * Data is stored in Mem[] array and can be accessed via ReadPos(-1), ReadSpeed(-1), etc.
 *
 * @param ID Servo ID
 * @return 1 on success, 0 on failure
 */
int CF::FeedBack(int ID)
{
	int nLen = Read(ID, CF_PRESENT_POSITION_L, Mem, sizeof(Mem));
	if (nLen != sizeof(Mem)) {
		Err = 1;
		return 0;
	}
	Err = 0;
	return 1;
}

/**
 * @brief Read current servo position
 *
 * @param ID Servo ID, or -1 to read from cached buffer (after FeedBack())
 * @return Position value (-32767 to 32767), -1 on error
 */
int CF::ReadPos(int ID)
{
	if (ID == -1) {
		return ServoUtils::readSignedWordFromBuffer(Mem, CF_PRESENT_POSITION_L - CF_PRESENT_POSITION_L,
		                                            CF_PRESENT_POSITION_H - CF_PRESENT_POSITION_L,
		                                            CF_DIRECTION_BIT_POS);
	}
	Err = 0;
	return readSignedWord(ID, CF_PRESENT_POSITION_L, CF_DIRECTION_BIT_POS);
}

/**
 * @brief Read current servo speed
 *
 *
 * @param ID Servo ID, or -1 to read from cached buffer
 * @return Speed value (-32767 to 32767), -1 on error
 */
int CF::ReadSpeed(int ID)
{
	if (ID == -1) {
		return ServoUtils::readSignedWordFromBuffer(Mem, CF_PRESENT_SPEED_L - CF_PRESENT_POSITION_L,
		                                            CF_PRESENT_SPEED_H - CF_PRESENT_POSITION_L,
		                                            CF_DIRECTION_BIT_POS);
	}
	Err = 0;
	return readSignedWord(ID, CF_PRESENT_SPEED_L, CF_DIRECTION_BIT_POS);
}

/**
 * @brief Read current servo load
 *
 * Reads load torque from servo or from cached buffer.
 *
 * @param ID Servo ID, or -1 to read from cached buffer
 * @return Load value (-1000 to 1000, 0.1% per unit), -1 on error
 */
int CF::ReadLoad(int ID)
{
	if (ID == -1) {
		return ServoUtils::readSignedWordFromBuffer(Mem, CF_PRESENT_LOAD_L - CF_PRESENT_POSITION_L,
		                                            CF_PRESENT_LOAD_H - CF_PRESENT_POSITION_L,
		                                            CF_LOAD_DIRECTION_BIT_POS);
	}
	Err = 0;
	return readSignedWord(ID, CF_PRESENT_LOAD_L, CF_LOAD_DIRECTION_BIT_POS);
}

/**
 * @brief Read servo supply voltage
 *
 * @param ID Servo ID, or -1 to read from cached buffer
 * @return Voltage in 0.1V units (e.g., 120 = 12.0V), -1 on error
 */
int CF::ReadVoltage(int ID)
{
	int Voltage = -1;
	if (ID == -1) {
		Voltage = Mem[CF_PRESENT_VOLTAGE - CF_PRESENT_POSITION_L];
	} else {
		Err = 0;
		Voltage = readByte(ID, CF_PRESENT_VOLTAGE);
		if (Voltage == -1) {
			Err = 1;
		}
	}
	return Voltage;
}

/**
 * @brief Read servo internal temperature
 *
 * @param ID Servo ID, or -1 to read from cached buffer
 * @return Temperature in degrees Celsius, -1 on error
 */
int CF::ReadTemper(int ID)
{
	int Temper = -1;
	if (ID == -1) {
		Temper = Mem[CF_PRESENT_TEMPERATURE - CF_PRESENT_POSITION_L];
	} else {
		Err = 0;
		Temper = readByte(ID, CF_PRESENT_TEMPERATURE);
		if (Temper == -1) {
			Err = 1;
		}
	}
	return Temper;
}

/**
 * @brief Read servo movement status
 *
 *
 * @param ID Servo ID, or -1 to read from cached buffer
 * @return 1 if moving, 0 if stopped, -1 on error
 */
int CF::ReadMove(int ID)
{
	int Move = -1;
	if (ID == -1) {
		Move = Mem[CF_MOVING - CF_PRESENT_POSITION_L];
	} else {
		Err = 0;
		Move = readByte(ID, CF_MOVING);
		if (Move == -1) {
			Err = 1;
		}
	}
	return Move;
}

/**
 * @brief Read servo motor current
 *
 * @param ID Servo ID, or -1 to read from cached buffer
 * @return Current value in 6,5 mA, -1 on error
 */
int CF::ReadCurrent(int ID)
{
	if (ID == -1) {
		return ServoUtils::readSignedWordFromBuffer(Mem, CF_PRESENT_CURRENT_L - CF_PRESENT_POSITION_L,
		                                            CF_PRESENT_CURRENT_H - CF_PRESENT_POSITION_L,
		                                            CF_DIRECTION_BIT_POS);
	}
	Err = 0;
	return readSignedWord(ID, CF_PRESENT_CURRENT_L, CF_DIRECTION_BIT_POS);
}