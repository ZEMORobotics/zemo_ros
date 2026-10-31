#ifndef _CF_H_
#define _CF_H_

// Baud rate definition
#define CF_1M 0
#define CF_0_5M 1
#define CF_250K 2
#define CF_128K 3
#define CF_115200 4
#define CF_76800 5
#define CF_57600 6
#define CF_38400 7

// Memory table definition
//-------EEPROM (Read only)--------
#define CF_FIRMWARE_MAJOR 0
#define CF_FIRMWARE_MINOR 1
// Default is little endian. 
// Every macro in this file is written for little endian in case of command with 2 byte values.
#define CF_ENDIAN 2		
#define CF_SERVO_MAJOR 3
#define CF_SERVO_MINOR 4

//-------EEPROM (Read and Write)--------
#define CF_ID 5
#define CF_BAUD_RATE 6 
#define CF_SECONDARY_ID 7
#define CF_RESPONSE_STATUS_LEVEL 8
#define CF_MIN_ANGLE_LIMIT_L 9
#define CF_MIN_ANGLE_LIMIT_H 10

#define CF_MAX_ANGLE_LIMIT_L 11
#define CF_MAX_ANGLE_LIMIT_H 12
#define CF_MAX_TEMPERATURE_LIMIT 13
#define CF_MAX_INPUT_VOLT 14
#define CF_MIN_INPUT_VOLT 15
#define CF_MAX_TORQUE_L 16
#define CF_MAX_TORQUE_H 17
#define CF_PHASE 18
#define CF_UNLOADING_CONDITION 19
#define CF_LED_ALARM_CONDITION 20
#define CF_MODE0_P_COEF 21
#define CF_MODE0_D_COEF 22
#define CF_MODE0_I_COEF 23
#define CF_MINIMUM_STARTUP_FORCE 24     // Kod CF je 8bit vrednost, inace 16bit
#define CF_INTEGRAL_LIMIT_VALUE 25
#define CF_CW_DEAD 26
#define CF_CCW_DEAD 27
#define CF_PROTECTION_CURRENT_L 28
#define CF_PROTECTION_CURRENT_H 29
#define CF_ANGULAR_RESOLUTION 30
#define CF_OFS_L 31
#define CF_OFS_H 32
#define CF_MODE 33
#define CF_CURRENT_LOOP_P_COEF 34
#define CF_CURRENT_LOOP_I_COEF 35
// #define CF_UNDEFIEND 36
#define CF_MODE1_P_COEF 37
#define CF_OVER_CURRENT_PROTECTION_TIME 38
#define CF_MODE1_I_COEF 39

//-------SRAM (Read and Write)--------
#define CF_TORQUE_ENABLE 40
#define CF_ACC 41
#define CF_GOAL_POSITION_L 42   // min -32767 max +32767
#define CF_GOAL_POSITION_H 43
#define CF_GOAL_CURRENT_L 44
#define CF_GOAL_CURRENT_H 45
#define CF_GOAL_SPEED_L 46	// min -32767 max +32767
#define CF_GOAL_SPEED_H 47
#define CF_TORQUE_LIMIT_L 48
#define CF_TORQUE_LIMIT_H 49
#define CF_KP 50
#define CF_KD 51
#define CF_KI 52
// #define CF_UNDEFINED 53 54
#define CF_LOCK 55
          

//-------SRAM (Read only)--------
#define CF_PRESENT_POSITION_L 56
#define CF_PRESENT_POSITION_H 57
#define CF_PRESENT_SPEED_L 58
#define CF_PRESENT_SPEED_H 59
#define CF_PRESENT_LOAD_L 60
#define CF_PRESENT_LOAD_H 61
#define CF_PRESENT_VOLTAGE 62
#define CF_PRESENT_TEMPERATURE 63
#define CF_ASYNC_FLAG 64
#define CF_SERVO_STATUS 65
#define CF_MOVING 66
#define CF_CURRENT_TARGET_POS_L 67
#define CF_CURRENT_TARGET_POS_H 68
#define CF_PRESENT_CURRENT_L 69
#define CF_PRESENT_CURRENT_H 70
// #define CF_UNDEFINED 71 72
#define CF_CURRENT_OFFSET_L 73
#define CF_CURRENT_OFFSET_H 74


#define CF_DIRECTION_BIT_POS 15      // Bit position for direction flag in position/speed
#define CF_LOAD_DIRECTION_BIT_POS 10 // Bit position for direction flag in load/PWM
#define CF_PHASE_FULL_ANGLE_FEEDBACK_MASK (1u << 4)


#define CF_MODE_SERVO 0        // Servo mode (position control)
#define CF_MODE_WHEEL_CLOSED 1 // Wheel mode - closed loop (velocity control)
#define CF_MODE_CURRENT 2   // Current mode - holds the motor based on given current
#define CF_MODE_PWM 3      // PWM mode - something like regular DC motors

// Special servo IDs
#define CF_BROADCAST_ID 0xFE // Broadcast ID for all servos

#define CF_DEFAULT_MAX_CURRENT 2047
// #define CF_CALIBRATION_CMD 128

#include "INST.h"
#include "SCSerial.h"
#include "ServoErrors.h"
#include "ServoUtils.h"

class CF : public SCSerial
{
public:
	CF();
	/** @brief Constructor with protocol end byte
	 *  @param End Protocol end byte (0 or 1) */
	CF(u8 End);
	/** @brief Constructor with protocol end byte and response level
	 *  @param End Protocol end byte (0 or 1)
	 *  @param Level Response level (0=no response, 1=response enabled) */
	CF(u8 End, u8 Level);

    	/** @brief Write position to single servo (Mode 0)
	 *  @param ID Servo ID (0-253, 254=broadcast)
	 *  @param Position Target position (0-4095 steps)
	 *  @param Speed Movement speed (0-3400 steps/s)
	 *  @param ACC Acceleration (0-254, units of 100 steps/s²)
	 *  @return 1 on success, 0 on failure */
	virtual int WritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC = 0);

	/** @brief Async write position (execute with RegWriteAction)
	 *  @param ID Servo ID
	 *  @param Position Target position
	 *  @param Speed Movement speed
	 *  @param ACC Acceleration
	 *  @return 1 on success, 0 on failure */
	virtual int RegWritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC = 0);

	/** @brief Sync write position to multiple servos
	 *  @param ID Array of servo IDs
	 *  @param IDN Number of servos
	 *  @param Position Array of target positions
	 *  @param Speed Array of speeds (NULL for 0)
	 *  @param ACC Array of accelerations (NULL for 0) */
	virtual void SyncWritePosEx(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[]);

	/** @brief Set operating mode
	 *  @param ID Servo ID
	 *  @param mode 0=servo, 1=wheel closed-loop, 2=wheel open-loop
	 *  @return 1 on success, 0 on failure */
	virtual int Mode(u8 ID, u8 mode);

	/** @brief Initialize motor (unlock EEPROM → set mode → lock EEPROM → enable torque)
	 *  @param ID Servo ID
	 *  @param mode Operating mode (0/1/2)
	 *  @param enableTorque 1=enable, 0=disable (default: 1)
	 *  @return 1 on success, 0 on failure
	 *  @note LSP compliant - available on all servo classes */
	virtual int InitMotor(u8 ID, u8 mode, u8 enableTorque = 1);

	// /** @brief Write speed to single servo (Mode 1)
	//  *  @param ID Servo ID
	//  *  @param Speed Target speed (-3400 to +3400 steps/s)
	//  *  @param ACC Acceleration
	//  *  @return 1 on success, 0 on failure */
	// virtual int WriteSpe(u8 ID, s16 Speed, u8 ACC = 0);

	// /** @brief Async write speed */
	// virtual int RegWriteSpe(u8 ID, s16 Speed, u8 ACC = 0);

	// /** @brief Sync write speed and acceleration to multiple servos (atomic)
	//  *
	//  *  Sends ACC + GOAL_SPEED to all servos in a single 7-byte sync-write
	//  *  packet starting from SMS_STS_ACC (reg 41).  Writing both values in
	//  *  one atomic bus transaction ensures per-servo acceleration profiles
	//  *  are applied together with their speed commands, which is essential
	//  *  for synchronised multi-motor systems (e.g. omni-wheel robots).
	//  *
	//  *  @param ID    Array of servo IDs
	//  *  @param IDN   Number of servos
	//  *  @param Speed Array of target speeds (steps/s; negative = reverse)
	//  *  @param ACC   Array of accelerations (0-254, units of 100 steps/s²),
	//  *               or NULL to use ACC=0 for all servos (max slew rate) */
	// virtual void SyncWriteSpe(u8 ID[], u8 IDN, s16 Speed[], u8 ACC[]);

	/** @brief Enable/disable motor torque
	 *  @param ID Servo ID
	 *  @param Enable 1=enable, 0=disable (free-moving)
	 *  @return 1 on success, 0 on failure */
	virtual int EnableTorque(u8 ID, u8 Enable);

	/** @brief Unlock EEPROM for writing configuration
	 *  @param ID Servo ID
	 *  @return 1 on success, 0 on failure */
	virtual int unLockEeprom(u8 ID);

	/** @brief Lock EEPROM to protect configuration
	 *  @param ID Servo ID
	 *  @return 1 on success, 0 on failure */
	virtual int LockEeprom(u8 ID);

	/** @brief Set runtime acceleration register (addr 41)
	 *  @param ID Servo ID
	 *  @param value Acceleration register value (0-254)
	 *  @return 1 on success, 0 on failure */
	virtual int WriteAcc(u8 ID, u8 value);

	/** @brief Read runtime acceleration register (addr 41)
	 *  @param ID Servo ID
	 *  @return Register value on success, -1 on failure */
	virtual int ReadAcc(int ID);

	/** @brief Read all feedback data into internal buffer
	 *  @param ID Servo ID
	 *  @return 1 on success, 0 on failure
	 *  @note Call before using Read* methods with ID=-1 for cached reads */
	virtual int FeedBack(int ID);

	/** @brief Read current position
	 *  @param ID Servo ID, or -1 to read from cache (after FeedBack)
	 *  @return Position (0-4095), -1 on error */
	virtual int ReadPos(int ID);

	/** @brief Read current speed
	 *  @param ID Servo ID, or -1 for cached read
	 *  @return Speed (±3400 steps/s), -1 on error */
	virtual int ReadSpeed(int ID);

	/** @brief Read motor load
	 *  @param ID Servo ID, or -1 for cached read
	 *  @return Load (±1000 = ±100% PWM), -1 on error */
	virtual int ReadLoad(int ID);

	/** @brief Read supply voltage
	 *  @param ID Servo ID, or -1 for cached read
	 *  @return Voltage in 0.1V units, -1 on error */
	virtual int ReadVoltage(int ID);

	/** @brief Read internal temperature
	 *  @param ID Servo ID, or -1 for cached read
	 *  @return Temperature in °C, -1 on error */
	virtual int ReadTemper(int ID);

	/** @brief Read movement status
	 *  @param ID Servo ID, or -1 for cached read
	 *  @return 1=moving, 0=stopped, -1=error */
	virtual int ReadMove(int ID);

	/** @brief Read motor current
	 *  @param ID Servo ID, or -1 for cached read
	 *  @return Current in milliamps, -1 on error */
	virtual int ReadCurrent(int ID);

  private:
	u8 Mem[CF_PRESENT_CURRENT_H - CF_PRESENT_POSITION_L + 1];
};

#endif