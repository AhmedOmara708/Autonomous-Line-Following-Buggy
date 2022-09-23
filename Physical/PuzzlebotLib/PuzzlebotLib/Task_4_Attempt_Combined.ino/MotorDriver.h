/**
 * \file MotorDriver.h
 * \author Eduard Codres, Mario Martinez, Zachary Madin
 * \copyright Manchester Robotics
 * \date March, 2019
 * \brief MotorDriver class header file.
 */

#ifndef MotorDriver_h
#define MotorDriver_h

/**
 * Motor types
 */
#define BRUSHLESS_MOTOR 1
#define DC_MOTOR        2

/**
 * \brief Motor class. Handles the speed of brushless or dc motors.
 */
class MotorDriver
{
private:
    int PWMpin;          ///< pwm pin used to send signal to the driver
    int PinA;            ///< pin used for direction setting
    int PinB;            ///< pin used for direction setting
    int MotorBaseFreq;   ///< base freq of the brushless motor
    int Precision;       ///< precision of the pwm
    int Channel;         ///< pwm channel
    int type;            ///< motor type :  BRUSHLESS, DC_MOTOR
    bool toggle;         ///< flag used for changing direction
    int sign;            ///< sign - determines if the motor rotates forwards or backwards
        
public:

    /**
     * \brief Constructor
     */
    MotorDriver();
    
    /**
     * \brief Sets up the pins and pwm channel of the motor driver
     * \param PWMpin the pwm pin of the motor
     * \param Channel the pwm channel of the motor
     * \param PinA pin_a for dc motor, direction pin for brushless motor
     * \param PinB pin_b of the dc motor
     */
    void DriverSetup(int PWMpin, int Channel, int PinA, int PinB = 0);
    
    /**
     * \brief Write the pwm value to the motor driver
     * \param PWM PWM value sent to the motor
     */
    void MotorWrite(float PWM);
    
    /**
     * \brief Set the motor type
     * \param type BRUSHLESS,DC_MOTOR
     */
    void SetMotorType(int type);
    
    /**
     * \brief Set the base frequency for the brushless motor
     * \param BaseFreq base frequency used for the brushless motor
     */
    void SetBaseFreq(int BaseFreq);
    
    /**
     * \brief sets motor rotation direction (forwards or backwards)
     * \param sign motor sign
     */
    void SetSign(int sign);

};
    
#endif
