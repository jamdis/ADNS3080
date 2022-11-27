
#include <arduino.h>
#include <SPI.h>

#ifndef ADNS3080_h
#define ADNS3080_h 

//------------ Constants and registers ---------------

// Signal delay time:
#define ADNS3080_T_IN_RST             500
#define ADNS3080_T_PW_RESET           10
#define ADNS3080_T_SRAD_MOT           75
#define ADNS3080_T_SWW                50
#define ADNS3080_T_SRAD               50
#define ADNS3080_T_LOAD               10
#define ADNS3080_T_BEXIT              4

// Pixel dimensions:
#define ADNS3080_PIXELS_X             30
#define ADNS3080_PIXELS_Y             30

// Registers: 
#define ADNS3080_PRODUCT_ID                    0x00
#define ADNS3080_REVISION_ID                   0x01
#define ADNS3080_INVERSE_PRODUCT_ID            0x3f
#define ADNS3080_MOTION                        0X02
#define ADNS3080_DELTA_X                       0X03
#define ADNS3080_DELTA_Y                       0X04
#define ADNS3080_SQUAL                         0X05
#define ADNS3080_PIXEL_SUM                     0X06
#define ADNS3080_MAX_PIXEL                     0X07
#define ADNS3080_CONFIGURATION_BITS            0x0a
#define ADNS3080_EXTENDED_CONFIG               0x0b
#define ADNS3080_MOTION_CLEAR                  0x12
#define ADNS3080_FRAME_CAPTURE                 0x13
#define ADNS3080_PIXEL_BURST                   0x40
#define ADNS3080_MOTION_BURST                  0x50
#define ADNS3080_PRODUCT_ID_VALUE              0x17
#define ADNS3080_FRAME_PERIOD_LOWER            0x10
#define ADNS3080_FRAME_PERIOD_UPPER            0x11
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER  0x19
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER  0x1a
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER  0x1b
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER  0x1c
#define ADNS3080_SHUTTER_MAX_BOUND_LOWER       0x1d
#define ADNS3080_SHUTTER_MAX_BOUND_UPPER       0x1e

//--------------- Template Parameters ---------------- [ No characters after backlash! ]

#define TEMPLATE_TYPE           \
        uint8_t PIN_RESET,      \
        uint8_t PIN_NCS         
    
#define TEMPLATE_INPUTS         \
                PIN_RESET,      \
                PIN_NCS 
  
//---------------- Class definition ------------------
  
template <TEMPLATE_TYPE>
class ADNS3080 {  
  private:
	//Get bit n of a byte
	bool getBit( uint8_t number, uint8_t n ); 
	
	//set bit n of a byte
	uint8_t setBit( uint8_t number, uint8_t n, bool set_to );
	
	uint16_t concatenateBytes( uint8_t upper, uint8_t lower );
	uint8_t upperByte(uint16_t input ); //TODO
	uint8_t lowerByte(uint16_t input ); //TODO
	
   
  public:
  	// Read and write registers:
    void writeRegister( const uint8_t, uint8_t );
    uint8_t readRegister( const uint8_t );
    
    // Tests
    uint8_t getProductID();
    uint8_t getRevisionID();
    uint8_t getInverseProductID();
    bool testProductID();  
     
    // Miscellaneous functions:
    void reset();
    bool setup( const bool, const bool );
    void motionClear();
    uint8_t getMotion();
    uint8_t getDeltaX();
    uint8_t getDeltaY();
    uint8_t getSqual();
    uint8_t getPixelSum();
    uint8_t getMaxPixel();
    
    //configuration 
    uint8_t getConfig();
    void setConfig( uint8_t new_config );
    uint8_t getExtConfig();
    void setExtConfig( uint8_t new_ext_config );
    bool getLedMode();
    void setLedMode( bool set_to );
    bool getResolution();
    void setResolution( bool set_to );
    
    //exposure
    bool getBusy();
    bool getManualFrameRate();
    void setManualFrameRate( bool set_to );
    bool getManualShutter();
    void setManualShutter( bool set_to );
    uint16_t getShutter();
	void setShutter( uint16_t set_to ); //actually sets this via the shuttermaxbound register
    uint16_t getShutterMaxBound();
    uint16_t getFramePeriod();
    uint16_t getFramePeriodMaxBound();
    void setFramePeriodMaxBound( uint16_t set_to );
    /*
    
    //TODO
    uint16_t getFramePeriodMinBound();//TODO
    void setFramePeriodMinBound( uint16_t set_to );//TODO
    //TODO
    void setExposure( uint16_t set_to );//TODO
    */
    
    
    // Major outputs:
    void motionBurst( uint8_t*, uint8_t*, int8_t*, int8_t*, uint8_t*, uint16_t*, uint8_t* );
    void displacement( int8_t*, int8_t* );
    void frameCapture( uint8_t[ADNS3080_PIXELS_X][ADNS3080_PIXELS_Y] );
};

#include "ADNS3080.tpp"

#endif
