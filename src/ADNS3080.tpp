//------------ Read and write registers --------------
 
template<TEMPLATE_TYPE>
void ADNS3080<TEMPLATE_INPUTS>
::writeRegister( const uint8_t reg, uint8_t output ) {
  // Enable communication
  digitalWrite( PIN_NCS, LOW );
  SPI.transfer( reg | B10000000 );
  
  // Send value
  SPI.transfer( output );
  
  // Disable communcation
  digitalWrite( PIN_NCS, HIGH );
  delayMicroseconds( ADNS3080_T_SWW );
}

template<TEMPLATE_TYPE>
uint8_t ADNS3080<TEMPLATE_INPUTS>
::readRegister( const uint8_t reg ) {
  // Enable communication
  digitalWrite( PIN_NCS, LOW );
  SPI.transfer( reg );
  delayMicroseconds( ADNS3080_T_SRAD_MOT );
  
  // Receive value
  uint8_t output = SPI.transfer(0x00);
  
  // Dissable communication
  digitalWrite( PIN_NCS, HIGH ); 
  return output;
}

template<TEMPLATE_TYPE>
void ADNS3080<TEMPLATE_INPUTS>
::writeDoubleRegister( uint16_t set_to, uint8_t lower_register ) {	
  uint8_t upper = upperByte( set_to );
  uint8_t lower = lowerByte( set_to );
  writeRegister( lower_register, lower ); //set lower first
  writeRegister( lower_register + 1, upper ); 
}

template<TEMPLATE_TYPE>
uint16_t ADNS3080<TEMPLATE_INPUTS>
::readDoubleRegister( uint8_t lower_register ) {
  uint8_t upper = readRegister( lower_register + 1 ); //Read upper first
  uint8_t lower = readRegister( lower_register ); 
  return concatenateBytes( upper, lower );
}




//------------ Miscellaneous functions ---------------

// Cycle reset pin
template<TEMPLATE_TYPE>
void ADNS3080<TEMPLATE_INPUTS>
::reset() {
  digitalWrite( PIN_RESET, HIGH );
  delayMicroseconds( ADNS3080_T_PW_RESET );                  
  digitalWrite( PIN_RESET, LOW );
  delayMicroseconds( ADNS3080_T_IN_RST );              
}

// Initialize and configure sensor
template<TEMPLATE_TYPE>
bool ADNS3080<TEMPLATE_INPUTS>
::setup( const bool led_mode, const bool resolution ) {
  
  // Configure SPI
  SPI.begin();
  SPI.setClockDivider( SPI_CLOCK_DIV32 );
  SPI.setDataMode( SPI_MODE3 );
  SPI.setBitOrder( MSBFIRST );  
  
  // Set sensor pins
  pinMode( PIN_RESET, OUTPUT );
  pinMode( PIN_NCS,   OUTPUT );

  // Disable communication and reset 
  digitalWrite( PIN_NCS, HIGH );
  reset();
 
  // Configure sensor:
  //                          LED Shutter    High resolution
  uint8_t mask = B00000000 | led_mode << 6 | resolution << 4;      
  writeRegister( ADNS3080_CONFIGURATION_BITS, mask );

  // Check Connection
  if( readRegister(ADNS3080_PRODUCT_ID) == ADNS3080_PRODUCT_ID_VALUE ) {
    return true;
  } else {
    return false;
  } 
}

// Read the motion register
template<TEMPLATE_TYPE>
uint8_t ADNS3080<TEMPLATE_INPUTS>
::getMotion() {
  return readRegister( ADNS3080_MOTION );
}

// Read delta x and delta y
template<TEMPLATE_TYPE>
uint8_t ADNS3080<TEMPLATE_INPUTS>
::getDeltaX() {
  return readRegister( ADNS3080_DELTA_X );
}

template<TEMPLATE_TYPE>
uint8_t ADNS3080<TEMPLATE_INPUTS>
::getDeltaY() {
  return readRegister( ADNS3080_DELTA_Y );
}

template<TEMPLATE_TYPE>
uint8_t ADNS3080<TEMPLATE_INPUTS>
::getSqual() {
  return readRegister( ADNS3080_SQUAL );
}

template<TEMPLATE_TYPE>
uint8_t ADNS3080<TEMPLATE_INPUTS>
::getPixelSum() {
  return readRegister( ADNS3080_PIXEL_SUM );
}

template<TEMPLATE_TYPE>
uint8_t ADNS3080<TEMPLATE_INPUTS>
::getMaxPixel() {
  return readRegister( ADNS3080_MAX_PIXEL );
}

// Clear DELTA_X, DELTA_Y and motion registers
template<TEMPLATE_TYPE>
void ADNS3080<TEMPLATE_INPUTS>
::motionClear() {
  writeRegister( ADNS3080_MOTION_CLEAR, 0x00 );
}


//----------------- Config ---------------------------
template<TEMPLATE_TYPE>
uint8_t ADNS3080<TEMPLATE_INPUTS>
::getConfig() {
  return readRegister( ADNS3080_CONFIGURATION_BITS );
}

template<TEMPLATE_TYPE>
bool ADNS3080<TEMPLATE_INPUTS>
::getLedMode() {
  uint8_t config = getConfig();
  bool led_mode = getBit( config, 6 );
  return led_mode;
}

template<TEMPLATE_TYPE>
void ADNS3080<TEMPLATE_INPUTS>
::setLedMode( bool set_to ) {
  uint8_t config = getConfig();
  uint8_t new_config = setBit( config, 6 , set_to );
  setConfig( new_config );
}

template<TEMPLATE_TYPE>
bool ADNS3080<TEMPLATE_INPUTS>
::getResolution() {
  uint8_t config = getConfig();
  bool resolution = getBit( config, 4 );
  return resolution;
}

template<TEMPLATE_TYPE>
void ADNS3080<TEMPLATE_INPUTS>
::setResolution( bool set_to ) {
  uint8_t config = getConfig();
  uint8_t new_config = setBit( config, 4 , set_to );
  setConfig( new_config );
}

template<TEMPLATE_TYPE>
void ADNS3080<TEMPLATE_INPUTS>
::setConfig( uint8_t new_config ) {
  writeRegister( ADNS3080_CONFIGURATION_BITS, new_config );
}

template<TEMPLATE_TYPE>
uint8_t ADNS3080<TEMPLATE_INPUTS>
::getExtConfig() {
  return readRegister( ADNS3080_EXTENDED_CONFIG );
}

template<TEMPLATE_TYPE>
void ADNS3080<TEMPLATE_INPUTS>
::setExtConfig( uint8_t new_ext_config ) {
  writeRegister( ADNS3080_EXTENDED_CONFIG, new_ext_config );
}


//----------------- Exposure ----------------------------

template<TEMPLATE_TYPE>
bool ADNS3080<TEMPLATE_INPUTS>
::getBusy() {
  uint8_t ext_config = getExtConfig();
  bool busy = getBit( ext_config, 7 );
  return busy;
}

template<TEMPLATE_TYPE>
bool ADNS3080<TEMPLATE_INPUTS>
::getManualShutter() {
  uint8_t ext_config = getExtConfig();
  bool manual_shutter = getBit( ext_config, 1 );
  return manual_shutter;
}

template<TEMPLATE_TYPE>
void ADNS3080<TEMPLATE_INPUTS>
::setManualShutter( bool set_to ) {
  uint8_t ext_config = getExtConfig();
  uint8_t new_config = setBit( ext_config, 1 , set_to );
  setConfig( new_config );
}

template<TEMPLATE_TYPE>
bool ADNS3080<TEMPLATE_INPUTS>
::getManualFrameRate() {
  uint8_t ext_config = getExtConfig();
  bool manual_frame_rate = getBit( ext_config, 0 );
  return manual_frame_rate;
}

template<TEMPLATE_TYPE>
void ADNS3080<TEMPLATE_INPUTS>
::setManualFrameRate( bool set_to ) {
  uint8_t ext_config = getExtConfig();
  uint8_t new_config = setBit( ext_config, 0 , set_to );
  setConfig( new_config );
}

template<TEMPLATE_TYPE>
uint16_t ADNS3080<TEMPLATE_INPUTS>
::getShutterMaxBound( ) {
  return readDoubleRegister( ADNS3080_SHUTTER_MAX_BOUND_LOWER );
}

template<TEMPLATE_TYPE>
void ADNS3080<TEMPLATE_INPUTS>
::setShutterMaxBound( uint16_t set_to ) {	//Setting shutter and setting shutter_max_bound are the same register
  writeDoubleRegister( set_to, ADNS3080_SHUTTER_MAX_BOUND_LOWER );
}

template<TEMPLATE_TYPE>
uint16_t ADNS3080<TEMPLATE_INPUTS>
::getFramePeriod( ) {
  return readDoubleRegister( ADNS3080_FRAME_PERIOD_LOWER );
}

template<TEMPLATE_TYPE>
uint16_t ADNS3080<TEMPLATE_INPUTS>
::getFramePeriodMaxBound( ) {
  return readDoubleRegister( ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER );
}

template<TEMPLATE_TYPE>
void ADNS3080<TEMPLATE_INPUTS>
::setFramePeriodMaxBound( uint16_t set_to ) {	
  writeDoubleRegister( set_to, ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER );
}

template<TEMPLATE_TYPE>
uint16_t ADNS3080<TEMPLATE_INPUTS>
::getFramePeriodMinBound( ) {
  return readDoubleRegister( ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER );
}

template<TEMPLATE_TYPE>
void ADNS3080<TEMPLATE_INPUTS>
::setFramePeriodMinBound( uint16_t set_to ) {	
  writeDoubleRegister( set_to, ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER ); 
}

template<TEMPLATE_TYPE>
bool ADNS3080<TEMPLATE_INPUTS>
::setExposure( bool manual_fp, bool manual_shutter, uint16_t frame_period_max, uint16_t frame_period_min, uint16_t shutter_max) {	
	//this function ensures that exposure registers are set in the proper order to
	//actually take efect, and enforces the rules laid out in the datasheet
	//returns 1 on error
	
	//Sanity checks on parameters:
	if( !manual_fp && manual_shutter ){ //Can’t have manual shutter without manual frame rate
		return 1;
	}
	if( frame_period_min < ADNS3080_MIN_FR ){ //Frame_period_min must be greater than 0x7E0E:
		frame_period_min = ADNS3080_MIN_FR;
	}
	//maxes must be higher than minimums:
	if( frame_period_max < frame_period_min ){
		frame_period_min = frame_period_max;
	}
	//Shutter can’t be slower than frame rate: Frame_Period_Max_Bound ≥ Frame_Period_Min_Bound + Shutter_Max_Bound
	if( frame_period_max < frame_period_min + shutter_max ){
		frame_period_max = frame_period_min + shutter_max;
	}
	//Check if busy
	if( getBusy() ){
		return 1;
	}
	//Set frame_rate & shutter to manual / auto
	setManualFrameRate( manual_fp );
	setManualShutter( manual_shutter );

	//Write to shutter_max
	setShutterMaxBound( shutter_max );
	
	//Write to frame_period_min
	setFramePeriodMinBound( frame_period_min );
	//Write to frame_period_max LAST
	setFramePeriodMinBound( frame_period_max );
	
   return true;
}




//----------------- Tests ----------------------------

//Get the product id.  Should be 0x17
template<TEMPLATE_TYPE>
uint8_t ADNS3080<TEMPLATE_INPUTS>
::getProductID() {
  return readRegister(ADNS3080_PRODUCT_ID);
}

//Get the revision id.
template<TEMPLATE_TYPE>
uint8_t ADNS3080<TEMPLATE_INPUTS>
::getRevisionID() {
  return readRegister(ADNS3080_REVISION_ID);
}

//Get the inverse product id.  Should be E8
template<TEMPLATE_TYPE>
uint8_t ADNS3080<TEMPLATE_INPUTS>
::getInverseProductID() {
  return readRegister(ADNS3080_INVERSE_PRODUCT_ID);
}

//Check that the inverse product id really is the inverse of the product id
template<TEMPLATE_TYPE>
bool ADNS3080<TEMPLATE_INPUTS>
::testProductID() {
  uint8_t manually_inverted_prod_id = ~getProductID();
  uint8_t chip_inverse_prod_id = getInverseProductID();
  return ( chip_inverse_prod_id == manually_inverted_prod_id );
}

//----------------- Major outputs --------------------

// Get motion burst data
template<TEMPLATE_TYPE>
void ADNS3080<TEMPLATE_INPUTS>
::motionBurst( uint8_t *motion, uint8_t *over_flow, int8_t *dx, int8_t *dy, uint8_t *squal, uint16_t *shutter, uint8_t *max_pix ) {
  
  // Enable communication
  digitalWrite( PIN_NCS, LOW );
  SPI.transfer( ADNS3080_MOTION_BURST );
  delayMicroseconds( ADNS3080_T_SRAD_MOT );

  // Receieve data:
  uint8_t motion_byte = SPI.transfer(0x00);
  *motion     = (motion_byte & B10000000) >> 7;
  *over_flow  = (motion_byte & B00010000) >> 4;
  //
  *dx      = SPI.transfer(0x00);
  *dy      = SPI.transfer(0x00);
  *squal   = SPI.transfer(0x00);
  *shutter = SPI.transfer16(0x00);
  *max_pix = SPI.transfer(0x00);

  // Disable communication
  digitalWrite( PIN_NCS, HIGH ); 

  // Zero displacement if motion did not occur
  if( *motion == false ) {
    *dx = 0;
    *dy = 0;
  }
}

// Get displacement data from motion burst
template<TEMPLATE_TYPE>
void ADNS3080<TEMPLATE_INPUTS>
::displacement( int8_t *dx, int8_t *dy ) {
  
  // Enable communication
  digitalWrite( PIN_NCS, LOW );
  SPI.transfer( ADNS3080_MOTION_BURST );
  delayMicroseconds( ADNS3080_T_SRAD_MOT );

  // Check if motion occured
  uint8_t motion  = (SPI.transfer(0x00) & B10000000) >> 7;
 
  if( motion == true ) {
    *dx = SPI.transfer(0x00);
    *dy = SPI.transfer(0x00);
  } else {
    *dx = 0;
    *dy = 0;
  }

  // Disable communication
  digitalWrite( PIN_NCS, HIGH ); 
}  

// Retreive pixels of next frame:
template<TEMPLATE_TYPE>
void ADNS3080<TEMPLATE_INPUTS>
::frameCapture( uint8_t output[ADNS3080_PIXELS_X][ADNS3080_PIXELS_Y] ) {  
  
  // Store pixel values
  writeRegister( ADNS3080_FRAME_CAPTURE, 0x83 );
  
  // Enable communication
    digitalWrite( PIN_NCS, LOW );
  SPI.transfer( ADNS3080_PIXEL_BURST );
  delayMicroseconds( ADNS3080_T_SRAD );

  //-- First pixel:
  uint8_t pixel = 0;

  // Recieve pixels until first is found 
  while( (pixel & B01000000) == 0 ) {
      pixel = SPI.transfer(0x00);
      delayMicroseconds( ADNS3080_T_LOAD );  
  }
  
  //-- Scan first frame:
  for( int y = 0; y < ADNS3080_PIXELS_Y; y += 1 ) {
    for( int x = 0; x < ADNS3080_PIXELS_X; x += 1 ) {  
      
      // Store and scale past pixel
      output[x][y] = pixel << 2; 
      
      // Receive new pixel
      pixel = SPI.transfer(0x00);
      delayMicroseconds( ADNS3080_T_LOAD );  
    }
  }
  // Disable communication
  digitalWrite( PIN_NCS, HIGH ); 
  delayMicroseconds( ADNS3080_T_LOAD + ADNS3080_T_BEXIT );
}

//----------------- Utils --------------------
//Get bit n from a byte
template<TEMPLATE_TYPE>
bool ADNS3080<TEMPLATE_INPUTS>
::getBit( uint8_t number, uint8_t n ) {
  bool bit;
  
  bit = (number >> n) & 1U;
  
  return bit;
}

//Set bin n in a byte
template<TEMPLATE_TYPE>
uint8_t ADNS3080<TEMPLATE_INPUTS>
::setBit( uint8_t number, uint8_t n, bool set_to ) {
  number ^= ( -set_to ^ number ) & (1U << n );
  
  return number;
}

//Concatenate two bytes into a 16 bit number
template<TEMPLATE_TYPE>
uint16_t ADNS3080<TEMPLATE_INPUTS>
::concatenateBytes( uint8_t upper, uint8_t lower ) {
  uint16_t output = upper;
  output = output << 8;
  output = output | lower; 
  return output;
}

//Get lower 8 bits from 16 bit number
template<TEMPLATE_TYPE>
uint8_t ADNS3080<TEMPLATE_INPUTS>
::lowerByte( uint16_t input ) {
  uint8_t output = input & 0xFF;
  return output;
}

//Get upper 8 bits from 16 bit number
template<TEMPLATE_TYPE>
uint8_t ADNS3080<TEMPLATE_INPUTS>
::upperByte( uint16_t input ) {
  uint8_t output = input >> 8;
  return output;
}


    
    