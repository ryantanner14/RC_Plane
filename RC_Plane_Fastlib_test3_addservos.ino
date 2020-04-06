#include <RCArduinoFastLib.h>
 // MultiChannels
//
// rcarduino.blogspot.com
//
// A simple approach for reading three RC Channels using pin change interrupts
//
// See related posts -
// http://rcarduino.blogspot.co.uk/2012/01/how-to-read-rc-receiver-with.html
// http://rcarduino.blogspot.co.uk/2012/03/need-more-interrupts-to-read-more.html
// http://rcarduino.blogspot.co.uk/2012/01/can-i-control-more-than-x-servos-with.html
//
// rcarduino.blogspot.com
//
// include the pinchangeint library - see the links in the related topics section above for details
#include <PinChangeInt.h>
// Assign your channel in pins
#define THROTTLE_IN_PIN A2
#define ELEVATOR_IN_PIN A4
#define RUDDER_IN_PIN 6
#define R_AILERON_IN_PIN 7
#define L_AILERON_IN_PIN A3
// Assign your channel out pins
#define THROTTLE_OUT_PIN 5
#define ELEVATOR_OUT_PIN 9
#define RUDDER_OUT_PIN 10
#define R_AILERON_OUT_PIN 4
#define L_AILERON_OUT_PIN 11
// Assign servo indexes
#define SERVO_THROTTLE 0
#define SERVO_ELEVATOR 1
#define SERVO_RUDDER 2
#define SERVO_R_AILERON 3 //may need to swap this and throttle since this is channel 1
#define SERVO_FRAME_SPACE 5 //changed from 3 to 5 with adding ailerons
// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define THROTTLE_FLAG 1
#define ELEVATOR_FLAG 2
#define RUDDER_FLAG 4
#define R_AILERON_FLAG 5 //arbitrarily picked 5
// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;
// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile uint16_t unThrottleInShared;
volatile uint16_t unElevatorInShared;
volatile uint16_t unRudderInShared;
volatile uint16_t unR_AileronInShared;
// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint16_t unThrottleInStart;
uint16_t unElevatorInStart;
uint16_t unRudderInStart;
uint16_t unR_AileronInStart;
uint16_t unLastAuxIn = 0;
uint32_t ulVariance = 0;
uint32_t ulGetNextSampleMillis = 0;
uint16_t unMaxDifference = 0;
void setup()
{
 Serial.begin(115200);
 Serial.println("multiChannels");
 // attach servo objects, these will generate the correct
 // pulses for driving Electronic speed controllers, servos or other devices
 // designed to interface directly with RC Receivers
 CRCArduinoFastServos::attach(SERVO_THROTTLE,THROTTLE_OUT_PIN);
 CRCArduinoFastServos::attach(SERVO_ELEVATOR,ELEVATOR_OUT_PIN);
 CRCArduinoFastServos::attach(SERVO_RUDDER,RUDDER_OUT_PIN);
 CRCArduinoFastServos::attach(SERVO_R_AILERON,R_AILERON_OUT_PIN);
 
 // lets set a standard rate of 50 Hz by setting a frame space of 10 * 2000 = 3 Servos + 7 times 2000
 CRCArduinoFastServos::setFrameSpaceA(SERVO_FRAME_SPACE,7*2000);
 CRCArduinoFastServos::begin();
 
 // using the PinChangeInt library, attach the interrupts
 // used to read the channels
 PCintPort::attachInterrupt(THROTTLE_IN_PIN, calcThrottle,CHANGE);
 PCintPort::attachInterrupt(ELEVATOR_IN_PIN, calcElevator,CHANGE);
 PCintPort::attachInterrupt(RUDDER_IN_PIN, calcRudder,CHANGE);
 PCintPort::attachInterrupt(R_AILERON_IN_PIN, calcR_Aileron,CHANGE);
}
void loop()
{
 // create local variables to hold a local copies of the channel inputs
 // these are declared static so that thier values will be retained
 // between calls to loop.
 static uint16_t unThrottleIn;
 static uint16_t unElevatorIn;
 static uint16_t unRudderIn;
 static uint16_t unR_AileronIn;
 // local copy of update flags
 static uint8_t bUpdateFlags;
 // check shared update flags to see if any channels have a new signal
 if(bUpdateFlagsShared)
 {
   noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables
   // take a local copy of which channels were updated in case we need to use this in the rest of loop
   bUpdateFlags = bUpdateFlagsShared;
  
   // in the current code, the shared values are always populated
   // so we could copy them without testing the flags
   // however in the future this could change, so lets
   // only copy when the flags tell us we can.
  
   if(bUpdateFlags & THROTTLE_FLAG)
   {
     unThrottleIn = unThrottleInShared;
   }
  
   if(bUpdateFlags & ELEVATOR_FLAG)
   {
     unElevatorIn = unElevatorInShared;
   }
  
   if(bUpdateFlags & RUDDER_FLAG)
   {
     unRudderIn = unRudderInShared;
   }
   
   if(bUpdateFlags & R_AILERON_FLAG)
   {
     unR_AileronIn = unR_AileronInShared;
   }
   // clear shared copy of updated flags as we have already taken the updates
   // we still have a local copy if we need to use it in bUpdateFlags
   bUpdateFlagsShared = 0;
  
   interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
   // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
   // service routines own these and could update them at any time. During the update, the
   // shared copies may contain junk. Luckily we have our local copies to work with :-)
 }
 // do any processing from here onwards
 // only use the local values unRudderIn, unThrottleIn and unSteeringIn, the shared
 // variables unRudderInShared, unThrottleInShared, unSteeringInShared are always owned by
 // the interrupt routines and should not be used in loop
 // the following code provides simple pass through
 // this is a good initial test, the Arduino will pass through
 // receiver input as if the Arduino is not there.
 // This should be used to confirm the circuit and power
 // before attempting any custom processing in a project.
 // we are checking to see if the channel value has changed, this is indicated
 // by the flags. For the simple pass through we don't really need this check,
 // but for a more complex project where a new signal requires significant processing
 // this allows us to only calculate new values when we have new inputs, rather than
 // on every cycle.
 if(bUpdateFlags & THROTTLE_FLAG)
 {
   CRCArduinoFastServos::writeMicroseconds(SERVO_THROTTLE,unThrottleIn);
 }
 if(bUpdateFlags & ELEVATOR_FLAG)
 {
  //Serial.println(unElevatorIn);
   CRCArduinoFastServos::writeMicroseconds(SERVO_ELEVATOR,unElevatorIn);
 }
 if(bUpdateFlags & RUDDER_FLAG)
 {
  CRCArduinoFastServos::writeMicroseconds(SERVO_RUDDER,unRudderIn);
  }
 if(bUpdateFlags & R_AILERON_FLAG)
 {
    Serial.println(unR_AileronIn);
  CRCArduinoFastServos::writeMicroseconds(SERVO_R_AILERON,unR_AileronIn);
  }
 bUpdateFlags = 0;
}
// simple interrupt service routine
void calcThrottle()
{
 if(PCintPort::pinState)
 {
   unThrottleInStart = TCNT1;
 }
 else
 {
   unThrottleInShared = (TCNT1 - unThrottleInStart)>>1;
   bUpdateFlagsShared |= THROTTLE_FLAG;
 }
}
void calcElevator()
{
 if(PCintPort::pinState)
 {
   unElevatorInStart = TCNT1;
 }
 else
 {
   unElevatorInShared = (TCNT1 - unElevatorInStart)>>1;
   bUpdateFlagsShared |= ELEVATOR_FLAG;
 }
}
void calcRudder()
{
 if(PCintPort::pinState)
 {
   unRudderInStart = TCNT1;
 }
 else
 {
   unRudderInShared = (TCNT1 - unRudderInStart)>>1;
   bUpdateFlagsShared |= RUDDER_FLAG; }
}
void calcR_Aileron()
{
 if(PCintPort::pinState)
 {
   unR_AileronInStart = TCNT1;
 }
 else
 {
   unR_AileronInShared = (TCNT1 - unR_AileronInStart)>>1;
   bUpdateFlagsShared |= R_AILERON_FLAG; }
}



void landPlane()
{
 // only use the local values unRudderIn, unThrottleIn and unSteeringIn, the shared
 // variables unRudderInShared, unThrottleInShared, unSteeringInShared are always owned by
 // the interrupt routines and should not be used in loop

 /*
  * 
  */


}
