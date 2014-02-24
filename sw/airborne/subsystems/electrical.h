#ifndef SUBSYSTEMS_ELECTRICAL_H
#define SUBSYSTEMS_ELECTRICAL_H

#include "std.h"

#ifndef LOW_BAT_LEVEL
#ifdef BATTERY_CELL

  #define CELL_MAXIMUM      4.2
  #define CELL_NOMINAL      3.7
  #define CELL_LOW          3.4
  #define CELL_CRITIC       3.2
  #define CELL_CATASTROPHIC 3.0

  #define MAX_BAT_LEVEL           BATTERY_CELL*CELL_MAXIMUM
  #define LOW_BAT_LEVEL           BATTERY_CELL*CELL_LOW
  #define CRITIC_BAT_LEVEL        BATTERY_CELL*CELL_CRITIC
  #define CATASTROPHIC_BAT_LEVEL  BATTERY_CELL*CELL_CATASTROPHIC
#else
  #ifndef CATASTROPHIC_BAT_LEVEL
    #error "Add this line to your airframe <define name="BATTERY_CELL" value="X"/> where X is the number of cell of a LiPo"
  #endif
#endif

#ifdef JOHN_AP_2012_V1
 #define ADC_COEFF_M         0.0354067865
 #define ADC_COEFF_B         0.0376761571
 #define VoltageOfAdc(adc)   (ADC_COEFF_M*adc + ADC_COEFF_B)
#endif

#ifdef JOHN_AP_2012_V2
 #define ADC_COEFF_M         0.0323745054
 #define ADC_COEFF_B         0.2191709665
 #define VoltageOfAdc(adc)   (ADC_COEFF_M*adc + ADC_COEFF_B)
#endif
#endif

struct Electrical {

  uint16_t vsupply; /* supply in decivolts */
  uint16_t adc_battery;
  int32_t current; /* current in milliamps */
  int32_t consumed; /* consumption in mAh */

};

extern struct Electrical electrical;

extern void electrical_init(void);
extern void electrical_periodic(void);

#endif /* SUBSYSTEMS_ELECTRICAL_H */
