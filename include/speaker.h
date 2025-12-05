#include <stdint.h>

// TODO: tune thresholds values so that they reflect what we want
#define TEMP_C_THRESHOLD_HIGH      77.0f // degrees farenheit
#define PRESSURE_KPA_THRESHOLD_LOW 80.0f //signals potential precipitation
#define HUMIDITY_RH_THRESHOLD_HIGH 90.0f //level that indicates humidity is high

#define PWM_PIN_A 10
#define PWM_PIN_B 11

typedef enum { AUDIO_IDLE=0, AUDIO_TEMP=1, AUDIO_PRESSURE=2, AUDIO_HUMID=3 } audio_mode_t;

void speakerInit(void);
void speakerLoop(void);

