// Source: https://wemakethings.net/2014/04/24/k-thermocouple-lib/
// Modified September 2016 - Andrew Gaylo
#ifndef _THRM_LUT_H_
#define _THRM_LUT_H_

typedef struct {
    uint32_t temperature;
    uint32_t microvolts;
} thrm_lookup_t;

#define TEMP_OFFSET     2700     // This is the offset for kelvin, to c, subtract this from temp to get C

/**
 * Returns temperature in 1/10ths kelvin as a function of the ambient temperature and the measured
 * thermocouple voltage.
 **/
int32_t thrmMicroVoltsToC(uint32_t microvoltsMeasured, uint32_t ambient_uvolts);

uint32_t thrmCToMicroVolts(int32_t measured_temp);

#endif
