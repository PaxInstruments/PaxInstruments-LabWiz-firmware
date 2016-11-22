// Source: https://wemakethings.net/2014/04/24/k-thermocouple-lib/
// Modified September 2016 - Andrew Gaylo
#include "stdint.h"
#include "thermocouples_LUT.h"

#define UVOLT_OFFSET    6458    // This is the offset for kelvin, to c, subtract this from uV to get 0C
#define TEMP_OFFSET     270    // This is the offset for kelvin, to c, subtract this from temp to get C

#define POINTS_COUNT    148

// These are temperatures from abs_zero (In deg C) and the corresponding microvolts
// Source: http://srdata.nist.gov/its90/download/type_k.tab
static thrm_lookup_t typeK_LUT_Kelvin[] = {
{0,0},
{10,17},
{20,54},
{30,114},
{40,196},
{50,300},
{60,423},
{70,567},
{80,728},
{90,908},
{100,1104},
{110,1317},
{120,1545},
{130,1789},
{140,2047},
{150,2320},
{160,2606},
{170,2904},
{180,3215},
{190,3538},
{200,3871},
{210,4215},
{220,4569},
{230,4931},
{240,5302},
{250,5680},
{260,6066},
{270,6458},
{280,6855},
{290,7256},
{300,7661},
{310,8070},
{320,8481},
{330,8894},
{340,9309},
{350,9725},
{360,10140},
{370,10554},
{380,10967},
{390,11378},
{400,11786},
{410,12193},
{420,12596},
{430,12998},
{440,13399},
{450,13798},
{460,14197},
{470,14596},
{480,14997},
{490,15398},
{500,15801},
{510,16205},
{520,16611},
{530,17019},
{540,17429},
{550,17840},
{560,18253},
{570,18667},
{580,19082},
{590,19498},
{600,19915},
{610,20332},
{620,20751},
{630,21171},
{640,21591},
{650,22012},
{660,22433},
{670,22855},
{680,23278},
{690,23701},
{700,24125},
{710,24549},
{720,24974},
{730,25399},
{740,25824},
{750,26250},
{760,26676},
{770,27102},
{780,27529},
{790,27955},
{800,28382},
{810,28808},
{820,29234},
{830,29661},
{840,30087},
{850,30513},
{860,30938},
{870,31363},
{880,31788},
{890,32213},
{900,32637},
{910,33060},
{920,33483},
{930,33905},
{940,34327},
{950,34747},
{960,35168},
{970,35587},
{980,36006},
{990,36423},
{1000,36840},
{1010,37256},
{1020,37671},
{1030,38086},
{1040,38499},
{1050,38911},
{1060,39323},
{1070,39733},
{1080,40143},
{1090,40551},
{1100,40959},
{1110,41366},
{1120,41771},
{1130,42176},
{1140,42579},
{1150,42982},
{1160,43383},
{1170,43784},
{1180,44183},
{1190,44582},
{1200,44980},
{1210,45376},
{1220,45772},
{1230,46166},
{1240,46559},
{1250,46952},
{1260,47343},
{1270,47734},
{1280,48123},
{1290,48511},
{1300,48898},
{1310,49284},
{1320,49669},
{1330,50053},
{1340,50436},
{1350,50817},
{1360,51198},
{1370,51577},
{1380,51955},
{1390,52331},
{1400,52707},
{1410,53081},
{1420,53453},
{1430,53825},
{1440,54195},
{1450,54563},
{1460,54931},
{1470,55296}
};

static inline int32_t interpolate(int32_t val, int32_t rangeStart, int32_t rangeEnd,
        int32_t valStart, int32_t valEnd)
{
    int32_t result;
    int32_t uvOver,uvDelta;
    int32_t interp;

    uvOver = (val - valStart);
    uvDelta = (valEnd - valStart);

    interp = (uvOver*1000) / uvDelta;

    result = ((rangeEnd - rangeStart) * interp)/1000;
    result += rangeStart;

    return result;
}

// Given a temperature in 1/10ths, find the microvolts associated with it
static inline int32_t interpolateVoltage(int32_t temp, int i)
{
    int32_t result;
    int32_t tOver,tDelta;
    int32_t interp;
    int32_t rangeStart,rangeEnd;

    rangeStart = typeK_LUT_Kelvin[i-1].temperature;
    rangeEnd = typeK_LUT_Kelvin[i].temperature;

    tOver = (temp- rangeStart);
    tDelta = (rangeEnd - rangeStart);

    interp = (tOver*100) / tDelta;

    result = ((typeK_LUT_Kelvin[i].microvolts - typeK_LUT_Kelvin[i-1].microvolts) * interp)/100;
    result += typeK_LUT_Kelvin[i-1].microvolts;

    return result;
}

static inline int32_t interpolateTemperature(int32_t microvolts, int i)
{
    int32_t result;
    int32_t uvOver,uvDelta;
    int32_t interp;

    uvOver = (microvolts- typeK_LUT_Kelvin[i-1].microvolts);
    uvDelta = (typeK_LUT_Kelvin[i].microvolts - typeK_LUT_Kelvin[i-1].microvolts);

    interp = (uvOver*1000) / uvDelta;

    result = ((typeK_LUT_Kelvin[i].temperature - typeK_LUT_Kelvin[i-1].temperature) * interp)/100;
    result += (typeK_LUT_Kelvin[i-1].temperature*10);

    return result;
}

/**
 * Returns the index of the first point whose temperature value is
greater than the argument
 **/
static inline int searchTemp(int32_t temp)
{
	int i;
	for(i = 0; i < POINTS_COUNT; i++)
	{
		if(typeK_LUT_Kelvin[i].temperature > temp)
		{
			return i;
		}
	}
	return POINTS_COUNT-1;
}

/**
 * Returns the index of the first point whose microvolts value is
greater than the argument
 **/
static inline int searchMicrovolts(int32_t microvolts)
{
	int i;
	for(i = 0; i < POINTS_COUNT; i++) {
		if(typeK_LUT_Kelvin[i].microvolts > microvolts)
		{
			return i;
		}
	}
	return POINTS_COUNT-1;
}

/**
 * Returns temperature as a function of the measured thermocouple voltage.
 *
 * int32_t microvoltsMeasured - Microvolts from 0v +/-
 *
 * Return: int32_t of the temperature in C in 1/10ths of a deg C
 *
  **/
int32_t thrmMicroVoltsToC(int32_t microvoltsMeasured)
{
    int32_t microvolts;
    int32_t result;

    // The microvolts are from 0, so add in the offset
    microvolts = microvoltsMeasured;

	//look up microvolts in The Table and interpolate
	result = interpolateTemperature(microvolts, searchMicrovolts(microvolts));

    result -= (TEMP_OFFSET*10);
	asm("nop");
	return result;
}


/**
 * Return the microvolts of a given temperature
 *
 * int32_t measured_temp - Temperature in 1/10ths of deg C
 *
 * Return: int32_t of the microvolts for the temperature from 0v +/-
 *
 **/
int32_t thrmCToMicroVolts(int32_t measured_temp)
{
    int32_t result;

    // The temp is in Celcius, so add in the offset to get to kelvin
    measured_temp += (TEMP_OFFSET*10);

    // This is the microvolts from abs zero
    // The measured_temp is in 1/10ths of volts, so div by 10 in the search
    result = interpolateVoltage(measured_temp/10, searchTemp(measured_temp/10));

    return result;
}
