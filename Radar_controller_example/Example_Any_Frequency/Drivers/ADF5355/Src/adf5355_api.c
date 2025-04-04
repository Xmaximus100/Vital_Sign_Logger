/*
 * adf5355_handler.c
 *
 *  Created on: Oct 15, 2024
 *      Author: Celelele
 */


#include <adf5355_api.h>
#include "adf5355.h"
#include "spi.h"


/******************************************************************************/
/***************************** Private Variables ******************************/
/******************************************************************************/

struct adf5355_init_param hadf5355;
struct adf5355_dev* dev;
static uint16_t freq_out = 12000;
static uint8_t freq_in = 10;
static bool outb_en = true;
static int8_t out_power = 3;
static uint32_t cp_ua = 1000;
static bool synced = false;
static enum adf5355_mux_out_sel mux_out = ADF5355_MUXOUT_DIGITAL_LOCK_DETECT;//ADF5355_MUXOUT_DVDD;//ADF5355_MUXOUT_GND;//

/******************************************************************************/
/************************** Functions Implementation **************************/
/******************************************************************************/

void* ADF5355_SetFrequencyOut(void* new_freq){
	static bool ret;
	uint32_t* value = (uint32_t*)new_freq;
	if (*value > 15000 || *value < 100) ret = false;
	else {
		freq_out = *value;
		ret = true;
	}
	return &ret;
}

void* ADF5355_SetFrequencyIn(void* new_freq){
	static bool ret;
	uint32_t* value = (uint32_t*)new_freq;
	if (*value > 200 || *value < 5) ret = false;
	else {
		freq_in = *value;
		ret = true;
	}
	return &ret;
}

void* ADF5355_SetPower(void* new_pow){
	static bool ret;
	int8_t* value = (int8_t*)new_pow;
	if (*value > 3 || *value < 0) ret = false;
	else {
		out_power = *value;
		ret = true;
	}
	return &ret;
}

void* ADF5355_SetCurrent(void* new_curr){
	static bool ret;
	uint32_t* value = (uint32_t*)new_curr;
	if (*value > 5000 || *value < 310) ret = false;
	else {
		cp_ua = *value;
		ret = true;
	}
	return &ret;
}

void* ADF5355_SetMuxOut(void* new_mux_out){
	static bool ret;
	enum adf5355_mux_out_sel* value = (enum adf5355_mux_out_sel*)new_mux_out;
	if (*value > ADF5355_MUXOUT_DIGITAL_LOCK_DETECT || *value < ADF5355_MUXOUT_THREESTATE) ret = false;
	else {
		mux_out = *value;
		if (synced){
			adf5355_set_muxout(dev, mux_out);
		}
		ret = true;
	}
	return &ret;
}

void* ADF5355_Enable(void* state){
	static bool ret;
	uint8_t* value = (uint8_t*)state;
	if (*value != 0 && *value != 1) ret = false;
	else {
		outb_en = *value;
		if (synced){
			adf5355_set_power(dev, outb_en, out_power);
		}
		ret = true;
	}
	return &ret;
}

void ADF5355_Param_Init(void){
	hadf5355.spi_init = &hspi3; // Wskaźnik do struktury SPI init
	hadf5355.dev_id = 1; // Identyfikator urządzenia ADF5355
	hadf5355.freq_req = (uint64_t)freq_out*(uint64_t)1000000; // Żądana częstotliwość wyjściowa w Hz
	hadf5355.freq_req_chan = 1; // Kanał częstotliwości
	hadf5355.clkin_freq = freq_in*1000000; // Częstotliwość zegara wejściowego w Hz
	hadf5355.cp_ua = cp_ua; // Prąd pompy ładunkowej w mikroamperach
	hadf5355.cp_neg_bleed_en = false; // Flaga aktywacji negatywnego prądu wycieku
	hadf5355.cp_gated_bleed_en = true;  // Flaga aktywacji bramkowania prądu wycieku
	hadf5355.cp_bleed_current_polarity_en = false;  // Flaga aktywacji biegunowości prądu wycieku
	hadf5355.mute_till_lock_en = false; // Flaga aktywacji funkcji mute till lock
	hadf5355.outa_en = false;  // Flaga aktywacji wyjścia A
	hadf5355.outb_en = outb_en;  // Flaga aktywacji wyjścia B
	hadf5355.out_power = out_power;  // Moc wyjścia B
	hadf5355.phase_detector_polarity_neg = false;  // Flaga aktywacji negatywnej polaryzacji detektora fazy
	hadf5355.ref_diff_en = false;  // Flaga aktywacji różnicowego wejścia referencyjnego
	hadf5355.mux_out_3v3_en = true;  // Flaga aktywacji wyjścia mux na 3,3V
	hadf5355.ref_doubler_en = false;  // Flaga aktywacji podwajacza częstotliwości referencyjnej
	hadf5355.ref_div2_en = false;  // Flaga aktywacji podzielnika przez 2
	hadf5355.mux_out_sel = mux_out;  // Wybór wyjścia mux
	hadf5355.outb_sel_fund = false;  // Flaga wyboru częstotliwości podstawowej na wyjściu B
	synced = true;
}


void* ADF5355_Load(void* arg){
	ADF5355_Param_Init();
	static bool ret = false;
	int32_t response = adf5355_init(&dev, &hadf5355);
	if (response == 0) ret = true;
	return &ret;
}

void* ADF5355_Run(void* arg){
	static bool ret = false;
	int32_t response = adf5355_change_freq(dev, hadf5355.freq_req);
	if (response == 0) ret = true;
	return &ret;
}
