/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * cam_helper_mira016.cpp - camera helper for mira016 sensor
 */

#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <cmath>

#include <libcamera/base/log.h>

/*
 * We have observed that the mira016 embedded data stream randomly returns junk
 * register values. Do not rely on embedded data until this has been resolved.
 */
#define ENABLE_EMBEDDED_DATA 0

#include "cam_helper.h"
#if ENABLE_EMBEDDED_DATA
#include "md_parser.h"
#endif

using namespace RPiController;
using namespace libcamera;
using libcamera::utils::Duration;
using namespace std::literals::chrono_literals;

namespace libcamera {
LOG_DECLARE_CATEGORY(IPARPI)
}

/*
 * We care about one gain register and a pair of exposure registers. Their I2C
 * addresses from the mira016 datasheet:
 */
// constexpr uint32_t gainReg = 0x0024;
// constexpr uint32_t expReg = 0x000E;
// constexpr uint32_t frameLengthHiReg = 0x1013;
// constexpr uint32_t frameLengthLoReg = 0x1012;
// constexpr std::initializer_list<uint32_t> registerList [[maybe_unused]]
// 	= { expReg, gainReg };

class CamHelperMira016 : public CamHelper
{
public:
	CamHelperMira016();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t code) const override;
	// unsigned int mistrustFramesModeSwitch() const override;
	// bool sensorEmbeddedDataPresent() const override;
	// void getDelays(int &exposureDelay, int &gainDelay,
	// 			   int &vblankDelay, int &hblankDelay) const override;


private:
	static constexpr uint32_t minExposureLines = 1;
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines.
	 */
	static constexpr int frameIntegrationDiff = 4;
	/* ROW_LENGTH is microseconds is (ROW_LENGTH * 8 / MIRA_DATA_RATE) */
	// #define MIRA016_DATA_RATE			1000 // Mbit/s
	// #define MIRA016_MIN_ROW_LENGTH			1842

	// void populateMetadata(const MdParser::RegisterMap &registers,
	// 		      Metadata &metadata) const override;
};

CamHelperMira016::CamHelperMira016()
#if ENABLE_EMBEDDED_DATA
	: CamHelper(std::make_unique<MdParserSmia>(registerList), frameIntegrationDiff)
#else
	: CamHelper({}, frameIntegrationDiff)
#endif
{
}

/*
 * The driver's analogue gain code is the gain in thousandths (1000 is 1x), up
 * to 16000 in 8-bit modes, 4000 in 10-bit modes and only 1000 in 12-bit modes.
 * The driver rounds it to the nearest gain the fine gain registers give:
 *
 *   gain = 57 * 2^n / (ADC_GAIN + 21)
 *
 * n is the octave of the gain (0 to 4) and ADC_GAIN is 21 to 63. Up to 4x the
 * 8 and 10-bit modes give the same gains, so the conversion does not depend on
 * the mode, which matters because the IPA converts the gain limits of a new
 * mode before it passes that mode to the helper.
 */
uint32_t CamHelperMira016::gainCode(double gain) const
{
	return std::max<long>(std::lround(gain * 1000), 1000);
}

double CamHelperMira016::gain(uint32_t gainCode) const
{
	/* The gain the driver programs for this code. */
	uint32_t code = std::clamp<uint32_t>(gainCode, 1000, 16000);
	uint32_t n = 0;
	while (n < 4 && (57000u << (n + 1)) <= 84 * code)
		n++;
	uint32_t num = 57000u << n;
	int adcGain = static_cast<int>((num + code / 2) / code) - 21;
	adcGain = std::clamp(adcGain, 0, 63);
	return static_cast<double>(num) / (adcGain + 21) / 1000.0;
}




// unsigned int CamHelperMira016::mistrustFramesModeSwitch() const
// {
// 	/*
// 	 * For reasons unknown, we do occasionally get a bogus metadata frame
// 	 * at a mode switch (though not at start-up). Possibly warrants some
// 	 * investigation, though not a big deal.
// 	 */
// 	return 1;
// }

// bool CamHelperMira016::sensorEmbeddedDataPresent() const
// {
// 	return ENABLE_EMBEDDED_DATA;
// }

// void CamHelperMira016::populateMetadata(const MdParser::RegisterMap &registers,
// 				       Metadata &metadata) const
// {
// 	DeviceStatus deviceStatus;

// 	deviceStatus.shutterSpeed = exposure(registers.at(expReg), deviceStatus.lineLength);
// 	deviceStatus.analogueGain = gain(registers.at(gainReg));

// 	metadata.set("device.status", deviceStatus);
// }

// void CamHelperMira016::getDelays(int &exposureDelay, int &gainDelay,
// 								 int &vblankDelay, int &hblankDelay) const
// {
// 	/* The driver appears to behave as follows: */
// 	exposureDelay = 1;
// 	gainDelay = 1;
// 	vblankDelay = 1;
// 	hblankDelay = 1;
// }


static CamHelper *create()
{
	return new CamHelperMira016();
}

static RegisterCamHelper reg("mira016", &create);

