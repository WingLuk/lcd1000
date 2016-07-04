/*
 * ldc1000.cpp
 *
 * Author: Petel__
 * Copyright (c) 2014-2016 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <cstring>
#include "libsc/system.h"
#include "libsc/device_h/ldc1000.h"
#include "libsc/ldc1000.h"
#include "libsc/config.h"

using namespace std;
using namespace libsc;
using namespace LIBBASE_NS;

#define RP_MAX					0x13
#define RP_MIN					0x34
#define SENSOR_FREQ				0x9A

namespace libsc
{

#ifdef LIBSC_USE_LDC1000

inline Pin::Name GetSdatPin(const uint8_t id)
{
	switch (id)
	{
	default:
		assert(false);
		// no break

	case 0:
		return LIBSC_LDC10001_SDAT;

	case 1:
		return LIBSC_LDC10002_SDAT;

//	case 2:
//		return LIBSC_LINEAR_CCD2_AD;
	}
}

inline Pin::Name GetSclkPin(const uint8_t id)
{
	switch (id)
	{
	default:
		assert(false);
		// no break

	case 0:
		return LIBSC_LDC10001_SCLK;

	case 1:
		return LIBSC_LDC10002_SCLK;

//	case 2:
//		return LIBSC_LINEAR_CCD2_CLK;
	}
}

inline Pin::Name GetDcPin(const uint8_t id)
{
	switch (id)
	{
	default:
		assert(false);
		// no break

	case 0:
		return LIBSC_LDC10001_DC;

	case 1:
		return LIBSC_LDC10002_DC;

//	case 2:
//		return LIBSC_LINEAR_CCD2_SI;
	}
}

inline Pin::Name GetCsPin(const uint8_t id)
{
	switch (id)
	{
	default:
		assert(false);
		// no break

	case 0:
		return LIBSC_LDC10001_CS;

	case 1:
		return LIBSC_LDC10002_CS;

//	case 2:
//		return LIBSC_LINEAR_CCD2_SI;
	}
}

Gpo::Config GetSdatConfig(const uint8_t id, const bool is_high)
{
	Gpo::Config config;
	config.pin = GetSdatPin(id);
	config.is_high = is_high;
	return config;
}

Gpo::Config GetSclkConfig(const uint8_t id, const bool is_high)
{
	Gpo::Config config;
	config.pin = GetSclkPin(id);
	config.is_high = is_high;
	return config;
}


Gpo::Config GetCsConfig(const uint8_t id, const bool is_high)
{
	Gpo::Config config;
	config.pin = GetCsPin(id);
	config.is_high = is_high;
	return config;
}


Gpo::Config GetDcConfig(const uint8_t id, const bool is_high)
{
	Gpo::Config config;
	config.pin = GetDcPin(id);
	config.is_high = is_high;
	return config;
}

Ldc1000::Ldc1000(const uint8_t id)
:
	m_buf{ 0 },
	m_mosi(GetDcConfig(id, true)),
	m_cs(GetCsConfig(id, true)),
	m_sck(GetSclkConfig(id, false)),
	m_miso(GetSdatConfig(id, false)),
	m_proxData(0),
	m_freq(0)





{
	WriteData(REG_RP_MAX, RP_MAX);
	WriteData(REG_RP_MIN, RP_MIN);
	WriteData(REG_SENSOR_FREQ, SENSOR_FREQ);
	WriteData(REG_LDC_CONFIG, 0x13);
	WriteData(REG_CLK_CONFIG, 0x00);
	WriteData(REG_INT_CONFIG, 0x02);
	WriteData(REG_PWR_CONFIG, 0x01);
	WriteData(REG_THRES_HI_LSB, 0x50);
	WriteData(REG_THRES_HI_MSB, 0x14);
	WriteData(REG_THRES_LO_LSB, 0xC0);
	WriteData(REG_THRES_LO_MSB, 0x12);
	ReadBuffer();

	assert(m_buf[0] == 0x80);
}

void Ldc1000::Update(void)
{
	m_proxData = (ReadData(REG_PROX_MSB) << 8) | ReadData(REG_PROX_LSB);
	m_freq = (ReadData(REG_FREQ_MSB) << 16) | (ReadData(REG_FREQ_MID) << 8) | ReadData(REG_FREQ_LSB);
}

uint16_t Ldc1000::GetData(void)
{
	return m_proxData;
}

uint32_t Ldc1000::GetFreq(void)
{
	return m_freq;
}

uint8_t	Ldc1000::DataRW(uint8_t rwData, bool isCmd)
{
	uint8_t ret = 0;

	for(uint8_t i = 0; i < 8; i++)
	{
		m_mosi.Set(rwData & 0x80);

		rwData <<= 1;
		ret <<= 1;

		m_sck.Turn();

		if(m_miso.Get())
			ret |= 1;

		m_sck.Turn();
	}

	return(ret);
}

uint8_t Ldc1000::ReadData(const uint8_t reg)
{
	uint8_t ret = 0;
	m_cs.Set(false);
	System::DelayUs(2);
	DataRW(reg | 0x80);

	asm("nop");

	ret = DataRW(0, true);
	m_cs.Set(true);
	return ret;
}

uint8_t Ldc1000::WriteData(const uint8_t reg, const uint8_t data)
{
	uint8_t ret = 0;
	m_cs.Set(false);
	System::DelayUs(2);
	DataRW(reg & ~0x80);

	asm("nop");

	ret = DataRW(data, true);
	m_cs.Set(true);
	return ret;
}

void Ldc1000::ReadBuffer(void)
{
	m_cs.Set(false);
	DataRW(0x80);

	for (uint8_t i = 0; i < 12; i++)
		m_buf[i] = DataRW(0);
	m_cs.Set(true);
}

#else

Ldc1000::Ldc1000(const uint8_t id)
:
	m_buf{ 0 },
	m_mosi(nullptr),
	m_cs(nullptr),
	m_sck(nullptr),
	m_miso(nullptr),
	m_proxData(0),
	m_freq(0)
{}

void Ldc1000::Update(void)
{}

uint16_t Ldc1000::GetData(void)
{
	return 0;
}

uint32_t Ldc1000::GetFreq(void)
{
	return 0;
}

#endif

}

