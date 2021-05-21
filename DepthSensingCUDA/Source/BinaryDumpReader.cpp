
#include "stdafx.h"

#include "BinaryDumpReader.h"
#include "GlobalAppState.h"
#include "MatrixConversion.h"

#include "sensorData/stb_image.h"


#ifdef BINARY_DUMP_READER

#include <algorithm>
#include <iostream>
#include <fstream>
#include <list>
#include <iomanip>
#include <vector>
#include <string>

#include <conio.h>
#include <filesystem>

BinaryDumpReader::BinaryDumpReader()
{
	m_NumFrames = 0;
	m_CurrFrame = 0;
	m_bHasColorData = false;
	//parameters are read from the calibration file
}

BinaryDumpReader::~BinaryDumpReader()
{
	releaseData();
}


HRESULT BinaryDumpReader::createFirstConnected()
{
	releaseData();

	if (GlobalAppState::get().s_binaryDumpSensorFile.size() == 0) throw MLIB_EXCEPTION("need to specific s_binaryDumpSensorFile[0]");
	std::string dirname = GlobalAppState::get().s_binaryDumpSensorFile[0];

	std::cout << "Start loading directory" << " " << dirname << std::endl;

	// Load a dummy file
	int width, height, channels;
	unsigned short* depth = stbi_load_16((dirname + "/depth/000001.png").c_str(), &width, &height, &channels, 0);
	stbi_image_free(depth);
	std::cout << width << " " << height << " " << channels << "\n";

	m_data.m_DepthImageHeight = height;
	m_data.m_DepthImageWidth = width;
	m_data.m_ColorImageHeight = height;
	m_data.m_ColorImageWidth = width;
	m_data.m_DepthNumFrames = 3000;
	m_data.m_ColorNumFrames = 0;

	std::vector<float> intrinsics_data = { 525, 0, 319.5, 0,
										   0, 525, 219.5, 0,
										   0, 0, 1, 0,
										   0, 0, 0, 1 };
	std::vector<float> extrinsics_data = { 1, 0, 0, 0, 
	                                       0, 1, 0, 0,
	                                       0, 0, 1, 0,
	                                       0, 0, 0, 1};
	mat4f intrinsics(intrinsics_data.data());
	mat4f extrinsics(extrinsics_data.data());

	m_data.m_CalibrationColor.setMatrices(intrinsics, extrinsics);
	m_data.m_CalibrationDepth.setMatrices(intrinsics, extrinsics);

	std::cout << "intrinsics:" << std::endl;
	std::cout << m_data.m_CalibrationDepth.m_Intrinsic << std::endl;

	m_data.m_trajectory.clear();
	std::stringstream ss;
	for (int i = 0; i < m_data.m_DepthNumFrames; ++i) {
		ss.str("");
		ss << dirname << "/depth/" << std::setfill('0') << std::setw(6) << (i + 1) << ".png";
		depth_filenames_.push_back(ss.str());
		//m_data.m_trajectory.push_back(extrinsics);
		//ss.str("");
		//ss << dirname << "/color/" << std::setfill('0') << std::setw(6) << (i + 1) << ".png";
		//color_filenames_.push_back(ss.str());
	}

	std::ifstream in(dirname + "/trajectory.log");
	int i, j, k;
	std::vector<float> pose_data;
	pose_data.resize(16);
	if (in.is_open()) {
		while (in >> i >> j >> k) {
			for (int ii = 0; ii < 16; ++ii) {
				in >> pose_data[ii];
			}
			mat4f pose(pose_data.data());
			m_data.m_trajectory.push_back(pose);
		}
	}
	std::cout << "trajectory size = " << m_data.m_trajectory.size() << "\n";


	RGBDSensor::init(m_data.m_DepthImageWidth, m_data.m_DepthImageHeight, std::max(m_data.m_ColorImageWidth,1u), std::max(m_data.m_ColorImageHeight,1u), 1);
	initializeDepthIntrinsics(m_data.m_CalibrationDepth.m_Intrinsic(0,0), m_data.m_CalibrationDepth.m_Intrinsic(1,1), m_data.m_CalibrationDepth.m_Intrinsic(0,2), m_data.m_CalibrationDepth.m_Intrinsic(1,2));
	initializeColorIntrinsics(m_data.m_CalibrationColor.m_Intrinsic(0,0), m_data.m_CalibrationColor.m_Intrinsic(1,1), m_data.m_CalibrationColor.m_Intrinsic(0,2), m_data.m_CalibrationColor.m_Intrinsic(1,2));

	initializeDepthExtrinsics(m_data.m_CalibrationDepth.m_Extrinsic);
	initializeColorExtrinsics(m_data.m_CalibrationColor.m_Extrinsic);

	m_NumFrames = m_data.m_DepthNumFrames;
	assert(m_data.m_ColorNumFrames == m_data.m_DepthNumFrames || m_data.m_ColorNumFrames == 0);		
		
	if (m_data.m_ColorImages.size() > 0) {
		m_bHasColorData = true;
	} else {
		m_bHasColorData = false;
	}
	std::cout << "Loading finished" << std::endl;
	system("pause");
	return S_OK;
}

HRESULT BinaryDumpReader::processDepth()
{
	if(m_CurrFrame >= m_NumFrames)
	{
		GlobalAppState::get().s_playData = false;
		std::cout << "binary dump sequence complete - press space to run again" << std::endl;
		m_CurrFrame = 0;
	}

	if(GlobalAppState::get().s_playData) {
		int width, height, channel;
		unsigned short* depths = stbi_load_16(depth_filenames_[m_CurrFrame].c_str(), &width, &height, &channel, 0);

		float* depth = getDepthFloat();
		for (int i = 0; i < width * height; ++i) {
			depth[i] = static_cast<float>(depths[i]) / 1000.0;
		}

		stbi_image_free(depths);

		incrementRingbufIdx();

		m_CurrFrame++;
		return S_OK;
	} else {
		return S_FALSE;
	}
}

void BinaryDumpReader::releaseData()
{
	m_CurrFrame = 0;
	m_bHasColorData = false;
	m_data.deleteData();
}



#endif
