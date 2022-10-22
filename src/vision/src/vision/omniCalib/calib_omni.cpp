/*
 *  _____ ____  ____   _____        __
 * | ____|  _ \/ ___| / _ \ \      / /
 * |  _| | |_) \___ \| | | \ \ /\ / / 
 * | |___|  _ < ___) | |_| |\ V  V /  
 * |_____|_| \_\____/ \___/  \_/\_/   
 * 
 * Team Name 	: EEPIS ROBO SOCCER ON WHEELED
 * Description	: Vision programs for calibration of ERSOW omnidirectional camera.
 * Author		: Muhamad Khoirul Anwar  - 3110141026 (ME14-ERSOW17)
 *  			  Satria Rachmad Santosa - 2210151021 (CE15-ERSOW17,18)
 *  			  Muhammad Abdul Haq     - 2210161005 (CE16-ERSOW17,18,19)
 *				  Fendiq Nur Wahyu       - 2210141043 (CE14-TA18)
 *  			  Erna Alfi Nurrohmah    - 2210181002 (CE18-ERSOW19)
 *  			  Fadl Lul Hakim Ihsan   - 2210181034 (CE18-ERSOW19)
 *  			  Ujang Supriyadi	     - 2210191007 (CE19-ERSOW20)
 * Start Date	: 01 December 2016
 * Last Edited	: 24 Oktober 2020
 * College		: Electronic Engineering Polytechnic Institute of Surabaya
 * Examples		: 
 *    			  >> ./calib_omni 
 */

#include "rosHeader.h"
#include "OmniCalib.h"

int main(int argc,char**argv)
{
	ros::init(argc,argv,"omniVisionCalib");
	OmniCalib calib_omni(argc, argv);
	while(ros::ok())
	{
		calib_omni.MainLoop();
	}
	cvDestroyAllWindows();
}