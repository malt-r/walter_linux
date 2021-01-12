#pragma once

//this will be the default values for every servo, if no configuration is loaded
//TODO: load a configuration from a csv-file
const unsigned int anNeutralPulses[3] =
{
	1400,	//HipJoint
	1455,	//UpperLegJoint
	1472	//LowerLegJoint
};
const unsigned int anMaxPulses[3] =
{
	1785,	//HipJoint
	2212,	//UpperLegJoint,
	2184	//LowerLegJoint
};
const unsigned int anMinPulses[3] =
{
	952,	//HipJoint
	1084,	//UpperLegJoint,
	921 	//LowerLegJoint
};

const double afNeutralAngles[3] =
{
    0,      //HipJoint
    22.5,     //UpperLegJoint
    85.4    //LowerLegJoint
};

const double servoConst_fAnglePerPulse = 0.12274; //in degree, this was measured with less than ideal methods and may be slightly inaccurate