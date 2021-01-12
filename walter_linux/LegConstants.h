#pragma once

//in mm
const double legConst_radius = 112;
const double legConst_femurLength = 58.5;
const double legConst_thighLength = 80.7;
const double legConst_footLength = 158.6;
const double legConst_afTheta0s[6] =
//{
//    330.0,
//    30.0,
//    90.0,
//    270.0,
//    210.0,
//    150.0
//};

{
    180.0,
    240.0,
    300.0,
    120.0,
    60.0,
    360.0
};

//TODO: figure these out (in mm)
const double legConst_neutralPointX = 120;
const double legConst_neutralPointY = 0;
const double legConst_neutralPointZ = 125;