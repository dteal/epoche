#!/usr/bin/python3

#https://toglefritz.com/hexapod-inverse-kinematics-equations/

BodySideLength = 45
CoxaLength = 35
FemurLength = 50
TibiaLength = 95

PosX = 0
PosY = 0
PosZ = 0
RotX = 0
RotY = 0
RotZ = 0

BodyCenterOffset1 = BodySideLength/2
BodyCenterOffset2 = sqrt(BodySideLength^2 - BodyCenterOffset1^2)

# Body Center Offset X

BodyCenterOffsetX_1 = BodyCenterOffset1
BodyCenterOffsetX_2 = BodySideLength
BodyCenterOffsetX_3 = BodyCenterOffset1
BodyCenterOffsetX_4 = -BodyCenterOffset1
BodyCenterOffsetX_5 = -BodySideLength
BodyCenterOffsetX_6 = -BodyCenterOffset1

# Body Center Offset Y

BodyCenterOffsetY_1 = BodyCenterOffset2
BodyCenterOffsetY_2 = 0
BodyCenterOffsetY_3 = -BodyCenterOffset2
BodyCenterOffsetY_4 = -BodyCenterOffset2
BodyCenterOffsetY_5 = 0
BodyCenterOffsetY_6 = BodyCenterOffset2

# Initial Feet Positions

FeetPosX_1 = cos(60/180*PI)*(CoxaLength + FemurLength)
FeetPosZ_1 = TibiaLength
FeetPosY_1 = sin(60/180*PI)*(CoxaLength + FemurLength)

FeetPosX_2 = CoxaLength + FemurLength
FeetPosZ_2 = TibiaLength
FeetPosY_2 = 0

FeetPosX_3 = cos(60/180*PI)*(CoxaLength + FemurLength)
FeetPosZ_3 = TibiaLength
FeetPosY_3 = sin(-60/180*PI)*(CoxaLength + FemurLength)

FeetPosX_4 = -cos(60/180*PI)*(CoxaLength + FemurLength)
FeetPosZ_4 = TibiaLength
FeetPosY_4 = sin(-60/180*PI)*(CoxaLength + FemurLength)

FeetPosX_5 = -(CoxaLength + FemurLength)
FeetPosZ_5 = TibiaLength
FeetPosY_5 = 0

FeetPosX_6 = -cos(60/180*PI)*(CoxaLength + FemurLength)
FeetPosZ_6 = TibiaLength
FeetPosY_6 = sin(60/180*PI)*(CoxaLength + FemurLength)

# Body IK

TotalY_1 = FeetPosY_1 + BodyCenterOffsetY_1 + PosY
TotalX_1 = FeetPosX_1 + BodyCenterOffsetX_1 + PosX
DistBodyCenterFeet_1 = sqrt(TotalY_1^2 + TotalX_1^2)
AngleBodyCenterX_1 = PI/2 - atan2(TotalY_1, TotalX_1)
RollZ_1 = tan(RotZ * PI/180) * TotalX_1
PitchZ_1 = tan(RotX * PI/180) * TotalY_1
BodyIKX_1 = cos(AngleBodyCenterX_1 + (RotY * PI/180)) * DistBodyCenterFeet_1 - TotalX_1
BodyIKY_1 = (sin(AngleBodyCenterX_1 + (RotY  * PI/180)) * DistBodyCenterFeet_1) - TotalY_1
BodyIKZ_1 = RollZ_1 + PitchZ_1

TotalY_2 = FeetPosY_2 + BodyCenterOffsetY_2 + PosY
TotalX_2 = FeetPosX_2 + PosX + BodyCenterOffsetX_2
DistBodyCenterFeet_2 = sqrt(TotalY_2^2 + TotalX_2^2)
AngleBodyCenterX_2 = PI/2 - atan2(TotalY_2, TotalX_2)
RollZ_2 = tan(RotZ * PI/180) * TotalX_2
PitchZ_2 = tan(RotX * PI/180) * TotalY_2
BodyIKX_2 = cos(AngleBodyCenterX_2 + (RotY * PI/180)) * DistBodyCenterFeet_2 - TotalX_2
BodyIKY_2 = (sin(AngleBodyCenterX_2 + (RotY  * PI/180)) * DistBodyCenterFeet_2) - TotalY_2
BodyIKZ_2 = RollZ_2 + PitchZ_2

TotalY_3 = FeetPosY_3 + BodyCenterOffsetY_3 + PosY
TotalX_3 = FeetPosX_3 + BodyCenterOffsetX_3 + PosX
DistBodyCenterFeet_3 = sqrt(TotalY_3^2 + TotalX_3^2)
AngleBodyCenterX_3 = PI/2 - atan2(TotalY_3, TotalX_3)
RollZ_3 = tan(RotZ * PI/180) * TotalX_3
PitchZ_3 = tan(RotX * PI/180) * TotalY_3
BodyIKX_3 = cos(AngleBodyCenterX_3 + (RotY * PI/180)) * DistBodyCenterFeet_3 - TotalX_3
BodyIKY_3 = (sin(AngleBodyCenterX_3 + (RotY  * PI/180)) * DistBodyCenterFeet_3) - TotalY_3
BodyIKZ_3 = RollZ_3 + PitchZ_3

TotalY_4 = FeetPosY_4 + BodyCenterOffsetY_4 + PosY
TotalX_4 = FeetPosX_4 + BodyCenterOffsetX_4 + PosX
DistBodyCenterFeet_4 = sqrt(TotalY_4^2 + TotalX_4^2)
AngleBodyCenterX_4 = PI/2 - atan2(TotalY_4, TotalX_4)
RollZ_4 = tan(RotZ * PI/180) * TotalX_4
PitchZ_4 = tan(RotX * PI/180) * TotalY_4
BodyIKX_4 = cos(AngleBodyCenterX_4 + (RotY * PI/180)) * DistBodyCenterFeet_4 - TotalX_4
BodyIKY_4 = (sin(AngleBodyCenterX_4 + (RotY  * PI/180)) * DistBodyCenterFeet_4) - TotalY_4
BodyIKZ_4 = RollZ_4 + PitchZ_4

TotalY_5 = FeetPosY_5 + BodyCenterOffsetY_5 + PosY
TotalX_5 = FeetPosX_5 + BodyCenterOffsetX_5 + PosX
DistBodyCenterFeet_5 = sqrt(TotalY_5^2 + TotalX_5^2)
AngleBodyCenterX_5 = PI/2 - atan2(TotalY_5, TotalX_5)
RollZ_5 = tan(RotZ * PI/180) * TotalX_5
PitchZ_5 = tan(RotX * PI/180) * TotalY_5
BodyIKX_5 = cos(AngleBodyCenterX_5 + (RotY * PI/180)) * DistBodyCenterFeet_5 - TotalX_5
BodyIKY_5 = (sin(AngleBodyCenterX_5 + (RotY  * PI/180)) * DistBodyCenterFeet_5) - TotalY_5
BodyIKZ_5 = RollZ_5 + PitchZ_5

TotalY_6 = FeetPosY_6 + BodyCenterOffsetZY_6 + PosY
TotalX_6 = FeetPosX_6 + BodyCenterOffsetX_6 + PosX
DistBodyCenterFeet_6 = sqrt(TotalY_6^2 + TotalX_6^2)
AngleBodyCenterX_6 = PI/2 - atan2(TotalY_6, TotalX_6)
RollZ_6 = tan(RotZ * PI/180) * TotalX_6
PitchZ_6 = tan(RotX * PI/180) * TotalY_6
BodyIKX_6 = cos(AngleBodyCenterX_6 + (RotY * PI/180)) * DistBodyCenterFeet_6 - TotalX_6
BodyIKY_6 = (sin(AngleBodyCenterX_6 + (RotY  * PI/180)) * DistBodyCenterFeet_6) - TotalY_6
BodyIKZ_6 = RollZ_6 + PitchZ_6

# More IK

NewPosX_1 = FeetPosX_1 + PosX +  BodyIKX_1
NewPosZ_1 = FeetPosZ_1 + PosZ + BodyIKZ_1
NewPosY_1 = FeetPosY_1 + PosY + BodyIKY_1
CoxaFeetDist_1 = sqrt(NewPosX_1^2   + NewPosY_1^2)
IKSW_1 = sqrt((CoxaFeetDist_1 - CoxaLength ) ^2 + NewPosZ_1^2)
IKA1_1 = atan((CoxaFeetDist_1 - CoxaLength)/NewPosZ_1)
IKA2_1 = acos((TibiaLength^2 - FemurLength^2 - IKSW_1^2)/(-2 * IKSW_1 *  FemurLength))
TAngle_1 = acos((IKSW_1^2 - TibiaLength^2 - FemurLength^2)/(-2 * FemurLength * TibiaLength))
IKTibiaAngle_1 = 90 - TAngle_1 * 180/PI
IKFemurAngle_1 = 90 - (IKA1_1 + IKA2_1) * 180/PI
IKCoxaAngle_1 = 90 - atan2(NewPosY_1, NewPosX_1) * 180/PI

NewPosX_2 = FeetPosX_2 + PosX +  BodyIKX_2
NewPosZ_2 = FeetPosZ_2 + PosZ + BodyIKZ_2
NewPosY_2 = FeetPosY_2 + PosY + BodyIKY_2
CoxaFeetDist_2 = sqrt(NewPosX_2^2   + NewPosY_2^2)
IKSW_2 = sqrt((CoxaFeetDist_2 - CoxaLength ) ^2 + NewPosZ_2^2)
IKA1_2 = atan((CoxaFeetDist_2 - CoxaLength)/NewPosZ_2)
IKA2_2 = acos((TibiaLength^2 - FemurLength^2 - IKSW_2^2)/(-2 * IKSW_2 *  FemurLength))
TAngle_2 = acos((IKSW_2^2 - TibiaLength^2 - FemurLength^2)/(-2 * FemurLength * TibiaLength))
IKTibiaAngle_2 = 90 - TAngle_2 * 180/PI
IKFemurAngle_2 = 90 - (IKA1_2 + IKA2_2) * 180/PI
IKCoxaAngle_2 = 90 - atan2(NewPosY_2, NewPosX_2) * 180/PI

NewPosX_3 = FeetPosX_3 + PosX +  BodyIKX_3
NewPosZ_3 = FeetPosZ_3 + PosZ + BodyIKZ_3
NewPosY_3 = FeetPosY_2 + PosY + BodyIKY_2
CoxaFeetDist_3 = sqrt(NewPosX_3^2   + NewPosY_3^2)
IKSW_3 = sqrt((CoxaFeetDist_3 - CoxaLength ) ^2 + NewPosZ_3^2)
IKA1_3 = atan((CoxaFeetDist_3 - CoxaLength)/NewPosZ_3)
IKA2_3 = acos((TibiaLength^2 - FemurLength^2 - IKSW_3^2)/(-2 * IKSW_3 *  FemurLength))
TAngle_3 = acos((IKSW_3^2 - TibiaLength^2 - FemurLength^2)/(-2 * FemurLength * TibiaLength))
IKTibiaAngle_3 = 90 - TAngle_3 * 180/PI
IKFemurAngle_3 = 90 - (IKA1_3 + IKA2_3) * 180/PI
IKCoxaAngle_3 = 90 - atan2(NewPosY_3, NewPosX_3) * 180/PI

NewPosX_4 = FeetPosX_4 + PosX +  BodyIKX_4
NewPosZ_4 = FeetPosZ_4 + PosYZ + BodyIKZ_4
NewPosY_4 = FeetPosY_4 + PosY + BodyIKY_4
CoxaFeetDist_4 = sqrt(NewPosX_4^2   + NewPosY_4^2)
IKSW_4 = sqrt((CoxaFeetDist_4 - CoxaLength ) ^2 + NewPosZ_4^2)
IKA1_4 = atan((CoxaFeetDist_4 - CoxaLength)/NewPosZ_4)
IKA2_4 = acos((TibiaLength^2 - FemurLength^2 - IKSW_4^2)/(-2 * IKSW_4 *  FemurLength))
TAngle_4 = acos((IKSW_4^2 - TibiaLength^2 - FemurLength^2)/(-2 * FemurLength * TibiaLength))
IKTibiaAngle_4 = 90 - TAngle_4 * 180/PI
IKFemurAngle_4 = 90 - (IKA1_4 + IKA2_4) * 180/PI
IKCoxaAngle_4 = 90 - atan2(NewPosY_4, NewPosX_4) * 180/PI

NewPosX_5 = FeetPosX_5 + PosX +  BodyIKX_5
NewPosZ_5 = FeetPosZ_5 + PosZ + BodyIKZ_5
NewPosY_5 = FeetPosY_5 + PosY + BodyIKY_5
CoxaFeetDist_5 = sqrt(NewPosX_5^2   + NewPosY_5^2)
IKSW_5 = sqrt((CoxaFeetDist_5 - CoxaLength ) ^2 + NewPosZ_5^2)
IKA1_5 = atan((CoxaFeetDist_5 - CoxaLength)/NewPosZ_5)
IKA2_5 = acos((TibiaLength^2 - FemurLength^2 - IKSW_5^2)/(-2 * IKSW_5 *  FemurLength))
TAngle_5 = acos((IKSW_5^2 - TibiaLength^2 - FemurLength^2)/(-2 * FemurLength * TibiaLength))
IKTibiaAngle_5 = 90 - TAngle_5 * 180/PI
IKFemurAngle_5 = 90 - (IKA1_5 + IKA2_5) * 180/PI
IKCoxaAngle_5 = 90 - atan2(NewPosY_5, NewPosX_5) * 180/PI

NewPosX_6 = FeetPosX_6 + PosX +  BodyIKX_6
NewPosZ_6 = FeetPosZ_6 + PosZ + BodyIKZ_6
NewPosY_6 = FeetPosY_6 + PosY + BodyIKY_6
CoxaFeetDist_6 = sqrt(NewPosX_6^2   + NewPosY_6^2)
IKSW_6 = sqrt((CoxaFeetDist_6 - CoxaLength ) ^2 + NewPosZ_6^2)
IKA1_6 = atan((CoxaFeetDist_6 - CoxaLength)/NewPosZ_6)
IKA2_6 = acos((TibiaLength^2 - FemurLength^2 - IKSW_6^2)/(-2 * IKSW_6 *  FemurLength))
TAngle_6 = acos((IKSW_6^2 - TibiaLength^2 - FemurLength^2)/(-2 * FemurLength * TibiaLength))
IKTibiaAngle_6 = 90 - TAngle_6 * 180/PI
IKFemurAngle_6 = 90 - (IKA1_6 + IKA2_6) * 180/PI
IKCoxaAngle_6 = 90 - atan2(NewPosY_6, NewPosX_6) * 180/PI

# Servo angles

CoxaAngle_1 = IKCoxaAngle_1 - 60
FemurAngle_1 = IKFemurAngle_1
TibiaAngle_1 = IKTibiaAngle_1

CoxaAngle_2 = IKCoxaAngle_2
FemurAngle_2 = IKFemurAngle_2
TibiaAngle_2 = IKTibiaAngle_2

CoxaAngle_3 = IKCoxaAngle_3 + 60
FemurAngle_3 = IKFemurAngle_3
TibiaAngle_3 = IKTibiaAngle_3

CoxaAngle_4 = IKCoxaAngle_4 - 240
FemurAngle_4 = IKFemurAngle_4
TibiaAngle_4 = IKTibiaAngle_4

CoxaAngle_5 = IKCoxaAngle_5 - 180
FemurAngle_5 = IKFemurAngle_5
TibiaAngle_5 = IKTibiaAngle_5

CoxaAngle_6 = IKCoxaAngle_6 - 120
FemurAngle_6 = IKFemurAngle_6
TibiaAngle_6 = IKTibiaAngle_6


