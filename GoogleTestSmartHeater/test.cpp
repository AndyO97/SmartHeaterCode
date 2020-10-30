#include "pch.h"
#include "AdcToTemperature.c"
#include "SoftwareFuse.c"
#include "fmiErrors.c"


/* VALID TEST CASES */

/* ADC */
/* 256bit counts */
TEST(fullConversionMainTestCase, A2DtoVoltageToCelsius) {
  EXPECT_NEAR(210, fullConversionMain(256),0.01);
}


/* 512bit counts */
TEST(fullConversionMainTestCase, A2DtoVoltageToCelsiusOvervoltage) {
	EXPECT_NEAR(210, fullConversionMain(512), 0.01);
}

/* 128bit counts */
TEST(fullConversionMainTestCase, A2DtoVoltageToMidVoltage) {
	EXPECT_NEAR(85, fullConversionMain(128), 0.01);
}

/* 0bit counts */
TEST(fullConversionMainTestCase, A2DtoVoltageToCelsius0BitCounts) {
	EXPECT_NEAR(-40, fullConversionMain(0), 0.01);
}

/* negativebit counts */
TEST(fullConversionMainTestCase, A2DtoVoltageToCelsiusNegativeBitCounts) {
	EXPECT_NEAR(-40, fullConversionMain(-256), 0.01);
}




/* Fuse */
int zero = 0;
int three = 3;
int almostTimeout = 1900000;

/* Below Threshold current */
TEST(softwareFuseMainTestCase, currentToFuse24mA) {
	EXPECT_EQ(0, softwareFuseMain(24,&zero));
}

/* Above Threshold current fuse doesn't break */
TEST(softwareFuseMainTestCase, currentToFuse87_5mA) {
	EXPECT_EQ(0, softwareFuseMain(87.5,&three));
}

/* Above Threshold current fuse does break */
TEST(softwareFuseMainTestCase, currentToFuse87_5mA2) {
	EXPECT_EQ(1, softwareFuseMain(87.5,&almostTimeout));
}

/* FMI Erros */

/* No error should be identified */
TEST(fmiErrorsMainTestCase, NoErrors) {
	EXPECT_EQ(7, fmiErrorsMain(256, 210, 24));
}

/* No error should be identified */
TEST(fmiErrorsMainTestCase, Error0) {
	EXPECT_EQ(0, fmiErrorsMain(512, 210, 24));
}

