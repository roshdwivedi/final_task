/*
 * bh1750_config.c
 *
 *  Created on: Feb 19, 2022
 *      Author: ROshan
 */


#include "bh1750.h"
#include "bh1750_config.h"
#include "main.h"
#include "i2c.h"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/
BH1750_HandleTypeDef hbh1750_1 = {
  .I2C = &hi2c1, .Address = BH1750_ADDRESS_L, .Timeout = 0xffff
};

BH1750_HandleTypeDef hbh1750_2 = {
  .I2C = &hi2c1, .Address = BH1750_ADDRESS_H, .Timeout = 0xffff
};

/* Private function prototypes -----------------------------------------------*/

/* Private function ----------------------------------------------------------*/

/* Public function -----------------------------------------------------------*/
