#include <Arduino.h>

extern "C" {
#ifndef HAL_I3C_MODULE_ENABLED
#define HAL_I3C_MODULE_ENABLED
#endif
#include "stm32u3xx_hal.h"
#include "stm32u3xx_hal_i3c.h"
#include "stm32u3xx_hal_pwr_ex.h"
#include "stm32u3xx_ll_i3c.h"
}

HardwareSerial VCP(PA10_ALT1, PA9_ALT1);  // RX, TX: USART1 -> ST-LINK VCP

static I3C_HandleTypeDef hi3c2;

static constexpr uint8_t IMU_COUNT = 2;
static constexpr uint8_t IMU_ASSIGN_STATIC_ADDR = 0x6B;  // TA0/SA0 high
static constexpr uint32_t STREAM_BAUD = 921600;
static constexpr uint32_t STREAM_INTERVAL_US = 8333;      // target ~120 Hz frames; actual rate is logged/checked offline

static constexpr uint8_t Broadcast_DISEC = 0x01;
static constexpr uint8_t Broadcast_RSTDAA = 0x06;
static constexpr uint8_t Direct_SETDASA = 0x87;

static constexpr uint8_t REG_WHO_AM_I = 0x0F;
static constexpr uint8_t REG_CTRL1 = 0x10;
static constexpr uint8_t REG_CTRL2 = 0x11;
static constexpr uint8_t REG_CTRL3 = 0x12;
static constexpr uint8_t REG_CTRL6 = 0x15;
static constexpr uint8_t REG_CTRL8 = 0x17;
static constexpr uint8_t REG_OUTX_L_G = 0x22;

static constexpr uint8_t WHO_EXPECTED = 0x73;

struct ImuSlot {
  const char *label;
  const char *position;
  const char *ta0Name;
  GPIO_TypeDef *ta0Port;
  uint16_t ta0Pin;
  uint8_t dynAddr;
  bool ok;
};

static ImuSlot imus[IMU_COUNT] = {
  {"IMU0", "CHILD_TOP_MOVING", "D2/PC8", GPIOC, GPIO_PIN_8, 0x32, false},
  {"IMU1", "PARENT_BOTTOM_FIXED", "D4/PB5", GPIOB, GPIO_PIN_5, 0x33, false},
};

static volatile bool i3cDone = false;
static volatile bool i3cError = false;

static uint32_t ctrlBuf[24];
static uint8_t txBuf[64];
static uint8_t rxBuf[32];

static void printHex8(uint8_t value);
static void printPositionMap();
static void printTableHeader();
static void initTa0Gpios();
static void selectOnlyTa0(uint8_t index);
static void setAllTa0Low();
static bool waitI3CReady(uint32_t timeoutMs);
static bool initI3C2();
static bool resetDynamicAddresses();
static bool setDynamicAddress(ImuSlot &imu);
static bool assignDynamicAddresses();
static bool i3cWriteReg(uint8_t dynAddr, uint8_t reg, uint8_t value);
static bool i3cReadReg(uint8_t dynAddr, uint8_t reg, uint8_t *data, uint16_t len);
static bool initISM6HG256X(ImuSlot &imu);
static bool readImuBurst(uint8_t dynAddr, int16_t &gx, int16_t &gy, int16_t &gz, int16_t &ax, int16_t &ay, int16_t &az);

extern "C" void HAL_I3C_MspInit(I3C_HandleTypeDef *hi3c)
{
  if (hi3c->Instance != I3C2) {
    return;
  }

  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_EnablePullUpPullDownConfig();
  HAL_PWREx_EnableI3CPullUp(PWR_I3CPU_PB13);  // PB13 / D15 / I3C2_SCL
  HAL_PWREx_EnableI3CPullUp(PWR_I3CPU_PB14);  // PB14 / D14 / I3C2_SDA

  RCC_PeriphCLKInitTypeDef clk = {};
  clk.PeriphClockSelection = RCC_PERIPHCLK_I3C2;
  clk.I3c2ClockSelection = RCC_I3C2CLKSOURCE_PCLK2;
  HAL_RCCEx_PeriphCLKConfig(&clk);

  __HAL_RCC_I3C2_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {};
  gpio.Pin = GPIO_PIN_13 | GPIO_PIN_14;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF6_I3C2;
  HAL_GPIO_Init(GPIOB, &gpio);

  HAL_NVIC_SetPriority(I3C2_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I3C2_EV_IRQn);
  HAL_NVIC_SetPriority(I3C2_ER_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I3C2_ER_IRQn);

  HAL_Delay(1);
}

extern "C" void I3C2_EV_IRQHandler(void)
{
  HAL_I3C_EV_IRQHandler(&hi3c2);
}

extern "C" void I3C2_ER_IRQHandler(void)
{
  HAL_I3C_ER_IRQHandler(&hi3c2);
}

extern "C" void HAL_I3C_CtrlTxCpltCallback(I3C_HandleTypeDef *hi3c)
{
  if (hi3c->Instance == I3C2) {
    i3cDone = true;
  }
}

extern "C" void HAL_I3C_CtrlRxCpltCallback(I3C_HandleTypeDef *hi3c)
{
  if (hi3c->Instance == I3C2) {
    i3cDone = true;
  }
}

extern "C" void HAL_I3C_CtrlMultipleXferCpltCallback(I3C_HandleTypeDef *hi3c)
{
  if (hi3c->Instance == I3C2) {
    i3cDone = true;
  }
}

extern "C" void HAL_I3C_ErrorCallback(I3C_HandleTypeDef *hi3c)
{
  if (hi3c->Instance == I3C2) {
    i3cError = true;
    i3cDone = true;
  }
}

void setup()
{
  VCP.begin(STREAM_BAUD);
  delay(300);
  VCP.println();
  VCP.println("ISM6HG256X I3C 2-IMU twist bench xyz stream");
  printPositionMap();

  initTa0Gpios();
  VCP.println("TA0 GPIO init OK");

  if (!initI3C2()) {
    VCP.println("ERR: I3C2 init failed");
    return;
  }
  VCP.println("I3C2 init OK");

  if (!assignDynamicAddresses()) {
    VCP.println("ERR: SETDASA failed. Check TA0 wires, SDA/SCL, pull-ups, power, and CS high.");
    return;
  }

  for (uint8_t i = 0; i < IMU_COUNT; i++) {
    imus[i].ok = initISM6HG256X(imus[i]);
    if (!imus[i].ok) {
      VCP.print("ERR: ");
      VCP.print(imus[i].label);
      VCP.println(" init/read failed");
      return;
    }
  }

  VCP.println("All IMUs config OK");
  printTableHeader();
}

void loop()
{
  static uint32_t lastHeaderMs = 0;
  static uint32_t nextFrameUs = 0;
  const uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextFrameUs) < 0) {
    return;
  }
  if ((uint32_t)(nowUs - nextFrameUs) > STREAM_INTERVAL_US * 4UL) {
    nextFrameUs = nowUs;
  } else {
    nextFrameUs += STREAM_INTERVAL_US;
  }

  const uint32_t now = millis();

  if (now - lastHeaderMs > 5000) {
    printTableHeader();
    lastHeaderMs = now;
  }

  for (uint8_t i = 0; i < IMU_COUNT; i++) {
    int16_t gxRaw = 0, gyRaw = 0, gzRaw = 0;
    int16_t axRaw = 0, ayRaw = 0, azRaw = 0;

    if (!readImuBurst(imus[i].dynAddr, gxRaw, gyRaw, gzRaw, axRaw, ayRaw, azRaw)) {
      VCP.print(now);
      VCP.print('\t');
      VCP.print(imus[i].label);
      VCP.print('\t');
      VCP.print(imus[i].position);
      VCP.println("\tREAD_FAIL");
      continue;
    }

    const float axMg = axRaw * 0.488f;            // low-g accel FS = +/-16 g
    const float ayMg = ayRaw * 0.488f;
    const float azMg = azRaw * 0.488f;
    const float gxDps = gxRaw * 70.0f / 1000.0f;  // gyro FS = +/-2000 dps
    const float gyDps = gyRaw * 70.0f / 1000.0f;
    const float gzDps = gzRaw * 70.0f / 1000.0f;

    VCP.print(now);
    VCP.print('\t');
    VCP.print(imus[i].label);
    VCP.print('\t');
    VCP.print(imus[i].position);
    VCP.print('\t');
    VCP.print("0x");
    printHex8(imus[i].dynAddr);
    VCP.print('\t');
    VCP.print(axMg, 1);
    VCP.print('\t');
    VCP.print(ayMg, 1);
    VCP.print('\t');
    VCP.print(azMg, 1);
    VCP.print('\t');
    VCP.print(gxDps, 2);
    VCP.print('\t');
    VCP.print(gyDps, 2);
    VCP.print('\t');
    VCP.println(gzDps, 2);
  }

  // No fixed delay here: the frame scheduler above controls the target stream rate.
}

static void printHex8(uint8_t value)
{
  if (value < 0x10) {
    VCP.print('0');
  }
  VCP.print(value, HEX);
}

static void printPositionMap()
{
  VCP.println("slot\tposition\tTA0_pin\tdyn_addr");
  for (uint8_t i = 0; i < IMU_COUNT; i++) {
    VCP.print(imus[i].label);
    VCP.print('\t');
    VCP.print(imus[i].position);
    VCP.print('\t');
    VCP.print(imus[i].ta0Name);
    VCP.print('\t');
    VCP.print("0x");
    printHex8(imus[i].dynAddr);
    VCP.println();
  }
  VCP.println();
}

static void printTableHeader()
{
  VCP.println("t_ms\timu\tposition\taddr\tax_mg\tay_mg\taz_mg\tgx_dps\tgy_dps\tgz_dps");
}

static void initTa0Gpios()
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {};
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;

  for (uint8_t i = 0; i < IMU_COUNT; i++) {
    gpio.Pin = imus[i].ta0Pin;
    HAL_GPIO_Init(imus[i].ta0Port, &gpio);
  }

  setAllTa0Low();
}

static void selectOnlyTa0(uint8_t index)
{
  for (uint8_t i = 0; i < IMU_COUNT; i++) {
    HAL_GPIO_WritePin(imus[i].ta0Port, imus[i].ta0Pin, (i == index) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
  delay(2);
}

static void setAllTa0Low()
{
  for (uint8_t i = 0; i < IMU_COUNT; i++) {
    HAL_GPIO_WritePin(imus[i].ta0Port, imus[i].ta0Pin, GPIO_PIN_RESET);
  }
  delay(2);
}

static bool waitI3CReady(uint32_t timeoutMs)
{
  const uint32_t start = millis();
  while (!i3cDone && !i3cError && (millis() - start < timeoutMs)) {
    yield();
  }
  return i3cDone && !i3cError && (HAL_I3C_GetState(&hi3c2) == HAL_I3C_STATE_READY);
}

static bool initI3C2()
{
  I3C_FifoConfTypeDef fifo = {};
  I3C_CtrlConfTypeDef ctrl = {};

  hi3c2.Instance = I3C2;
  hi3c2.Mode = HAL_I3C_MODE_CONTROLLER;
  hi3c2.Init.CtrlBusCharacteristic.SDAHoldTime = HAL_I3C_SDA_HOLD_TIME_0_5;
  hi3c2.Init.CtrlBusCharacteristic.WaitTime = HAL_I3C_OWN_ACTIVITY_STATE_0;
  hi3c2.Init.CtrlBusCharacteristic.SCLPPLowDuration = 0x2F;
  hi3c2.Init.CtrlBusCharacteristic.SCLI3CHighDuration = 0x2F;
  hi3c2.Init.CtrlBusCharacteristic.SCLODLowDuration = 0x2F;
  hi3c2.Init.CtrlBusCharacteristic.SCLI2CHighDuration = 0x00;
  hi3c2.Init.CtrlBusCharacteristic.BusFreeDuration = 0x13;
  hi3c2.Init.CtrlBusCharacteristic.BusIdleDuration = 0x5E;

  if (HAL_I3C_Init(&hi3c2) != HAL_OK) {
    return false;
  }

  fifo.RxFifoThreshold = HAL_I3C_RXFIFO_THRESHOLD_4_4;
  fifo.TxFifoThreshold = HAL_I3C_TXFIFO_THRESHOLD_1_4;
  fifo.ControlFifo = HAL_I3C_CONTROLFIFO_DISABLE;
  fifo.StatusFifo = HAL_I3C_STATUSFIFO_DISABLE;
  if (HAL_I3C_SetConfigFifo(&hi3c2, &fifo) != HAL_OK) {
    return false;
  }

  ctrl.DynamicAddr = 0;
  ctrl.StallTime = 0;
  ctrl.HotJoinAllowed = DISABLE;
  ctrl.ACKStallState = DISABLE;
  ctrl.CCCStallState = DISABLE;
  ctrl.TxStallState = DISABLE;
  ctrl.RxStallState = DISABLE;
  ctrl.HighKeeperSDA = DISABLE;
  return HAL_I3C_Ctrl_Config(&hi3c2, &ctrl) == HAL_OK;
}

static bool resetDynamicAddresses()
{
  uint8_t disecPayload = 0x08;
  I3C_CCCTypeDef broadcastCcc[] = {
    {0, Broadcast_DISEC, {&disecPayload, 1}, LL_I3C_DIRECTION_WRITE},
    {0, Broadcast_RSTDAA, {NULL, 0}, LL_I3C_DIRECTION_WRITE},
  };

  I3C_XferTypeDef xfer = {};
  memset(ctrlBuf, 0, sizeof(ctrlBuf));
  memset(txBuf, 0, sizeof(txBuf));
  xfer.CtrlBuf.pBuffer = ctrlBuf;
  xfer.CtrlBuf.Size = 16;
  xfer.TxBuf.pBuffer = txBuf;
  xfer.TxBuf.Size = 1;

  if (HAL_I3C_AddDescToFrame(&hi3c2, broadcastCcc, NULL, &xfer, 2, I3C_BROADCAST_WITHOUT_DEFBYTE_RESTART) != HAL_OK) {
    return false;
  }

  i3cDone = false;
  i3cError = false;
  if (HAL_I3C_Ctrl_TransmitCCC_IT(&hi3c2, &xfer) != HAL_OK) {
    return false;
  }
  return waitI3CReady(1000);
}

static bool setDynamicAddress(ImuSlot &imu)
{
  uint8_t setdasaPayload = imu.dynAddr << 1;
  I3C_CCCTypeDef directCcc[] = {
    {IMU_ASSIGN_STATIC_ADDR, Direct_SETDASA, {&setdasaPayload, 1}, LL_I3C_DIRECTION_WRITE},
  };

  I3C_XferTypeDef xfer = {};
  memset(ctrlBuf, 0, sizeof(ctrlBuf));
  memset(txBuf, 0, sizeof(txBuf));
  xfer.CtrlBuf.pBuffer = ctrlBuf;
  xfer.CtrlBuf.Size = 16;
  xfer.TxBuf.pBuffer = txBuf;
  xfer.TxBuf.Size = 1;

  if (HAL_I3C_AddDescToFrame(&hi3c2, directCcc, NULL, &xfer, 1, I3C_DIRECT_WITHOUT_DEFBYTE_STOP) != HAL_OK) {
    return false;
  }

  i3cDone = false;
  i3cError = false;
  if (HAL_I3C_Ctrl_TransmitCCC_IT(&hi3c2, &xfer) != HAL_OK) {
    return false;
  }
  if (!waitI3CReady(1000)) {
    return false;
  }

  return HAL_I3C_Ctrl_IsDeviceI3C_Ready(&hi3c2, imu.dynAddr, 300, 1000) == HAL_OK;
}

static bool assignDynamicAddresses()
{
  setAllTa0Low();

  if (!resetDynamicAddresses()) {
    VCP.println("ERR: broadcast DISEC/RSTDAA failed");
    return false;
  }

  for (uint8_t i = 0; i < IMU_COUNT; i++) {
    selectOnlyTa0(i);

    VCP.print("SETDASA ");
    VCP.print(imus[i].label);
    VCP.print(" via ");
    VCP.print(imus[i].ta0Name);
    VCP.print(": static 0x6B -> dyn 0x");
    printHex8(imus[i].dynAddr);
    VCP.print(" ... ");

    if (!setDynamicAddress(imus[i])) {
      VCP.println("FAIL");
      return false;
    }

    VCP.println("OK");
    setAllTa0Low();
  }

  return true;
}

static bool i3cWriteReg(uint8_t dynAddr, uint8_t reg, uint8_t value)
{
  uint8_t payload[2] = {reg, value};
  I3C_PrivateTypeDef desc = {dynAddr, {payload, sizeof(payload)}, {NULL, 0}, HAL_I3C_DIRECTION_WRITE};
  I3C_XferTypeDef xfer = {};

  memset(ctrlBuf, 0, sizeof(ctrlBuf));
  memset(txBuf, 0, sizeof(txBuf));
  xfer.CtrlBuf.pBuffer = ctrlBuf;
  xfer.CtrlBuf.Size = 1;
  xfer.TxBuf.pBuffer = txBuf;
  xfer.TxBuf.Size = sizeof(payload);

  if (HAL_I3C_AddDescToFrame(&hi3c2, NULL, &desc, &xfer, 1, I3C_PRIVATE_WITH_ARB_STOP) != HAL_OK) {
    return false;
  }

  i3cDone = false;
  i3cError = false;
  if (HAL_I3C_Ctrl_Transmit_IT(&hi3c2, &xfer) != HAL_OK) {
    return false;
  }
  return waitI3CReady(1000);
}

static bool i3cReadReg(uint8_t dynAddr, uint8_t reg, uint8_t *data, uint16_t len)
{
  I3C_PrivateTypeDef desc[2] = {
    {dynAddr, {&reg, 1}, {NULL, 0}, HAL_I3C_DIRECTION_WRITE},
    {dynAddr, {NULL, 0}, {data, len}, HAL_I3C_DIRECTION_READ},
  };
  I3C_XferTypeDef xfer = {};

  memset(ctrlBuf, 0, sizeof(ctrlBuf));
  memset(txBuf, 0, sizeof(txBuf));
  xfer.CtrlBuf.pBuffer = ctrlBuf;
  xfer.CtrlBuf.Size = 2;
  xfer.TxBuf.pBuffer = txBuf;
  xfer.TxBuf.Size = 1;
  xfer.RxBuf.pBuffer = data;
  xfer.RxBuf.Size = len;

  if (HAL_I3C_AddDescToFrame(&hi3c2, NULL, desc, &xfer, 2, I3C_PRIVATE_WITH_ARB_RESTART) != HAL_OK) {
    return false;
  }

  i3cDone = false;
  i3cError = false;
  if (HAL_I3C_Ctrl_MultipleTransfer_IT(&hi3c2, &xfer) != HAL_OK) {
    return false;
  }
  return waitI3CReady(1000);
}

static bool initISM6HG256X(ImuSlot &imu)
{
  uint8_t who = 0;
  if (!i3cReadReg(imu.dynAddr, REG_WHO_AM_I, &who, 1)) {
    return false;
  }

  VCP.print(imu.label);
  VCP.print(" WHO_AM_I = 0x");
  printHex8(who);
  VCP.println();

  if (who != WHO_EXPECTED) {
    VCP.println("ERR: expected WHO_AM_I 0x73");
    return false;
  }

  if (!i3cWriteReg(imu.dynAddr, REG_CTRL3, 0x44)) {  // BDU + IF_INC
    return false;
  }
  if (!i3cWriteReg(imu.dynAddr, REG_CTRL8, 0x03)) {  // accel FS = +/-16 g
    return false;
  }
  if (!i3cWriteReg(imu.dynAddr, REG_CTRL6, 0x04)) {  // gyro FS = +/-2000 dps
    return false;
  }
  if (!i3cWriteReg(imu.dynAddr, REG_CTRL1, 0x06)) {  // accel ODR = 120 Hz
    return false;
  }
  if (!i3cWriteReg(imu.dynAddr, REG_CTRL2, 0x06)) {  // gyro ODR = 120 Hz
    return false;
  }

  delay(20);
  return true;
}

static bool readImuBurst(uint8_t dynAddr, int16_t &gx, int16_t &gy, int16_t &gz, int16_t &ax, int16_t &ay, int16_t &az)
{
  if (!i3cReadReg(dynAddr, REG_OUTX_L_G, rxBuf, 12)) {
    return false;
  }

  gx = (int16_t)((uint16_t)rxBuf[1] << 8 | rxBuf[0]);
  gy = (int16_t)((uint16_t)rxBuf[3] << 8 | rxBuf[2]);
  gz = (int16_t)((uint16_t)rxBuf[5] << 8 | rxBuf[4]);
  ax = (int16_t)((uint16_t)rxBuf[7] << 8 | rxBuf[6]);
  ay = (int16_t)((uint16_t)rxBuf[9] << 8 | rxBuf[8]);
  az = (int16_t)((uint16_t)rxBuf[11] << 8 | rxBuf[10]);
  return true;
}
