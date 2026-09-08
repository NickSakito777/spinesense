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

static constexpr uint8_t IMU_COUNT = 5;
static constexpr uint8_t STATIC_ADDR_TA0_HIGH = 0x6B;
static constexpr uint8_t WHO_AM_I_REG = 0x0F;
static constexpr uint8_t WHO_EXPECTED = 0x73;

static constexpr uint8_t Broadcast_DISEC = 0x01;
static constexpr uint8_t Broadcast_RSTDAA = 0x06;
static constexpr uint8_t Direct_SETDASA = 0x87;

// DAA robustness for long-yarn/garment bus (2026-08 field patch):
// settle after power before first DAA; retry RSTDAA/SETDASA on marginal contact.
static constexpr uint32_t BOOT_SETTLE_MS = 150;
static constexpr uint8_t RSTDAA_RETRIES = 3;
static constexpr uint8_t SETDASA_RETRIES = 3;
static constexpr uint32_t RETRY_GAP_MS = 30;

struct ImuSlot {
  const char *name;
  const char *ta0PinName;
  GPIO_TypeDef *ta0Port;
  uint16_t ta0Pin;
  uint8_t dynAddr;
};

static ImuSlot imus[IMU_COUNT] = {
  // Handoff 2026-09-08: match the streaming sketch and package wiring table.
  {"IMU0", "D2/PC8", GPIOC, GPIO_PIN_8, 0x32},
  {"IMU1", "D4/PB5", GPIOB, GPIO_PIN_5, 0x33},
  // 2026-08-19: IMU2 TA0 moved D7/PA8 -> D6/PB10. Fault isolation showed the
  // failure stays with D7 regardless of which wire/chip it drives (suspect
  // dead GPIO/header pin). D6/PB10 per official NUCLEO_U385RG_Q variant map.
  {"IMU2", "D6/PB10", GPIOB, GPIO_PIN_10, 0x34},
  {"IMU3", "D8/PC7", GPIOC, GPIO_PIN_7, 0x35},
  {"IMU4", "D9/PC6", GPIOC, GPIO_PIN_6, 0x36},
};

static volatile bool i3cDone = false;
static volatile bool i3cError = false;

static uint32_t ctrlBuf[24];
static uint8_t txBuf[64];
static uint8_t rxBuf[8];
static bool i3cReady = false;

static void printHex8(uint8_t value);
static void initTa0Gpios();
static void selectOnlyTa0(uint8_t index);
static void setAllTa0Low();
static bool initI3C2();
static bool waitI3CReady(uint32_t timeoutMs);
static bool resetDynamicAddresses();
static bool setDynamicAddress(uint8_t dynAddr);
static bool readReg(uint8_t dynAddr, uint8_t reg, uint8_t *data, uint16_t len);
static bool testOneImu(uint8_t index);
static void runConnectivityTest();

extern "C" void HAL_I3C_MspInit(I3C_HandleTypeDef *hi3c)
{
  if (hi3c->Instance != I3C2) {
    return;
  }

  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_EnablePullUpPullDownConfig();
  HAL_PWREx_EnableI3CPullUp(PWR_I3CPU_PB13);  // D15 / PB13 / I3C2_SCL
  HAL_PWREx_EnableI3CPullUp(PWR_I3CPU_PB14);  // D14 / PB14 / I3C2_SDA

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
  VCP.begin(115200);
  delay(300);
  VCP.println();
  VCP.println("=== ISM6HG256X 5-IMU I3C connectivity test ===");
  VCP.println("Expected wiring:");
  VCP.println("SCL=D15/PB13, SDA=D14/PB14");
  VCP.println("TA0: IMU0=D2, IMU1=D4, IMU2=D6, IMU3=D8, IMU4=D9");
  VCP.println("Each IMU: VDDIO+VDD=3V3, GND=GND, CS=3V3 recommended");
  VCP.println("D9 PWM label is OK here: it is used as a normal GPIO output.");
  VCP.println();

  initTa0Gpios();
  if (!initI3C2()) {
    VCP.println("I3C2_INIT: FAIL");
    return;
  }
  VCP.println("I3C2_INIT: PASS");
  // Let sensor VDD settle and on-chip boot finish even behind a resistive
  // contact (slow supply ramp) before the first DAA.
  delay(BOOT_SETTLE_MS);
  i3cReady = true;
  runConnectivityTest();
}

void loop()
{
  delay(3000);
  if (i3cReady) {
    runConnectivityTest();
  } else {
    VCP.println("I3C2 not ready");
  }
}

static void printHex8(uint8_t value)
{
  if (value < 0x10) {
    VCP.print('0');
  }
  VCP.print(value, HEX);
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

static bool initI3C2()
{
  I3C_FifoConfTypeDef fifo = {};
  I3C_CtrlConfTypeDef ctrl = {};

  hi3c2.Instance = I3C2;
  hi3c2.Mode = HAL_I3C_MODE_CONTROLLER;
  hi3c2.Init.CtrlBusCharacteristic.SDAHoldTime = HAL_I3C_SDA_HOLD_TIME_1_5;
  hi3c2.Init.CtrlBusCharacteristic.WaitTime = HAL_I3C_OWN_ACTIVITY_STATE_0;
  // Connectivity tool: run the whole bus at the slowest usable timing so the
  // open-drain DAA phase survives the high-RC garment bus (~4x slower than
  // the 0x2F streaming profile). Throughput is irrelevant here.
  hi3c2.Init.CtrlBusCharacteristic.SCLPPLowDuration = 0xFF;
  hi3c2.Init.CtrlBusCharacteristic.SCLI3CHighDuration = 0xFF;
  hi3c2.Init.CtrlBusCharacteristic.SCLODLowDuration = 0xFF;
  hi3c2.Init.CtrlBusCharacteristic.SCLI2CHighDuration = 0xF0;
  hi3c2.Init.CtrlBusCharacteristic.BusFreeDuration = 0x4C;
  hi3c2.Init.CtrlBusCharacteristic.BusIdleDuration = 0xFF;

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

static bool waitI3CReady(uint32_t timeoutMs)
{
  const uint32_t start = millis();
  while (!i3cDone && !i3cError && (millis() - start < timeoutMs)) {
    yield();
  }
  return i3cDone && !i3cError && (HAL_I3C_GetState(&hi3c2) == HAL_I3C_STATE_READY);
}

// After a NACK/timeout on a marginal bus the HAL can be left in an error
// state that fails every later transfer. Re-init the controller if needed.
static void recoverI3CIfNeeded()
{
  if (HAL_I3C_GetState(&hi3c2) == HAL_I3C_STATE_READY) {
    return;
  }
  HAL_I3C_DeInit(&hi3c2);
  delay(2);
  if (!initI3C2()) {
    VCP.println("I3C2_RECOVER: FAIL");
    i3cReady = false;
  }
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

static bool setDynamicAddress(uint8_t dynAddr)
{
  uint8_t payload = dynAddr << 1;
  I3C_CCCTypeDef directCcc[] = {
    {STATIC_ADDR_TA0_HIGH, Direct_SETDASA, {&payload, 1}, LL_I3C_DIRECTION_WRITE},
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
  return waitI3CReady(1000);
}

static bool readReg(uint8_t dynAddr, uint8_t reg, uint8_t *data, uint16_t len)
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

static bool testOneImu(uint8_t index)
{
  ImuSlot &imu = imus[index];
  selectOnlyTa0(index);

  VCP.print(imu.name);
  VCP.print(" TA0=");
  VCP.print(imu.ta0PinName);
  VCP.print(" static 0x6B -> dyn 0x");
  printHex8(imu.dynAddr);
  VCP.print(": ");

  bool assigned = false;
  for (uint8_t attempt = 1; attempt <= SETDASA_RETRIES; attempt++) {
    if (setDynamicAddress(imu.dynAddr)) {
      assigned = true;
      if (attempt > 1) {
        VCP.print("(SETDASA ok on try ");
        VCP.print(attempt);
        VCP.print(") ");
      }
      break;
    }
    recoverI3CIfNeeded();
    delay(RETRY_GAP_MS);
    selectOnlyTa0(index);
  }
  if (!assigned) {
    VCP.print("SETDASA_FAIL after ");
    VCP.print(SETDASA_RETRIES);
    VCP.println(" tries");
    setAllTa0Low();
    return false;
  }

  setAllTa0Low();

  if (HAL_I3C_Ctrl_IsDeviceI3C_Ready(&hi3c2, imu.dynAddr, 300, 1000) != HAL_OK) {
    VCP.println("NO_ACK");
    return false;
  }

  rxBuf[0] = 0;
  if (!readReg(imu.dynAddr, WHO_AM_I_REG, rxBuf, 1)) {
    VCP.println("WHO_READ_FAIL");
    return false;
  }

  VCP.print("WHO_AM_I=0x");
  printHex8(rxBuf[0]);
  if (rxBuf[0] == WHO_EXPECTED) {
    VCP.println(" PASS");
    return true;
  }

  VCP.println(" WRONG_ID");
  return false;
}

static void runConnectivityTest()
{
  VCP.println();
  VCP.println("--- 5-IMU connectivity scan (slow-bus DAA profile) ---");
  setAllTa0Low();

  bool rstOk = false;
  for (uint8_t attempt = 1; attempt <= RSTDAA_RETRIES; attempt++) {
    if (resetDynamicAddresses()) {
      rstOk = true;
      if (attempt > 1) {
        VCP.print("RSTDAA: PASS (try ");
        VCP.print(attempt);
        VCP.println(")");
      } else {
        VCP.println("RSTDAA: PASS");
      }
      break;
    }
    recoverI3CIfNeeded();
    delay(RETRY_GAP_MS);
  }
  if (!rstOk) {
    VCP.print("RSTDAA: FAIL after ");
    VCP.print(RSTDAA_RETRIES);
    VCP.println(" tries");
    VCP.println("Shared bus is down: check SCL/SDA, 3V3/GND, CS=3V3, main connector seating.");
    return;
  }

  uint8_t passCount = 0;
  for (uint8_t i = 0; i < IMU_COUNT; i++) {
    if (testOneImu(i)) {
      passCount++;
    }
  }

  VCP.print("SUMMARY: ");
  VCP.print(passCount);
  VCP.print("/");
  VCP.print(IMU_COUNT);
  VCP.println(" IMUs connected");

  if (passCount == IMU_COUNT) {
    VCP.println("RESULT: ALL_PASS");
  } else {
    VCP.println("RESULT: FAIL - fix the failed IMU TA0/power/GND/SCL/SDA/CS wiring first");
  }
}
