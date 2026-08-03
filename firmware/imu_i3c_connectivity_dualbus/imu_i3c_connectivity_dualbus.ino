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

static I3C_HandleTypeDef hi3c1;
static I3C_HandleTypeDef hi3c2;

struct BusCtx {
  const char *name;
  I3C_HandleTypeDef *handle;
  I3C_TypeDef *instance;
  bool ready;
  volatile bool done;
  volatile bool error;
};

static BusCtx busI3C2 = {"I3C2 D15/PB13 SCL + D14/PB14 SDA", &hi3c2, I3C2, false, false, false};
static BusCtx busI3C1 = {"I3C1 D11/PA7 SCL + A4/PC1 SDA", &hi3c1, I3C1, false, false, false};

static constexpr uint8_t IMU_COUNT = 5;
static constexpr uint8_t STATIC_ADDR_TA0_HIGH = 0x6B;
static constexpr uint8_t WHO_AM_I_REG = 0x0F;
static constexpr uint8_t WHO_EXPECTED = 0x73;

static constexpr uint8_t Broadcast_DISEC = 0x01;
static constexpr uint8_t Broadcast_RSTDAA = 0x06;
static constexpr uint8_t Direct_SETDASA = 0x87;

struct ImuSlot {
  const char *name;
  const char *ta0PinName;
  GPIO_TypeDef *ta0Port;
  uint16_t ta0Pin;
  uint8_t dynAddr;
  BusCtx *bus;
};

static ImuSlot imus[IMU_COUNT] = {
  {"IMU0", "D2/PC8", GPIOC, GPIO_PIN_8, 0x32, &busI3C2},
  {"IMU1", "D4/PB5", GPIOB, GPIO_PIN_5, 0x33, &busI3C2},
  {"IMU2", "D7/PA8", GPIOA, GPIO_PIN_8, 0x34, &busI3C2},
  {"IMU3", "D8/PC7", GPIOC, GPIO_PIN_7, 0x35, &busI3C1},
  {"IMU4", "D9/PC6", GPIOC, GPIO_PIN_6, 0x36, &busI3C1},
};

static uint32_t ctrlBuf[24];
static uint8_t txBuf[64];
static uint8_t rxBuf[8];

static void printHex8(uint8_t value);
static void initTa0Gpios();
static void selectOnlyTa0(uint8_t index);
static void setAllTa0Low();
static BusCtx *ctxFor(I3C_HandleTypeDef *hi3c);
static bool initI3C(BusCtx &bus);
static bool waitI3CReady(BusCtx &bus, uint32_t timeoutMs);
static bool resetDynamicAddresses(BusCtx &bus);
static bool setDynamicAddress(BusCtx &bus, uint8_t dynAddr);
static bool readReg(BusCtx &bus, uint8_t dynAddr, uint8_t reg, uint8_t *data, uint16_t len);
static bool testOneImu(uint8_t index);
static void runConnectivityTest();

extern "C" void HAL_I3C_MspInit(I3C_HandleTypeDef *hi3c)
{
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_EnablePullUpPullDownConfig();

  RCC_PeriphCLKInitTypeDef clk = {};
  GPIO_InitTypeDef gpio = {};
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  if (hi3c->Instance == I3C1) {
    HAL_PWREx_EnableI3CPullUp(PWR_I3CPU_PA7);  // D11 / PA7 / I3C1_SCL
    HAL_PWREx_EnableI3CPullUp(PWR_I3CPU_PC1);  // A4 / PC1 / I3C1_SDA

    clk.PeriphClockSelection = RCC_PERIPHCLK_I3C1;
    clk.I3c1ClockSelection = RCC_I3C1CLKSOURCE_PCLK1;
    HAL_RCCEx_PeriphCLKConfig(&clk);

    __HAL_RCC_I3C1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_7;
    gpio.Alternate = GPIO_AF6_I3C1;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_1;
    gpio.Alternate = GPIO_AF5_I3C1;
    HAL_GPIO_Init(GPIOC, &gpio);

    HAL_NVIC_SetPriority(I3C1_EV_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I3C1_EV_IRQn);
    HAL_NVIC_SetPriority(I3C1_ER_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I3C1_ER_IRQn);
    return;
  }

  if (hi3c->Instance == I3C2) {
    HAL_PWREx_EnableI3CPullUp(PWR_I3CPU_PB13);  // D15 / PB13 / I3C2_SCL
    HAL_PWREx_EnableI3CPullUp(PWR_I3CPU_PB14);  // D14 / PB14 / I3C2_SDA

    clk.PeriphClockSelection = RCC_PERIPHCLK_I3C2;
    clk.I3c2ClockSelection = RCC_I3C2CLKSOURCE_PCLK2;
    HAL_RCCEx_PeriphCLKConfig(&clk);

    __HAL_RCC_I3C2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_13 | GPIO_PIN_14;
    gpio.Alternate = GPIO_AF6_I3C2;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_NVIC_SetPriority(I3C2_EV_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I3C2_EV_IRQn);
    HAL_NVIC_SetPriority(I3C2_ER_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I3C2_ER_IRQn);
  }
}

extern "C" void I3C1_EV_IRQHandler(void)
{
  HAL_I3C_EV_IRQHandler(&hi3c1);
}

extern "C" void I3C1_ER_IRQHandler(void)
{
  HAL_I3C_ER_IRQHandler(&hi3c1);
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
  BusCtx *bus = ctxFor(hi3c);
  if (bus) {
    bus->done = true;
  }
}

extern "C" void HAL_I3C_CtrlRxCpltCallback(I3C_HandleTypeDef *hi3c)
{
  BusCtx *bus = ctxFor(hi3c);
  if (bus) {
    bus->done = true;
  }
}

extern "C" void HAL_I3C_CtrlMultipleXferCpltCallback(I3C_HandleTypeDef *hi3c)
{
  BusCtx *bus = ctxFor(hi3c);
  if (bus) {
    bus->done = true;
  }
}

extern "C" void HAL_I3C_ErrorCallback(I3C_HandleTypeDef *hi3c)
{
  BusCtx *bus = ctxFor(hi3c);
  if (bus) {
    bus->error = true;
    bus->done = true;
  }
}

void setup()
{
  VCP.begin(115200);
  delay(300);
  VCP.println();
  VCP.println("=== ISM6HG256X dual-bus 5-IMU I3C connectivity test ===");
  VCP.println("Bus A: IMU0-IMU2 on I3C2, SCL=D15/PB13, SDA=D14/PB14");
  VCP.println("Bus B: IMU3-IMU4 on I3C1, SCL=D11/PA7, SDA=A4/PC1");
  VCP.println("TA0: IMU0=D2, IMU1=D4, IMU2=D7, IMU3=D8, IMU4=D9");
  VCP.println("Each IMU: VDDIO+VDD=3V3, GND=GND, CS=3V3 recommended");
  VCP.println();

  initTa0Gpios();

  busI3C2.ready = initI3C(busI3C2);
  VCP.print("I3C2_INIT: ");
  VCP.println(busI3C2.ready ? "PASS" : "FAIL");

  busI3C1.ready = initI3C(busI3C1);
  VCP.print("I3C1_INIT: ");
  VCP.println(busI3C1.ready ? "PASS" : "FAIL");

  runConnectivityTest();
}

void loop()
{
  delay(3000);
  runConnectivityTest();
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

static BusCtx *ctxFor(I3C_HandleTypeDef *hi3c)
{
  if (hi3c->Instance == I3C1) {
    return &busI3C1;
  }
  if (hi3c->Instance == I3C2) {
    return &busI3C2;
  }
  return nullptr;
}

static bool initI3C(BusCtx &bus)
{
  I3C_FifoConfTypeDef fifo = {};
  I3C_CtrlConfTypeDef ctrl = {};
  I3C_HandleTypeDef *h = bus.handle;

  h->Instance = bus.instance;
  h->Mode = HAL_I3C_MODE_CONTROLLER;
  h->Init.CtrlBusCharacteristic.SDAHoldTime = HAL_I3C_SDA_HOLD_TIME_0_5;
  h->Init.CtrlBusCharacteristic.WaitTime = HAL_I3C_OWN_ACTIVITY_STATE_0;
  h->Init.CtrlBusCharacteristic.SCLPPLowDuration = 0x2F;
  h->Init.CtrlBusCharacteristic.SCLI3CHighDuration = 0x2F;
  h->Init.CtrlBusCharacteristic.SCLODLowDuration = 0x2F;
  h->Init.CtrlBusCharacteristic.SCLI2CHighDuration = 0x00;
  h->Init.CtrlBusCharacteristic.BusFreeDuration = 0x13;
  h->Init.CtrlBusCharacteristic.BusIdleDuration = 0x5E;

  if (HAL_I3C_Init(h) != HAL_OK) {
    return false;
  }

  fifo.RxFifoThreshold = HAL_I3C_RXFIFO_THRESHOLD_4_4;
  fifo.TxFifoThreshold = HAL_I3C_TXFIFO_THRESHOLD_1_4;
  fifo.ControlFifo = HAL_I3C_CONTROLFIFO_DISABLE;
  fifo.StatusFifo = HAL_I3C_STATUSFIFO_DISABLE;
  if (HAL_I3C_SetConfigFifo(h, &fifo) != HAL_OK) {
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
  return HAL_I3C_Ctrl_Config(h, &ctrl) == HAL_OK;
}

static bool waitI3CReady(BusCtx &bus, uint32_t timeoutMs)
{
  const uint32_t start = millis();
  while (!bus.done && !bus.error && (millis() - start < timeoutMs)) {
    yield();
  }
  return bus.done && !bus.error && (HAL_I3C_GetState(bus.handle) == HAL_I3C_STATE_READY);
}

static bool resetDynamicAddresses(BusCtx &bus)
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

  if (HAL_I3C_AddDescToFrame(bus.handle, broadcastCcc, NULL, &xfer, 2, I3C_BROADCAST_WITHOUT_DEFBYTE_RESTART) != HAL_OK) {
    return false;
  }

  bus.done = false;
  bus.error = false;
  if (HAL_I3C_Ctrl_TransmitCCC_IT(bus.handle, &xfer) != HAL_OK) {
    return false;
  }
  return waitI3CReady(bus, 1000);
}

static bool setDynamicAddress(BusCtx &bus, uint8_t dynAddr)
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

  if (HAL_I3C_AddDescToFrame(bus.handle, directCcc, NULL, &xfer, 1, I3C_DIRECT_WITHOUT_DEFBYTE_STOP) != HAL_OK) {
    return false;
  }

  bus.done = false;
  bus.error = false;
  if (HAL_I3C_Ctrl_TransmitCCC_IT(bus.handle, &xfer) != HAL_OK) {
    return false;
  }
  return waitI3CReady(bus, 1000);
}

static bool readReg(BusCtx &bus, uint8_t dynAddr, uint8_t reg, uint8_t *data, uint16_t len)
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

  if (HAL_I3C_AddDescToFrame(bus.handle, NULL, desc, &xfer, 2, I3C_PRIVATE_WITH_ARB_RESTART) != HAL_OK) {
    return false;
  }

  bus.done = false;
  bus.error = false;
  if (HAL_I3C_Ctrl_MultipleTransfer_IT(bus.handle, &xfer) != HAL_OK) {
    return false;
  }
  return waitI3CReady(bus, 1000);
}

static bool testOneImu(uint8_t index)
{
  ImuSlot &imu = imus[index];
  BusCtx &bus = *imu.bus;

  VCP.print(imu.name);
  VCP.print(" on ");
  VCP.print(bus.name);
  VCP.print(" TA0=");
  VCP.print(imu.ta0PinName);
  VCP.print(" static 0x6B -> dyn 0x");
  printHex8(imu.dynAddr);
  VCP.print(": ");

  if (!bus.ready) {
    VCP.println("BUS_INIT_FAIL");
    return false;
  }

  selectOnlyTa0(index);
  if (!setDynamicAddress(bus, imu.dynAddr)) {
    VCP.println("SETDASA_FAIL");
    setAllTa0Low();
    return false;
  }

  setAllTa0Low();

  if (HAL_I3C_Ctrl_IsDeviceI3C_Ready(bus.handle, imu.dynAddr, 300, 1000) != HAL_OK) {
    VCP.println("NO_ACK");
    return false;
  }

  rxBuf[0] = 0;
  if (!readReg(bus, imu.dynAddr, WHO_AM_I_REG, rxBuf, 1)) {
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
  VCP.println("--- isolated dual-bus 5-IMU connectivity scan ---");
  setAllTa0Low();

  uint8_t passCount = 0;

  for (uint8_t i = 0; i < IMU_COUNT; i++) {
    ImuSlot &imu = imus[i];
    BusCtx &bus = *imu.bus;

    VCP.print("Isolate ");
    VCP.print(imu.name);
    VCP.print(": reset ");
    VCP.print(bus.name);
    VCP.print(", then only TA0=");
    VCP.print(imu.ta0PinName);
    VCP.print(" high: ");

    if (!bus.ready) {
      VCP.println("BUS_INIT_FAIL");
      continue;
    }

    setAllTa0Low();
    if (!resetDynamicAddresses(bus)) {
      VCP.println("RSTDAA_FAIL");
      continue;
    }

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
    VCP.println("RESULT: FAIL - fix the failed IMU bus/TA0/power/GND/CS wiring first");
  }
}
