#include "mrd106.h"
#include "mrd106_helper.h"

static int s_mrd106_init();
static int s_mrd106_powerOn();
static int s_mrd106_powerOff();
static int s_mrd106_startMeasurement();
static int s_mrd106_stopMeasurement();
static int s_mrd106_enableSensors(int en);
static int s_mrd106_enableTimer(int tmr, int en);
static int s_mrd106_updateTimerPeriod(int tmr, int period, int tmr_ps);
static void s_mrd106_setBatteryPercentage(int battPer);
static int s_mrd106_getBatteryPercentage();
static void s_mrd106_setChargingStatus(bool en);
static bool s_mrd106_getChargingStatus();
static void s_mrd106_setStatusLedEnable(bool en);
static bool s_mrd106_getStatusLedEnable();
static void s_mrd106_setMeasurementEnable(bool en);
static bool s_mrd106_getMesaurementEnable();
static void s_mrd106_setDebugLogEnable(bool en);
static bool s_mrd106_getDebugLogEnable();
static void s_mrd106_setFlashLogEnable(bool en);
static bool s_mrd106_getFlashLogEnable();
static void s_mrd106_setTempSensorEnable(bool en);
static bool s_mrd106_getTempSensorEnable();
static void s_mrd106_setTempSensorFreq(int ind);
static int s_mrd106_getTempSensorFreq();
static void s_mrd106_setBleSentEnable(bool en);
static bool s_mrd106_getBleSentEnable();
static void s_mrd106_setBleSentEvt(int evt);
static int s_mrd106_getBleSentEvt();
static void s_mrd106_setStopCmd(int cmd);
static int s_mrd106_getStopCmd();
static sensor_t* s_mrd106_getSensor(int sensorNo);

static mrd_t mrd106 = {
    .init = s_mrd106_init,
    .powerOn = s_mrd106_powerOn,
    .powerOff = s_mrd106_powerOff,
    .startMeasurement = s_mrd106_startMeasurement,
    .stopMeasurement = s_mrd106_stopMeasurement,
    .enableSensors = s_mrd106_enableSensors,
    .enableTimer = s_mrd106_enableTimer,
    .updateTimerPeriod = s_mrd106_updateTimerPeriod,

    .setBatteryPercentage = s_mrd106_setBatteryPercentage,
    .getBatteryPercentage = s_mrd106_getBatteryPercentage,
    .setChargingStatus = s_mrd106_setChargingStatus,
    .getChargingStatus = s_mrd106_getChargingStatus,
    .setStatusLedEnable = s_mrd106_setStatusLedEnable,
    .getStatusLedEnable = s_mrd106_getStatusLedEnable,
    .setMeasurementEnable = s_mrd106_setMeasurementEnable,
    .getMeasurementEnable = s_mrd106_getMesaurementEnable,
    .setDebugLogEnable = s_mrd106_setDebugLogEnable,
    .getDebugLogEnable = s_mrd106_getDebugLogEnable,
    .setFlashLogEnable = s_mrd106_setFlashLogEnable,
    .getFlashLogEnable = s_mrd106_getFlashLogEnable,
    .setTempSensorEnable = s_mrd106_setTempSensorEnable,
    .getTempSensorEnable = s_mrd106_getTempSensorEnable,
    .setTempSensorFreq = s_mrd106_setTempSensorFreq,
    .getTempSensorFreq = s_mrd106_getTempSensorFreq,
    .setBleSentEnable = s_mrd106_setBleSentEnable,
    .getBleSentEnable = s_mrd106_getBleSentEnable,
    .setBleSentEvt = s_mrd106_setBleSentEvt,
    .getBleSentEvt = s_mrd106_getBleSentEvt,
    .setStopCmd = s_mrd106_setStopCmd,
    .getStopCmd = s_mrd106_getStopCmd,

    .getSensor = s_mrd106_getSensor,
};


static int s_mrd106_init()
{
    mrd106.p_isInit = 1;
    mrd_ble_init();
    mrd_comm_init();
    mrd_peripheral_init();
    mrd_api_init();
    mrd_timers_init();
    mrd_isr_init();
    mrd_sensor_init(&mrd106.sensors[SH_ACC_SENSOR], SH_ACC_SENSOR);
    mrd_sensor_init(&mrd106.sensors[SH_MAX86178], SH_MAX86178);
    mrd_sensor_init(&mrd106.sensors[SH_TEMP_SENSOR], SH_TEMP_SENSOR);

    return 0;
}

static int s_mrd106_powerOn()
{
    return 0;
}
static int s_mrd106_powerOff()
{
    return 0;
}
static int s_mrd106_startMeasurement()
{
    return 0;
}
static int s_mrd106_stopMeasurement()
{
    return 0;
}

static int s_mrd106_enableSensors(int en)
{
    mrd_sensors_enable(en);
    return 0;
}

static int s_mrd106_enableTimer(int tmr, int en)
{

    mrd_timer_enable(tmr, en);

    return 0;
}

static int s_mrd106_updateTimerPeriod(int tmr, int period, int tmr_ps)
{
    mrd_timer_period_update(tmr, period, tmr_ps);

    return 0;
}

static void s_mrd106_setBatteryPercentage(int battPer)
{
    mrd106.p_batteryPercentage = battPer;
}

static int s_mrd106_getBatteryPercentage()
{
    return mrd106.p_batteryPercentage;
}

static void s_mrd106_setChargingStatus(bool en)
{
    mrd106.p_chargingStatus = en;
}

static bool s_mrd106_getChargingStatus()
{
    return mrd106.p_chargingStatus;
}

static void s_mrd106_setStatusLedEnable(bool en)
{
    mrd106.p_statusLedEnable = en;
}

static bool s_mrd106_getStatusLedEnable()
{
    return mrd106.p_statusLedEnable;
}

static void s_mrd106_setMeasurementEnable(bool en)
{
    mrd106.p_measurementEnable = en;
}

static bool s_mrd106_getMesaurementEnable()
{
    return mrd106.p_measurementEnable;
}

static void s_mrd106_setDebugLogEnable(bool en)
{

}

static bool s_mrd106_getDebugLogEnable()
{
    return true;
}

static void s_mrd106_setFlashLogEnable(bool en)
{
    mrd106.p_flashLogEnable = en;
}

static bool s_mrd106_getFlashLogEnable()
{
    return mrd106.p_flashLogEnable;
}

static void s_mrd106_setTempSensorEnable(bool en)
{
    mrd106.p_tempSensorEnable = en;
}

static bool s_mrd106_getTempSensorEnable()
{
    return mrd106.p_tempSensorEnable;
}

static void s_mrd106_setTempSensorFreq(int ind)
{
    mrd106.p_tempSensorFreq = ind;
}

static int s_mrd106_getTempSensorFreq()
{
    return mrd106.p_tempSensorFreq;
}

static void s_mrd106_setBleSentEnable(bool en)
{
    mrd106.p_bleSentEnable = en;
}

static bool s_mrd106_getBleSentEnable()
{
    return mrd106.p_bleSentEnable;
}

static void s_mrd106_setBleSentEvt(int evt)
{
    mrd106.e_bleSentEvt = evt;
}

static int s_mrd106_getBleSentEvt()
{
    return mrd106.e_bleSentEvt;
}

static void s_mrd106_setStopCmd(int cmd)
{
    mrd106.p_stopCmd = cmd;
}

static int s_mrd106_getStopCmd()
{
    return mrd106.p_stopCmd;
}

static sensor_t* s_mrd106_getSensor(int sensorNo)
{
    return mrd106.sensors[sensorNo];
}

mrd_t* getMRD()
{
    return &mrd106;
}