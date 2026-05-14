#include "Telemetry.h"
#include "Filters.h"

static TelemetryData store;
static portMUX_TYPE telemetryMux = portMUX_INITIALIZER_UNLOCKED;

// Externs from main.cpp; consolidated into SharedState.h in a later phase.
extern portMUX_TYPE stateMux;
extern volatile bool motorsArmed;
extern volatile FlightMode flightMode;
extern volatile bool rcConnected;
extern volatile int rcArmSwitchVal;
extern volatile int rcModeSwitchVal;
extern GyroFilterBank gyroFilter;

namespace Telemetry {

void publish(const TelemetryData& s) {
    portENTER_CRITICAL(&telemetryMux);
    store = s;
    portEXIT_CRITICAL(&telemetryMux);
}

void snapshot(TelemetryData& out) {
    portENTER_CRITICAL(&telemetryMux);
    out = store;
    portEXIT_CRITICAL(&telemetryMux);
}

String buildJson() {
    TelemetryData s;
    snapshot(s);

    bool armed, rc;
    FlightMode mode;
    int armSw, modeSw;

    portENTER_CRITICAL(&stateMux);
    armed = motorsArmed;
    mode = flightMode;
    rc = rcConnected;
    armSw = rcArmSwitchVal;
    modeSw = rcModeSwitchVal;
    portEXIT_CRITICAL(&stateMux);

    // Short keys to cut bandwidth on the ESP32 web server
    String json = "{";

    json += "\"r\":" + String(s.roll, 1);
    json += ",\"p\":" + String(s.pitch, 1);
    json += ",\"y\":" + String(s.yaw, 1);
    json += ",\"ae\":[" + String(s.angleErrRoll, 1) + "," + String(s.angleErrPitch, 1) + "]";

    json += ",\"rc\":[" + String(s.rcRoll, 3) + "," + String(s.rcPitch, 3) + "," +
            String(s.rcYaw, 3) + "," + String(s.rcThrottle) + "]";
    json += ",\"rcs\":[" + String(armSw) + "," + String(modeSw) + "]";

    json += ",\"gr\":[" + String(s.gyroX, 1) + "," + String(s.gyroY, 1) + "," + String(s.gyroZ, 1) + "]";
    json += ",\"gf\":[" + String(s.gyroXf, 1) + "," + String(s.gyroYf, 1) + "," + String(s.gyroZf, 1) + "]";

    json += ",\"rs\":[" + String(s.rateSetRoll, 1) + "," + String(s.rateSetPitch, 1) + "," + String(s.rateSetYaw, 1) + "]";
    json += ",\"re\":[" + String(s.rateErrRoll, 1) + "," + String(s.rateErrPitch, 1) + "," + String(s.rateErrYaw, 1) + "]";

    json += ",\"pp\":[" + String(s.pTermRoll, 0) + "," + String(s.pTermPitch, 0) + "," + String(s.pTermYaw, 0) + "]";
    json += ",\"pi\":[" + String(s.iTermRoll, 0) + "," + String(s.iTermPitch, 0) + "," + String(s.iTermYaw, 0) + "]";
    json += ",\"pd\":[" + String(s.dTermRoll, 0) + "," + String(s.dTermPitch, 0) + "," + String(s.dTermYaw, 0) + "]";
    json += ",\"pf\":[" + String(s.fTermRoll, 0) + "," + String(s.fTermPitch, 0) + "," + String(s.fTermYaw, 0) + "]";
    json += ",\"pt\":[" + String(s.pidTotalRoll, 1) + "," + String(s.pidTotalPitch, 1) + "," + String(s.pidTotalYaw, 1) + "]";

    json += ",\"thr\":" + String(s.throttle);
    json += ",\"m1\":" + String(s.m1);
    json += ",\"m2\":" + String(s.m2);
    json += ",\"m3\":" + String(s.m3);
    json += ",\"m4\":" + String(s.m4);
    json += ",\"mix\":[" + String(s.mixRoll, 1) + "," + String(s.mixPitch, 1) + "," + String(s.mixYaw, 1) + "]";
    json += ",\"sat\":" + String(s.motorSaturated ? "\"SAT\"" : "\"None\"");

    json += ",\"acc\":[" + String(s.accelX, 2) + "," + String(s.accelY, 2) + "," + String(s.accelZ, 2) + "]";
    json += ",\"mag\":[" + String(s.magX, 1) + "," + String(s.magY, 1) + "," + String(s.magZ, 1) + "]";

    json += ",\"armed\":" + String(armed ? "true" : "false");
    json += ",\"mode\":" + String((int)mode);
    json += ",\"rcOk\":" + String(rc ? "true" : "false");
    json += ",\"conv\":" + String(s.fusionConverged ? "true" : "false");
    json += ",\"accOk\":true";
    json += ",\"magOk\":" + String(s.magValid ? "true" : "false");
    json += ",\"air\":" + String(s.airmodeActive ? "true" : "false");
    json += ",\"gfOn\":" + String(gyroFilter.isEnabled() ? "true" : "false");

    json += ",\"tpa\":" + String(s.tpaFactor, 2);
    json += ",\"trim\":[" + String(s.trimRoll, 2) + "," + String(s.trimPitch, 2) + "]";

    json += ",\"hl\":" + String(s.headlessActive ? "true" : "false");
    json += ",\"hlRef\":" + String(s.headlessRefYaw, 1);

    json += ",\"lp\":" + String(s.loopTime);
    json += ",\"ls\":[" + String(s.loopTime) + "," + String(s.loopMin) + "," +
            String(s.loopMax) + "," + String(s.loopAvg) + "," +
            String(s.loopJitter) + "," + String(s.overruns) + "]";
    json += ",\"up\":" + String(millis() / 1000);

    json += "}";

    return json;
}

}  // namespace Telemetry
