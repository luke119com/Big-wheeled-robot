#include "wifi_manager.h"

#include "device_tuning.h"
#include "PS2.h"
#include "pid.h"
#include "robot.h"

#include <DNSServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>

namespace
{
constexpr const char *kPrefsNamespace = "wifi";
constexpr const char *kPrefsSsidKey = "ssid";
constexpr const char *kPrefsPassKey = "password";
constexpr const char *kPrefsNameKey = "device_name";
constexpr const char *kPrefsLeftYKey = "left_y";
constexpr const char *kPrefsRightYKey = "right_y";
constexpr const char *kPortalApPassword = "87654321";
constexpr uint32_t kConnectTimeoutMs = 15000;
constexpr uint16_t kDnsPort = 53;
constexpr int kMaxScanResults = 12;
constexpr size_t kMaxDisplayNameLen = 20;
constexpr size_t kMaxHostLabelLen = 24;
constexpr size_t kMaxApSsidLen = 32;

Preferences g_preferences;
WebServer g_server(80);
DNSServer g_dnsServer;

float readArgFloat(const String &name, float fallback)
{
  if (!g_server.hasArg(name))
  {
    return fallback;
  }

  String raw = g_server.arg(name);
  raw.trim();
  if (raw.isEmpty())
  {
    return fallback;
  }

  return raw.toFloat();
}

int readArgInt(const String &name, int fallback)
{
  if (!g_server.hasArg(name))
  {
    return fallback;
  }

  String raw = g_server.arg(name);
  raw.trim();
  if (raw.isEmpty())
  {
    return fallback;
  }

  return raw.toInt();
}

String buildProfileKey(const char *prefix, int index)
{
  char key[12];
  snprintf(key, sizeof(key), "%s%02d", prefix, index);
  return String(key);
}
}

WiFiManagerPortal wifiManagerPortal;

void WiFiManagerPortal::begin()
{
  WiFi.persistent(false);
  WiFi.setSleep(false);

  initIdentity();
  setupRoutes();
  loadPreferences();
  loadTuningConfig();
  refreshIdentity();
  cachedNetworkOptions_ = "<option value=''>Press Rescan Wi-Fi to scan nearby networks</option>";

  if (!savedSsid_.isEmpty() && connectToSavedNetwork())
  {
    return;
  }

  startAccessPoint();
  statusMessage_ = savedSsid_.isEmpty()
                       ? "No Wi-Fi credentials saved. AP portal started."
                       : "Wi-Fi connect failed. AP portal started.";
}

void WiFiManagerPortal::loop()
{
  if (dnsActive_)
  {
    g_dnsServer.processNextRequest();
  }

  g_server.handleClient();

  if (pendingCredentials_)
  {
    processPendingCredentials();
  }
}

void WiFiManagerPortal::setupRoutes()
{
  g_server.on("/", HTTP_GET, [this]() { handleRoot(); });
  g_server.on("/api/status", HTTP_GET, [this]() { handleStatusApi(); });
  g_server.on("/api/height", HTTP_POST, [this]() { handleHeightSet(); });
  g_server.on("/save", HTTP_POST, [this]() { handleSave(); });
  g_server.on("/tuning", HTTP_POST, [this]() { handleTuningSave(); });
  g_server.on("/forget", HTTP_POST, [this]() { handleForget(); });
  g_server.on("/rescan", HTTP_POST, [this]() { handleRescan(); });
  g_server.onNotFound([this]() { handleNotFound(); });
}

void WiFiManagerPortal::loadPreferences()
{
  if (!g_preferences.begin(kPrefsNamespace, false))
  {
    preferencesReady_ = false;
    statusMessage_ = "Failed to open NVS. Wi-Fi settings will not persist.";
    savedSsid_ = "";
    savedPassword_ = "";
    savedDeviceName_ = "";
    return;
  }

  preferencesReady_ = true;
  savedSsid_ = g_preferences.isKey(kPrefsSsidKey) ? g_preferences.getString(kPrefsSsidKey, "") : "";
  savedPassword_ = g_preferences.isKey(kPrefsPassKey) ? g_preferences.getString(kPrefsPassKey, "") : "";
  savedDeviceName_ = g_preferences.isKey(kPrefsNameKey) ? g_preferences.getString(kPrefsNameKey, "") : "";
}

void WiFiManagerPortal::loadTuningConfig()
{
  DeviceTuningConfig config = getDeviceTuningConfig();
  if (!preferencesReady_)
  {
    applyDeviceTuningConfig(config);
    return;
  }
  config.leftHeight = g_preferences.isKey(kPrefsLeftYKey) ? g_preferences.getFloat(kPrefsLeftYKey, config.leftHeight) : config.leftHeight;
  config.rightHeight = g_preferences.isKey(kPrefsRightYKey) ? g_preferences.getFloat(kPrefsRightYKey, config.rightHeight) : config.rightHeight;

  for (int i = 0; i < kHeightProfileCount; ++i)
  {
    HeightProfile &profile = config.profiles[i];
    const String balancePointKey = buildProfileKey("bp", i);
    const String velKpKey = buildProfileKey("vk", i);
    const String balanceKpKey = buildProfileKey("bk", i);
    const String balanceKdKey = buildProfileKey("bd", i);
    const String balanceKiKey = buildProfileKey("bi", i);
    const String robotKpKey = buildProfileKey("rk", i);
    const String speedLimitKey = buildProfileKey("sl", i);

    profile.balancePoint = g_preferences.isKey(balancePointKey.c_str()) ? g_preferences.getFloat(balancePointKey.c_str(), profile.balancePoint) : profile.balancePoint;
    profile.velKp = g_preferences.isKey(velKpKey.c_str()) ? g_preferences.getFloat(velKpKey.c_str(), profile.velKp) : profile.velKp;
    profile.balanceKp = g_preferences.isKey(balanceKpKey.c_str()) ? g_preferences.getFloat(balanceKpKey.c_str(), profile.balanceKp) : profile.balanceKp;
    profile.balanceKd = g_preferences.isKey(balanceKdKey.c_str()) ? g_preferences.getFloat(balanceKdKey.c_str(), profile.balanceKd) : profile.balanceKd;
    profile.balanceKi = g_preferences.isKey(balanceKiKey.c_str()) ? g_preferences.getFloat(balanceKiKey.c_str(), profile.balanceKi) : profile.balanceKi;
    profile.robotKp = g_preferences.isKey(robotKpKey.c_str()) ? g_preferences.getFloat(robotKpKey.c_str(), profile.robotKp) : profile.robotKp;
    profile.speedLimit = g_preferences.isKey(speedLimitKey.c_str()) ? g_preferences.getInt(speedLimitKey.c_str(), profile.speedLimit) : profile.speedLimit;
  }

  applyDeviceTuningConfig(config);
}

void WiFiManagerPortal::saveCredentials(const String &ssid, const String &password)
{
  savedSsid_ = ssid;
  savedPassword_ = password;
  if (preferencesReady_)
  {
    g_preferences.putString(kPrefsSsidKey, ssid);
    g_preferences.putString(kPrefsPassKey, password);
  }
}

void WiFiManagerPortal::saveDeviceName(const String &deviceName)
{
  savedDeviceName_ = deviceName;
  if (preferencesReady_)
  {
    g_preferences.putString(kPrefsNameKey, deviceName);
  }
  refreshIdentity();
}

void WiFiManagerPortal::saveTuningConfig()
{
  if (!preferencesReady_)
  {
    return;
  }
  const DeviceTuningConfig config = getDeviceTuningConfig();
  g_preferences.putFloat(kPrefsLeftYKey, config.leftHeight);
  g_preferences.putFloat(kPrefsRightYKey, config.rightHeight);
  for (int i = 0; i < kHeightProfileCount; ++i)
  {
    const HeightProfile &profile = config.profiles[i];
    const String balancePointKey = buildProfileKey("bp", i);
    const String velKpKey = buildProfileKey("vk", i);
    const String balanceKpKey = buildProfileKey("bk", i);
    const String balanceKdKey = buildProfileKey("bd", i);
    const String balanceKiKey = buildProfileKey("bi", i);
    const String robotKpKey = buildProfileKey("rk", i);
    const String speedLimitKey = buildProfileKey("sl", i);

    g_preferences.putFloat(balancePointKey.c_str(), profile.balancePoint);
    g_preferences.putFloat(velKpKey.c_str(), profile.velKp);
    g_preferences.putFloat(balanceKpKey.c_str(), profile.balanceKp);
    g_preferences.putFloat(balanceKdKey.c_str(), profile.balanceKd);
    g_preferences.putFloat(balanceKiKey.c_str(), profile.balanceKi);
    g_preferences.putFloat(robotKpKey.c_str(), profile.robotKp);
    g_preferences.putInt(speedLimitKey.c_str(), profile.speedLimit);
  }
}

void WiFiManagerPortal::clearCredentials()
{
  savedSsid_ = "";
  savedPassword_ = "";
  if (preferencesReady_)
  {
    g_preferences.remove(kPrefsSsidKey);
    g_preferences.remove(kPrefsPassKey);
  }
}

void WiFiManagerPortal::initIdentity()
{
  const uint64_t chipId = ESP.getEfuseMac();
  char suffix[7];
  snprintf(suffix, sizeof(suffix), "%06llX", chipId & 0xFFFFFFULL);
  chipSuffix_ = String(suffix);

  uint8_t mac[6];
  for (int i = 0; i < 6; ++i)
  {
    mac[i] = static_cast<uint8_t>((chipId >> (8 * (5 - i))) & 0xFF);
  }

  char macBuffer[18];
  snprintf(macBuffer,
           sizeof(macBuffer),
           "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0],
           mac[1],
           mac[2],
           mac[3],
           mac[4],
           mac[5]);
  macAddress_ = String(macBuffer);
}

void WiFiManagerPortal::refreshIdentity()
{
  const String cleanDisplay = sanitizeDisplayName(savedDeviceName_);
  displayName_ = cleanDisplay.isEmpty() ? "SF Wheel " + chipSuffix_ : cleanDisplay + " " + chipSuffix_;

  const String hostBase = sanitizeHostLabel(savedDeviceName_);
  mdnsHostname_ = hostBase.isEmpty() ? "sf-wheel-" + chipSuffix_ : hostBase + "-" + chipSuffix_;
  mdnsHostname_.toLowerCase();
  mdnsHostname_ = trimToLength(mdnsHostname_, 63);

  const String apBase = cleanDisplay.isEmpty() ? "SF_Wheel" : cleanDisplay;
  String apName = apBase;
  apName.replace(" ", "_");
  apName.replace("-", "_");
  apSsid_ = apName + "_" + chipSuffix_;
  apSsid_ = trimToLength(apSsid_, kMaxApSsidLen);
}

bool WiFiManagerPortal::connectToSavedNetwork()
{
  return connectToNetwork(savedSsid_, savedPassword_, false);
}

bool WiFiManagerPortal::connectToNetwork(const String &ssid, const String &password, bool keepApAlive)
{
  if (ssid.isEmpty())
  {
    return false;
  }

  if (keepApAlive)
  {
    WiFi.mode(WIFI_AP_STA);
  }
  else
  {
    stopAccessPoint();
    WiFi.mode(WIFI_STA);
  }

  WiFi.disconnect(false, false);
  delay(100);

  Serial.printf("Connecting to Wi-Fi SSID: %s\r\n", ssid.c_str());
  WiFi.begin(ssid.c_str(), password.c_str());

  const uint32_t startMs = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startMs < kConnectTimeoutMs)
  {
    delay(250);
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Wi-Fi connection failed.");
    stopMdns();
    if (!keepApAlive)
    {
      WiFi.disconnect(false, false);
    }
    return false;
  }

  statusMessage_ = "Wi-Fi connected.";
  Serial.printf("Wi-Fi connected. IP: %s\r\n", WiFi.localIP().toString().c_str());

  if (keepApAlive)
  {
    stopAccessPoint();
  }

  ensureWebServerRunning();
  startMdns();
  return true;
}

void WiFiManagerPortal::startAccessPoint()
{
  stopMdns();

  WiFi.mode(WIFI_AP_STA);

  const IPAddress apIp(192, 168, 4, 1);
  const IPAddress gateway(192, 168, 4, 1);
  const IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(apIp, gateway, subnet);
  WiFi.softAP(apSsid_.c_str(), kPortalApPassword);

  if (!dnsActive_)
  {
    g_dnsServer.start(kDnsPort, "*", WiFi.softAPIP());
    dnsActive_ = true;
  }

  apModeActive_ = true;
  ensureWebServerRunning();

  Serial.printf("AP mode enabled. SSID: %s, password: %s, IP: %s\r\n",
                apSsid_.c_str(),
                kPortalApPassword,
                WiFi.softAPIP().toString().c_str());
}

void WiFiManagerPortal::stopAccessPoint()
{
  if (dnsActive_)
  {
    g_dnsServer.stop();
    dnsActive_ = false;
  }

  if (apModeActive_)
  {
    WiFi.softAPdisconnect(true);
    apModeActive_ = false;
  }
}

void WiFiManagerPortal::ensureWebServerRunning()
{
  if (serverStarted_)
  {
    return;
  }

  g_server.begin();
  serverStarted_ = true;
}

void WiFiManagerPortal::startMdns()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    return;
  }

  stopMdns();

  if (!MDNS.begin(mdnsHostname_.c_str()))
  {
    Serial.println("mDNS start failed.");
    return;
  }

  MDNS.addService("http", "tcp", 80);
  mdnsActive_ = true;
  Serial.printf("mDNS started: http://%s.local\r\n", mdnsHostname_.c_str());
}

void WiFiManagerPortal::stopMdns()
{
  if (!mdnsActive_)
  {
    return;
  }

  MDNS.end();
  mdnsActive_ = false;
}

void WiFiManagerPortal::handleRoot()
{
  g_server.send(200, "text/html; charset=utf-8", buildRootPage());
}

void WiFiManagerPortal::handleSave()
{
  const String ssid = g_server.arg("ssid");
  const String password = g_server.arg("password");
  const String deviceName = sanitizeDisplayName(g_server.arg("device_name"));

  if (ssid.isEmpty())
  {
    statusMessage_ = "SSID is required.";
    g_server.send(400, "text/html; charset=utf-8", buildRootPage());
    return;
  }

  pendingSsid_ = ssid;
  pendingPassword_ = password;
  pendingDeviceName_ = deviceName;
  pendingCredentials_ = true;
  statusMessage_ = "Received new settings. Trying to connect now.";

  g_server.send(200,
                "text/html; charset=utf-8",
                "<!DOCTYPE html><html><head><meta charset='utf-8'>"
                "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                "<title>Applying</title></head><body><h1>Applying Settings</h1>"
                "<p>The device is trying to connect with the new configuration.</p>"
                "<p>If you are on the AP portal, it will disconnect after a successful join.</p>"
                "<p>Try reconnecting with one of these links after the device joins Wi-Fi:</p>"
                "<p><a href='http://" + getHostUrl() + "'>http://" + getHostUrl() + "</a></p>"
                "<p><a href='http://" + getStaUrl() + "'>http://" + getStaUrl() + "</a></p>"
                "<p><a href='/'>Back</a></p></body></html>");
}

void WiFiManagerPortal::handleForget()
{
  clearCredentials();
  statusMessage_ = "Stored Wi-Fi settings cleared. AP portal is active.";
  WiFi.disconnect(false, false);
  startAccessPoint();
  cachedNetworkOptions_ = "<option value=''>Press Rescan Wi-Fi to scan nearby networks</option>";
  g_server.send(200, "text/html; charset=utf-8", buildRootPage());
}

void WiFiManagerPortal::handleRescan()
{
  refreshNetworkOptions();
  if (cachedNetworkOptions_.indexOf("failed") >= 0)
  {
    statusMessage_ = "Wi-Fi scan failed.";
  }
  else
  {
    statusMessage_ = "Wi-Fi scan finished. Nearby list updated.";
  }
  g_server.send(200, "text/html; charset=utf-8", buildRootPage());
}

void WiFiManagerPortal::handleTuningSave()
{
  DeviceTuningConfig config = getDeviceTuningConfig();
  config.leftHeight = readArgFloat("left_height", config.leftHeight);
  config.rightHeight = readArgFloat("right_height", config.rightHeight);
  for (int i = 0; i < kHeightProfileCount; ++i)
  {
    HeightProfile &profile = config.profiles[i];
    profile.balancePoint = readArgFloat("balance_point_" + String(i), profile.balancePoint);
    profile.velKp = readArgFloat("vel_kp_" + String(i), profile.velKp);
    profile.balanceKp = readArgFloat("balance_kp_" + String(i), profile.balanceKp);
    profile.balanceKd = readArgFloat("balance_kd_" + String(i), profile.balanceKd);
    profile.balanceKi = readArgFloat("balance_ki_" + String(i), profile.balanceKi);
    profile.robotKp = readArgFloat("robot_kp_" + String(i), profile.robotKp);
    profile.speedLimit = readArgInt("speed_limit_" + String(i), profile.speedLimit);
  }

  applyDeviceTuningConfig(config);
  saveTuningConfig();
  statusMessage_ = "Height profiles updated and saved.";
  g_server.send(200, "text/html; charset=utf-8", buildRootPage());
}

void WiFiManagerPortal::handleHeightSet()
{
  ZeparamremoteValue = clampHeightOffset(readArgInt("offset", ZeparamremoteValue));
  updateBalanceOffsetByCurrentHeight();
  statusMessage_ = "Live height offset updated from web slider.";
  g_server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  g_server.send(200, "application/json; charset=utf-8", buildStatusJson());
}

void WiFiManagerPortal::handleStatusApi()
{
  g_server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  g_server.send(200, "application/json; charset=utf-8", buildStatusJson());
}

void WiFiManagerPortal::handleNotFound()
{
  if (apModeActive_)
  {
    g_server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
    g_server.send(302, "text/plain", "");
    return;
  }

  g_server.send(404, "text/plain; charset=utf-8", "Not found");
}

void WiFiManagerPortal::processPendingCredentials()
{
  pendingCredentials_ = false;
  saveDeviceName(pendingDeviceName_);
  saveCredentials(pendingSsid_, pendingPassword_);

  if (connectToNetwork(savedSsid_, savedPassword_, true))
  {
    statusMessage_ = "New Wi-Fi settings applied.";
    return;
  }

  startAccessPoint();
  statusMessage_ = "New Wi-Fi connect failed. Returned to AP portal.";
}

void WiFiManagerPortal::refreshNetworkOptions()
{
  cachedNetworkOptions_ = "<option value=''>Scanning Wi-Fi...</option>";
  const int networkCount = WiFi.scanNetworks(false, true);
  if (networkCount < 0)
  {
    cachedNetworkOptions_ = "<option value=''>Wi-Fi scan failed</option>";
    WiFi.scanDelete();
    return;
  }

  String options = "<option value=''>Select a scanned Wi-Fi network</option>";
  if (networkCount == 0)
  {
    options += "<option value=''>No Wi-Fi found</option>";
  }
  else
  {
    const int resultCount = networkCount < kMaxScanResults ? networkCount : kMaxScanResults;
    for (int i = 0; i < resultCount; ++i)
    {
      const String ssid = WiFi.SSID(i);
      options += "<option value='" + htmlEscape(ssid) + "'>";
      options += htmlEscape(ssid);
      options += " (";
      options += String(WiFi.RSSI(i));
      options += " dBm)";
      options += "</option>";
    }
  }
  cachedNetworkOptions_ = options;
  WiFi.scanDelete();
}

String WiFiManagerPortal::buildStatusJson() const
{
  const DeviceTuningConfig config = getDeviceTuningConfig();
  const int activeIndex = getHeightProfileIndexForOffset(ZeparamremoteValue);
  const HeightProfile activeProfile = getHeightProfileByIndex(activeIndex);

  String json;
  json.reserve(4200);
  json += "{";
  json += "\"mode\":\"" + jsonEscape(getModeLabel()) + "\",";
  json += "\"status\":\"" + jsonEscape(statusMessage_) + "\",";
  json += "\"network\":\"" + jsonEscape(getConnectionDetails()) + "\",";
  json += "\"display_name\":\"" + jsonEscape(displayName_) + "\",";
  json += "\"imu\":{";
  json += "\"acc_x\":" + String(accX, 4) + ",";
  json += "\"acc_y\":" + String(accY, 4) + ",";
  json += "\"acc_z\":" + String(accZ, 4) + ",";
  json += "\"roll\":" + String(roll, 3) + ",";
  json += "\"pitch\":" + String(pitch, 3) + ",";
  json += "\"yaw\":" + String(yaw, 3) + ",";
  json += "\"gyro_x\":" + String(gyroX, 3) + ",";
  json += "\"gyro_y\":" + String(gyroY, 3) + ",";
  json += "\"gyro_z\":" + String(gyroZ, 3) + ",";
  json += "\"temp_c\":" + String(imuTemp, 2);
  json += "},";
  json += "\"control\":{";
  json += "\"vel_kp\":" + String(vel_kp, 4) + ",";
  json += "\"balance_kp\":" + String(balance_kp, 4) + ",";
  json += "\"balance_kd\":" + String(balance_kd, 4) + ",";
  json += "\"balance_ki\":" + String(balance_ki, 4) + ",";
  json += "\"robot_kp\":" + String(robot_kp, 4) + ",";
  json += "\"speed_limit\":" + String(speed_limit) + ",";
  json += "\"wheel_left_target\":" + String(wheel_motor1_target, 3) + ",";
  json += "\"wheel_right_target\":" + String(wheel_motor2_target, 3) + ",";
  json += "\"wheel_left_velocity\":" + String(motor1_vel, 3) + ",";
  json += "\"wheel_right_velocity\":" + String(motor2_vel, 3);
  json += "},";
  json += "\"tuning\":{";
  json += "\"current_height\":" + String(getCurrentHeightForPitchControl(), 3) + ",";
  json += "\"current_pitch_target\":" + String(getCurrentPitchTarget(), 3) + ",";
  json += "\"balance_offset\":" + String(balance_offset, 3) + ",";
  json += "\"height_offset\":" + String(static_cast<float>(clampHeightOffset(ZeparamremoteValue)), 3) + ",";
  json += "\"active_step\":" + String(activeIndex) + ",";
  json += "\"active_offset\":" + String(activeProfile.offset) + ",";
  json += "\"left_height\":" + String(leftY, 3) + ",";
  json += "\"right_height\":" + String(rightY, 3) + ",";
  json += "\"left_target_y\":" + String(Y1, 3) + ",";
  json += "\"right_target_y\":" + String(y2, 3);
  json += "},";
  json += "\"ik\":{";
  json += "\"serial_mode\":" + String(serialFootPoseMode ? "true" : "false") + ",";
  json += "\"ps2_connected\":" + String(ps2IsConnected() ? "true" : "false") + ",";
  json += "\"left_x\":" + String(x1, 3) + ",";
  json += "\"right_x\":" + String(x2, 3) + ",";
  json += "\"serial_left_y\":" + String(serialLeftTargetY, 3) + ",";
  json += "\"serial_right_y\":" + String(serialRightTargetY, 3);
  json += "},";
  json += "\"profiles\":[";
  for (int i = 0; i < kHeightProfileCount; ++i)
  {
    const HeightProfile profile = config.profiles[i];
    if (i > 0)
    {
      json += ",";
    }
    json += "{";
    json += "\"index\":" + String(i) + ",";
    json += "\"offset\":" + String(profile.offset) + ",";
    json += "\"balance_point\":" + String(profile.balancePoint, 3) + ",";
    json += "\"vel_kp\":" + String(profile.velKp, 4) + ",";
    json += "\"balance_kp\":" + String(profile.balanceKp, 4) + ",";
    json += "\"balance_kd\":" + String(profile.balanceKd, 4) + ",";
    json += "\"balance_ki\":" + String(profile.balanceKi, 4) + ",";
    json += "\"robot_kp\":" + String(profile.robotKp, 4) + ",";
    json += "\"speed_limit\":" + String(profile.speedLimit);
    json += "}";
  }
  json += "]}";
  return json;
}

String WiFiManagerPortal::buildRootPage()
{
  const DeviceTuningConfig config = getDeviceTuningConfig();
  const float baseAverageHeight = (config.leftHeight + config.rightHeight) * 0.5f;

  String html;
  html.reserve(48000);

  html += "<!DOCTYPE html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>SF Wheel Console</title>";
  html += R"HTML(<style>
:root{--bg:#eef4f1;--panel:#ffffff;--panel-2:#f7faf8;--line:#d7e2db;--text:#172026;--muted:#5d6c66;--accent:#0f766e;--accent-2:#d97706;--shadow:0 18px 48px rgba(18,42,32,.10);}
*{box-sizing:border-box}body{margin:0;font-family:"Trebuchet MS","Segoe UI",sans-serif;color:var(--text);background:radial-gradient(circle at top left,#f9fcfb 0,#eef4f1 42%,#dde9e2 100%);padding:20px}
.app{max-width:1180px;margin:0 auto}
.hero{display:grid;grid-template-columns:1.5fr 1fr;gap:16px;margin-bottom:16px}
.surface{background:rgba(255,255,255,.92);backdrop-filter:blur(10px);border:1px solid rgba(255,255,255,.8);border-radius:24px;box-shadow:var(--shadow);padding:22px}
.eyebrow{margin:0 0 8px;color:var(--accent-2);font-size:12px;letter-spacing:.12em;text-transform:uppercase}
h1{margin:0 0 10px;font-size:34px;line-height:1.05}h2{margin:0 0 14px;font-size:20px}h3{margin:0 0 12px;font-size:15px}
p{margin:0;color:var(--muted);line-height:1.6}
.chip-row{display:flex;flex-wrap:wrap;gap:10px;margin-top:16px}.chip{padding:10px 14px;border-radius:999px;background:#e8f5f2;color:#0b5f58;font-weight:700;font-size:13px}
.menu{display:flex;gap:10px;flex-wrap:wrap;margin:0 0 16px}.menu-btn{border:0;border-radius:999px;padding:12px 18px;background:#dbe8e2;color:#27433b;font-weight:700;cursor:pointer}.menu-btn.active{background:linear-gradient(135deg,var(--accent),#155e75);color:#fff}
.panel{display:none;animation:fade .24s ease}.panel.active{display:block}@keyframes fade{from{opacity:0;transform:translateY(6px)}to{opacity:1;transform:translateY(0)}}
.grid{display:grid;grid-template-columns:repeat(12,1fr);gap:16px}.col-4{grid-column:span 4}.col-6{grid-column:span 6}.col-8{grid-column:span 8}.col-12{grid-column:span 12}
.stat-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:12px}.stat-card{background:var(--panel-2);border:1px solid var(--line);border-radius:18px;padding:16px}.stat-card span{display:block;color:var(--muted);font-size:12px;text-transform:uppercase;letter-spacing:.08em}.stat-card strong{display:block;margin-top:8px;font-size:28px}
.kv-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px}.kv{padding:14px;border-radius:16px;background:var(--panel-2);border:1px solid var(--line)}.kv span{display:block;color:var(--muted);font-size:12px;margin-bottom:6px}.kv strong{font-size:18px}
.profile-grid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:10px}
.profile-card{padding:14px;border-radius:16px;background:var(--panel-2);border:1px solid var(--line)}
.profile-card.active{border-color:var(--accent);box-shadow:inset 0 0 0 1px rgba(15,118,110,.18);background:#ebfbf7}
.profile-card span{display:block;color:var(--muted);font-size:12px;margin-bottom:6px}
.profile-card strong{display:block;font-size:18px}
.mono{font-family:Consolas,"Courier New",monospace}
label{display:block;margin:14px 0 8px;font-weight:700;font-size:14px}input,select{width:100%;padding:12px 14px;border-radius:14px;border:1px solid var(--line);background:#fff;color:var(--text);font-size:15px}
.btn-row{display:flex;gap:10px;flex-wrap:wrap;margin-top:18px}.btn{display:inline-flex;align-items:center;justify-content:center;min-width:180px;padding:12px 18px;border:0;border-radius:14px;background:linear-gradient(135deg,var(--accent),#155e75);color:#fff;font-weight:700;cursor:pointer}.btn.secondary{background:#415954}
.table-scroll{overflow-x:auto}.profile-table{width:100%;border-collapse:collapse;min-width:1080px}.profile-table th,.profile-table td{padding:10px;border-bottom:1px solid var(--line);text-align:left;vertical-align:top}.profile-table th{font-size:12px;letter-spacing:.08em;text-transform:uppercase;color:var(--muted)}
.range-wrap{padding:18px;border-radius:18px;background:var(--panel-2);border:1px solid var(--line)}.range-head{display:flex;justify-content:space-between;gap:10px;align-items:center;margin-bottom:8px}.range-value{font-size:26px;font-weight:700}
input[type=range]{padding:0;border:0;background:transparent}
.chart-box{padding:14px;border-radius:18px;background:var(--panel-2);border:1px solid var(--line)}.chart-box canvas{display:block;width:100%;height:220px;background:#fff;border-radius:14px;border:1px solid #e5ece8}
.record-box textarea{width:100%;min-height:180px;margin-top:12px;padding:12px 14px;border-radius:14px;border:1px solid var(--line);font-family:Consolas,"Courier New",monospace;font-size:12px;resize:vertical}
.small{font-size:12px;color:var(--muted)}.hint{margin-top:10px;font-size:13px}.subtle{color:var(--muted);font-size:13px}
@media(max-width:980px){.hero,.stat-grid,.profile-grid{grid-template-columns:1fr}.col-4,.col-6,.col-8,.col-12{grid-column:span 12}.kv-grid{grid-template-columns:1fr}}
</style>)HTML";
  html += "</head><body><div class='app'>";
  html += "<div class='hero'>";
  html += "<section class='surface'><p class='eyebrow'>Embedded Control Portal</p><h1>SF Wheel Console</h1>";
  html += "<p>Live telemetry, PS2 height profiles, and Wi-Fi setup in one page.</p>";
  html += "<div class='chip-row'><div class='chip'>Mode: <span id='modeChip'>" + htmlEscape(getModeLabel()) + "</span></div>";
  html += "<div class='chip'>Network: <span id='networkChip'>" + htmlEscape(getConnectionDetails()) + "</span></div></div></section>";
  html += "<section class='surface'><h2>Portal Status</h2>";
  html += "<div class='kv-grid'>";
  html += "<div class='kv'><span>Device</span><strong class='mono'>" + htmlEscape(displayName_) + "</strong></div>";
  html += "<div class='kv'><span>mDNS</span><strong class='mono'>" + htmlEscape(mdnsHostname_) + ".local</strong></div>";
  html += "<div class='kv'><span>STA URL</span><strong class='mono'>" + htmlEscape(getStaUrl()) + "</strong></div>";
  html += "<div class='kv'><span>AP SSID</span><strong class='mono'>" + htmlEscape(apSsid_) + "</strong></div>";
  html += "</div><p class='hint'>Status: <span id='statusMessage'>" + htmlEscape(statusMessage_) + "</span></p></section></div>";

  html += "<div class='menu'>";
  html += "<button type='button' class='menu-btn active' data-panel='status'>Status</button>";
  html += "<button type='button' class='menu-btn' data-panel='tuning'>Tuning</button>";
  html += "<button type='button' class='menu-btn' data-panel='wifi'>Wi-Fi</button>";
  html += "</div>";

  html += "<section id='panel-status' class='panel active'><div class='grid'>";
  html += "<div class='col-12 surface'><div class='stat-grid'>";
  html += "<div class='stat-card'><span>Pitch</span><strong id='statPitch'>--</strong></div>";
  html += "<div class='stat-card'><span>Current Height</span><strong id='statHeight'>--</strong></div>";
  html += "<div class='stat-card'><span>Balance Point</span><strong id='statPitchTarget'>--</strong></div>";
  html += "<div class='stat-card'><span>Active Step</span><strong id='statActiveStep'>--</strong></div>";
  html += "</div></div>";
  html += "<div class='col-12 surface'><h2>Live Height Control</h2>";
  html += "<div class='range-wrap'><div class='range-head'><div><strong>PS2 Height Offset</strong><div class='small'>Slider uses the same 0~150 / step 10 scale as the PS2 PAD UP and PAD DOWN.</div></div><div class='range-value' id='liveOffsetLabel'>--</div></div>";
  html += "<input id='heightSlider' type='range' min='0' max='150' step='10' value='" + String(clampHeightOffset(ZeparamremoteValue)) + "'>";
  html += "<div class='kv-grid' style='margin-top:12px'>";
  html += "<div class='kv'><span>Active Profile</span><strong id='sliderActiveProfile'>--</strong></div>";
  html += "<div class='kv'><span>Estimated Height</span><strong id='sliderEstimatedHeight'>--</strong></div>";
  html += "</div></div></div>";
  html += "<div class='col-6 surface'><h2>IMU And Drive</h2><div class='kv-grid'>";
  html += "<div class='kv'><span>Roll</span><strong id='imuRoll'>--</strong></div>";
  html += "<div class='kv'><span>Yaw</span><strong id='imuYaw'>--</strong></div>";
  html += "<div class='kv'><span>Gyro Y</span><strong id='imuGyroY'>--</strong></div>";
  html += "<div class='kv'><span>Height Offset</span><strong id='tuningHeightOffset'>--</strong></div>";
  html += "<div class='kv'><span>Wheel Left Target</span><strong id='wheelLeftTarget'>--</strong></div>";
  html += "<div class='kv'><span>Wheel Right Target</span><strong id='wheelRightTarget'>--</strong></div>";
  html += "<div class='kv'><span>Wheel Left Velocity</span><strong id='wheelLeftVelocity'>--</strong></div>";
  html += "<div class='kv'><span>Wheel Right Velocity</span><strong id='wheelRightVelocity'>--</strong></div>";
  html += "</div></div>";
  html += "<div class='col-6 surface'><h2>PID Status</h2><div class='kv-grid'>";
  html += "<div class='kv'><span>vel_kp</span><strong id='pidVelKp'>--</strong></div>";
  html += "<div class='kv'><span>balance_kp</span><strong id='pidBalanceKp'>--</strong></div>";
  html += "<div class='kv'><span>balance_kd</span><strong id='pidBalanceKd'>--</strong></div>";
  html += "<div class='kv'><span>balance_ki</span><strong id='pidBalanceKi'>--</strong></div>";
  html += "<div class='kv'><span>robot_kp</span><strong id='pidRobotKp'>--</strong></div>";
  html += "<div class='kv'><span>speed_limit</span><strong id='pidSpeedLimit'>--</strong></div>";
  html += "</div></div>";
  html += "<div class='col-6 surface'><h2>Height And Pose</h2><div class='kv-grid'>";
  html += "<div class='kv'><span>Balance Offset</span><strong id='tuningBalanceOffset'>--</strong></div>";
  html += "<div class='kv'><span>Left Base Height</span><strong id='tuningLeftHeight'>--</strong></div>";
  html += "<div class='kv'><span>Right Base Height</span><strong id='tuningRightHeight'>--</strong></div>";
  html += "<div class='kv'><span>Left Target Y</span><strong id='tuningLeftTargetY'>--</strong></div>";
  html += "<div class='kv'><span>Right Target Y</span><strong id='tuningRightTargetY'>--</strong></div>";
  html += "<div class='kv'><span>PS2 Height Offset</span><strong id='tuningHeightOffset'>--</strong></div>";
  html += "<div class='kv'><span>Wheel Mode</span><strong id='statSerialMode'>--</strong></div>";
  html += "</div></div>";
  html += "<div class='col-6 surface'><h2>IK Status</h2><div class='kv-grid'>";
  html += "<div class='kv'><span>Left X</span><strong id='ikLeftX'>--</strong></div>";
  html += "<div class='kv'><span>Right X</span><strong id='ikRightX'>--</strong></div>";
  html += "<div class='kv'><span>Serial Left Y</span><strong id='ikSerialLeftY'>--</strong></div>";
  html += "<div class='kv'><span>Serial Right Y</span><strong id='ikSerialRightY'>--</strong></div>";
  html += "</div><p class='hint'>API: <span id='apiHealth'>waiting</span></p></div>";
  html += "<div class='col-12 surface'><h2>PS2 Height Profiles</h2><div id='profileStatusGrid' class='profile-grid'></div></div>";
  html += "<div class='col-6 surface'><h2>IMU Raw Values</h2><div class='kv-grid'>";
  html += "<div class='kv'><span>Acc X</span><strong id='imuAccX'>--</strong></div>";
  html += "<div class='kv'><span>Acc Y</span><strong id='imuAccY'>--</strong></div>";
  html += "<div class='kv'><span>Acc Z</span><strong id='imuAccZ'>--</strong></div>";
  html += "<div class='kv'><span>Gyro X</span><strong id='imuGyroX'>--</strong></div>";
  html += "<div class='kv'><span>Gyro Y Raw</span><strong id='imuGyroYRaw'>--</strong></div>";
  html += "<div class='kv'><span>Gyro Z</span><strong id='imuGyroZ'>--</strong></div>";
  html += "<div class='kv'><span>IMU Temp</span><strong id='imuTemp'>--</strong></div>";
  html += "<div class='kv'><span>PS2 Link</span><strong id='ps2Link'>--</strong></div>";
  html += "</div></div>";
  html += "<div class='col-6 surface record-box'><h2>Record And Export</h2><p>Record live telemetry while you tune, then export CSV text for later analysis.</p>";
  html += "<div class='btn-row'><button type='button' id='startRecord' class='btn'>Start Record</button><button type='button' id='stopRecord' class='btn secondary'>Stop</button><button type='button' id='clearRecord' class='btn secondary'>Clear</button><button type='button' id='exportRecord' class='btn secondary'>Export CSV</button></div>";
  html += "<p class='hint'>Samples: <span id='recordCount'>0</span></p><textarea id='recordOutput' placeholder='Recorded CSV will appear here'></textarea></div>";
  html += "<div class='col-12 surface'><h2>Live Charts</h2><div class='grid'>";
  html += "<div class='col-4'><div class='chart-box'><h3>Pitch / Balance</h3><canvas id='chartPitch' width='320' height='220'></canvas></div></div>";
  html += "<div class='col-4'><div class='chart-box'><h3>Gyro / Acc Z</h3><canvas id='chartImu' width='320' height='220'></canvas></div></div>";
  html += "<div class='col-4'><div class='chart-box'><h3>Height / Speed</h3><canvas id='chartHeight' width='320' height='220'></canvas></div></div>";
  html += "</div></div>";
  html += "</div></section>";

  html += "<section id='panel-tuning' class='panel'><div class='grid'>";
  html += "<div class='col-12 surface'><h2>Tuning Setup</h2><p>The table below is locked to the PS2 height steps: <span class='mono'>0, 10, 20, ... 150</span>. Each step has its own balance point and PID values. Saving writes to NVS and survives reboot.</p>";
  html += "<form method='post' action='/tuning'><div class='grid'>";
  html += "<div class='col-6'><label for='left_height'>Left Base Height (mm)</label><input id='left_height' name='left_height' type='number' step='0.1' value='" + String(config.leftHeight, 1) + "'></div>";
  html += "<div class='col-6'><label for='right_height'>Right Base Height (mm)</label><input id='right_height' name='right_height' type='number' step='0.1' value='" + String(config.rightHeight, 1) + "'></div>";
  html += "<div class='col-12'><h3>Height Step Table</h3><p class='subtle'>Estimated height = base average height + PS2 offset. The offset itself stays aligned with the PS2 buttons and is not editable.</p></div>";
  html += "<div class='col-12'><div class='table-scroll'><table class='profile-table'><thead><tr>";
  html += "<th>Step</th><th>PS2 Offset</th><th>Est. Height</th><th>Balance Point</th><th>vel_kp</th><th>balance_kp</th><th>balance_kd</th><th>balance_ki</th><th>robot_kp</th><th>speed_limit</th>";
  html += "</tr></thead><tbody>";
  for (int i = 0; i < kHeightProfileCount; ++i)
  {
    const HeightProfile &profile = config.profiles[i];
    html += "<tr>";
    html += "<td><strong>" + String(i) + "</strong></td>";
    html += "<td><span class='mono'>" + String(profile.offset) + "</span></td>";
    html += "<td>" + String(baseAverageHeight + static_cast<float>(profile.offset), 1) + " mm</td>";
    html += "<td><input name='balance_point_" + String(i) + "' type='number' step='0.01' value='" + String(profile.balancePoint, 3) + "'></td>";
    html += "<td><input name='vel_kp_" + String(i) + "' type='number' step='0.001' value='" + String(profile.velKp, 4) + "'></td>";
    html += "<td><input name='balance_kp_" + String(i) + "' type='number' step='0.001' value='" + String(profile.balanceKp, 4) + "'></td>";
    html += "<td><input name='balance_kd_" + String(i) + "' type='number' step='0.001' value='" + String(profile.balanceKd, 4) + "'></td>";
    html += "<td><input name='balance_ki_" + String(i) + "' type='number' step='0.001' value='" + String(profile.balanceKi, 4) + "'></td>";
    html += "<td><input name='robot_kp_" + String(i) + "' type='number' step='0.001' value='" + String(profile.robotKp, 4) + "'></td>";
    html += "<td><input name='speed_limit_" + String(i) + "' type='number' step='1' value='" + String(profile.speedLimit) + "'></td>";
    html += "</tr>";
  }
  html += "</tbody></table></div></div>";
  html += "<div class='col-12'><div class='btn-row'><button type='submit' class='btn'>Save Tuning</button></div></div>";
  html += "</div></form></div></div></section>";

  html += "<section id='panel-wifi' class='panel'><div class='grid'>";
  html += "<div class='col-6 surface'><h2>Wi-Fi Setup</h2><form method='post' action='/save'>";
  html += "<label for='device_name'>Custom Device Name</label>";
  html += "<input id='device_name' name='device_name' value='" + htmlEscape(savedDeviceName_) + "' placeholder='Example: left-leg'>";
  html += "<label for='ssid'>Wi-Fi SSID</label>";
  html += "<input id='ssid' name='ssid' value='" + htmlEscape(savedSsid_) + "' placeholder='Enter SSID' required>";
  html += "<label for='password'>Wi-Fi Password</label>";
  html += "<input id='password' name='password' type='password' placeholder='Enter password'>";
  html += "<div class='btn-row'><button type='submit' class='btn'>Save And Connect</button></div></form></div>";
  html += "<div class='col-6 surface'><h2>Nearby Networks</h2>";
  html += "<label for='ssidList'>Scanned SSID</label>";
  html += "<select id='ssidList' onchange=\"document.getElementById('ssid').value=this.value;\">";
  html += buildNetworkOptions();
  html += "</select>";
  html += "<div class='btn-row'><form method='post' action='/rescan'><button type='submit' class='btn secondary'>Rescan Wi-Fi</button></form>";
  html += "<form method='post' action='/forget'><button type='submit' class='btn secondary'>Clear Stored Wi-Fi</button></form></div>";
  html += "<p class='hint'>The custom name is used for AP SSID and mDNS host naming.</p></div>";
  html += "<div class='col-12 surface'><h2>Network Metadata</h2><div class='kv-grid'>";
  html += "<div class='kv'><span>Device Name</span><strong class='mono'>" + htmlEscape(displayName_) + "</strong></div>";
  html += "<div class='kv'><span>mDNS Hostname</span><strong class='mono'>" + htmlEscape(mdnsHostname_) + ".local</strong></div>";
  html += "<div class='kv'><span>STA URL</span><strong class='mono'>" + htmlEscape(getStaUrl()) + "</strong></div>";
  html += "<div class='kv'><span>mDNS URL</span><strong class='mono'>" + htmlEscape(getHostUrl()) + "</strong></div>";
  html += "<div class='kv'><span>AP SSID</span><strong class='mono'>" + htmlEscape(apSsid_) + "</strong></div>";
  html += "<div class='kv'><span>AP Password</span><strong class='mono'>" + htmlEscape(String(kPortalApPassword)) + "</strong></div>";
  html += "<div class='kv'><span>MAC Address</span><strong class='mono'>" + htmlEscape(macAddress_) + "</strong></div>";
  html += "<div class='kv'><span>Unique ID</span><strong class='mono'>" + htmlEscape(chipSuffix_) + "</strong></div>";
  html += "</div></div></div></section>";

  html += R"HTML(<script>
const buttons=document.querySelectorAll('.menu-btn');
const panels=document.querySelectorAll('.panel');
const heightSlider=document.getElementById('heightSlider');
const chartHistory=[];
const maxChartSamples=120;
let isRecording=false;
let recordRows=[];
function activatePanel(name){
  buttons.forEach((button)=>button.classList.toggle('active',button.dataset.panel===name));
  panels.forEach((panel)=>panel.classList.toggle('active',panel.id==='panel-'+name));
}
buttons.forEach((button)=>button.addEventListener('click',()=>activatePanel(button.dataset.panel)));
function setText(id,value){
  const node=document.getElementById(id);
  if(node){node.textContent=value;}
}
function pick(root,key,empty){
  return root&&root[key]!==undefined&&root[key]!==null?root[key]:empty;
}
function fmt(value,digits){
  const number=Number(value);
  return Number.isFinite(number)?number.toFixed(digits):'--';
}
function clampSliderValue(value){
  const number=Number(value);
  if(!Number.isFinite(number)){return 0;}
  return Math.max(0,Math.min(150,Math.round(number/10)*10));
}
function renderProfiles(profiles,activeStep){
  const host=document.getElementById('profileStatusGrid');
  if(!host){return;}
  host.innerHTML=(profiles||[]).map((profile)=>{
    const active=Number(profile.index)===Number(activeStep)?' active':'';
    return '<div class="profile-card'+active+'"><span>Step '+profile.index+' / offset '+profile.offset+'</span><strong>'+fmt(profile.balance_point,2)+' deg</strong><div class="small">vel '+fmt(profile.vel_kp,3)+' | bal '+fmt(profile.balance_kp,3)+' / '+fmt(profile.balance_kd,3)+' / '+fmt(profile.balance_ki,3)+' | robot '+fmt(profile.robot_kp,3)+' | limit '+String(profile.speed_limit)+'</div></div>';
  }).join('');
}
function drawChart(canvasId,series,options){
  const canvas=document.getElementById(canvasId);
  if(!canvas){return;}
  const ctx=canvas.getContext('2d');
  const width=canvas.width;
  const height=canvas.height;
  ctx.clearRect(0,0,width,height);
  ctx.fillStyle='#ffffff';
  ctx.fillRect(0,0,width,height);
  ctx.strokeStyle='#d7e2db';
  for(let i=0;i<5;i++){
    const y=12+(height-24)*i/4;
    ctx.beginPath();
    ctx.moveTo(10,y);
    ctx.lineTo(width-10,y);
    ctx.stroke();
  }
  const values=[];
  series.forEach((item)=>item.data.forEach((value)=>{ if(Number.isFinite(value)){values.push(value);} }));
  if(values.length===0){return;}
  let min=Math.min(...values);
  let max=Math.max(...values);
  if(options&&Number.isFinite(options.min)){min=options.min;}
  if(options&&Number.isFinite(options.max)){max=options.max;}
  if(Math.abs(max-min)<1e-6){max=min+1;}
  ctx.font='11px Trebuchet MS';
  ctx.fillStyle='#5d6c66';
  ctx.fillText(max.toFixed(2),12,16);
  ctx.fillText(min.toFixed(2),12,height-8);
  series.forEach((item)=>{
    ctx.strokeStyle=item.color;
    ctx.lineWidth=2;
    ctx.beginPath();
    item.data.forEach((value,index)=>{
      if(!Number.isFinite(value)){return;}
      const x=12+(width-24)*(series[0].data.length<=1?0:index/(series[0].data.length-1));
      const y=(height-12)-((value-min)/(max-min))*(height-24);
      if(index===0){ctx.moveTo(x,y);}else{ctx.lineTo(x,y);}
    });
    ctx.stroke();
  });
}
function pushChartSample(data){
  chartHistory.push({
    pitch:Number(pick(data.imu,'pitch',NaN)),
    balancePoint:Number(pick(data.tuning,'current_pitch_target',NaN)),
    gyroY:Number(pick(data.imu,'gyro_y',NaN)),
    accZ:Number(pick(data.imu,'acc_z',NaN)),
    currentHeight:Number(pick(data.tuning,'current_height',NaN)),
    wheelLeft:Number(pick(data.control,'wheel_left_target',NaN)),
    wheelRight:Number(pick(data.control,'wheel_right_target',NaN))
  });
  while(chartHistory.length>maxChartSamples){chartHistory.shift();}
}
function renderCharts(){
  drawChart('chartPitch',[
    {data:chartHistory.map((row)=>row.pitch),color:'#0f766e'},
    {data:chartHistory.map((row)=>row.balancePoint),color:'#d97706'}
  ]);
  drawChart('chartImu',[
    {data:chartHistory.map((row)=>row.gyroY),color:'#155e75'},
    {data:chartHistory.map((row)=>row.accZ),color:'#7c3aed'}
  ]);
  drawChart('chartHeight',[
    {data:chartHistory.map((row)=>row.currentHeight),color:'#15803d'},
    {data:chartHistory.map((row)=>row.wheelLeft),color:'#b45309'},
    {data:chartHistory.map((row)=>row.wheelRight),color:'#be123c'}
  ]);
}
function updateRecordOutput(text){
  const node=document.getElementById('recordOutput');
  if(node){node.value=text;}
}
function updateRecordCount(){
  setText('recordCount',String(recordRows.length));
}
function buildCsv(){
  if(recordRows.length===0){return '';}
  const headers=Object.keys(recordRows[0]);
  const lines=[headers.join(',')];
  recordRows.forEach((row)=>{
    lines.push(headers.map((key)=>String(row[key])).join(','));
  });
  return lines.join('\n');
}
async function postHeightOffset(offset){
  try{
    const form=new URLSearchParams();
    form.set('offset',String(clampSliderValue(offset)));
    const response=await fetch('/api/height',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:form.toString()});
    if(!response.ok){throw new Error('http');}
  }catch(error){
    setText('statusMessage','Live height update failed');
  }
}
let sliderTimer=null;
if(heightSlider){
  heightSlider.addEventListener('input',()=>{
    setText('liveOffsetLabel',heightSlider.value);
  });
  heightSlider.addEventListener('change',()=>{
    if(sliderTimer){clearTimeout(sliderTimer);}
    sliderTimer=setTimeout(()=>postHeightOffset(heightSlider.value),40);
  });
}
const startRecordBtn=document.getElementById('startRecord');
const stopRecordBtn=document.getElementById('stopRecord');
const clearRecordBtn=document.getElementById('clearRecord');
const exportRecordBtn=document.getElementById('exportRecord');
if(startRecordBtn){
  startRecordBtn.addEventListener('click',()=>{
    isRecording=true;
    setText('statusMessage','Telemetry recording started in browser');
  });
}
if(stopRecordBtn){
  stopRecordBtn.addEventListener('click',()=>{
    isRecording=false;
    setText('statusMessage','Telemetry recording stopped');
  });
}
if(clearRecordBtn){
  clearRecordBtn.addEventListener('click',()=>{
    recordRows=[];
    updateRecordCount();
    updateRecordOutput('');
  });
}
if(exportRecordBtn){
  exportRecordBtn.addEventListener('click',()=>{
    updateRecordOutput(buildCsv());
  });
}
async function refreshStatus(){
  try{
    const response=await fetch('/api/status',{cache:'no-store'});
    if(!response.ok){throw new Error('http');}
    const data=await response.json();
    setText('modeChip',data.mode||'--');
    setText('networkChip',data.network||'--');
    setText('statusMessage',data.status||'--');
    setText('statPitch',fmt(pick(data.imu,'pitch',null),2)+' deg');
    setText('statHeight',fmt(pick(data.tuning,'current_height',null),1)+' mm');
    setText('statPitchTarget',fmt(pick(data.tuning,'current_pitch_target',null),2)+' deg');
    setText('statActiveStep','Step '+String(pick(data.tuning,'active_step','--'))+' / '+String(pick(data.tuning,'active_offset','--')));
    setText('liveOffsetLabel',String(clampSliderValue(pick(data.tuning,'height_offset',0))));
    setText('sliderActiveProfile','Step '+String(pick(data.tuning,'active_step','--'))+' / offset '+String(pick(data.tuning,'active_offset','--')));
    setText('sliderEstimatedHeight',fmt(pick(data.tuning,'current_height',null),1)+' mm');
    if(heightSlider){heightSlider.value=String(clampSliderValue(pick(data.tuning,'height_offset',0)));}
    setText('statSerialMode',pick(data.ik,'serial_mode',false)?'SERIAL IK':'AUTO');
    setText('imuRoll',fmt(pick(data.imu,'roll',null),2)+' deg');
    setText('imuYaw',fmt(pick(data.imu,'yaw',null),2)+' deg');
    setText('imuGyroY',fmt(pick(data.imu,'gyro_y',null),2)+' deg/s');
    setText('imuAccX',fmt(pick(data.imu,'acc_x',null),3)+' g');
    setText('imuAccY',fmt(pick(data.imu,'acc_y',null),3)+' g');
    setText('imuAccZ',fmt(pick(data.imu,'acc_z',null),3)+' g');
    setText('imuGyroX',fmt(pick(data.imu,'gyro_x',null),2)+' deg/s');
    setText('imuGyroYRaw',fmt(pick(data.imu,'gyro_y',null),2)+' deg/s');
    setText('imuGyroZ',fmt(pick(data.imu,'gyro_z',null),2)+' deg/s');
    setText('imuTemp',fmt(pick(data.imu,'temp_c',null),2)+' C');
    setText('ps2Link',pick(data.ik,'ps2_connected',false)?'connected':'offline');
    setText('tuningHeightOffset',fmt(pick(data.tuning,'height_offset',null),1)+' mm');
    setText('wheelLeftTarget',fmt(pick(data.control,'wheel_left_target',null),2));
    setText('wheelRightTarget',fmt(pick(data.control,'wheel_right_target',null),2));
    setText('wheelLeftVelocity',fmt(pick(data.control,'wheel_left_velocity',null),2));
    setText('wheelRightVelocity',fmt(pick(data.control,'wheel_right_velocity',null),2));
    setText('pidVelKp',fmt(pick(data.control,'vel_kp',null),4));
    setText('pidBalanceKp',fmt(pick(data.control,'balance_kp',null),4));
    setText('pidBalanceKd',fmt(pick(data.control,'balance_kd',null),4));
    setText('pidBalanceKi',fmt(pick(data.control,'balance_ki',null),4));
    setText('pidRobotKp',fmt(pick(data.control,'robot_kp',null),4));
    setText('pidSpeedLimit',String(pick(data.control,'speed_limit','--')));
    setText('tuningBalanceOffset',fmt(pick(data.tuning,'balance_offset',null),2)+' deg');
    setText('tuningLeftHeight',fmt(pick(data.tuning,'left_height',null),1)+' mm');
    setText('tuningRightHeight',fmt(pick(data.tuning,'right_height',null),1)+' mm');
    setText('tuningLeftTargetY',fmt(pick(data.tuning,'left_target_y',null),1)+' mm');
    setText('tuningRightTargetY',fmt(pick(data.tuning,'right_target_y',null),1)+' mm');
    setText('ikLeftX',fmt(pick(data.ik,'left_x',null),1)+' mm');
    setText('ikRightX',fmt(pick(data.ik,'right_x',null),1)+' mm');
    setText('ikSerialLeftY',fmt(pick(data.ik,'serial_left_y',null),1)+' mm');
    setText('ikSerialRightY',fmt(pick(data.ik,'serial_right_y',null),1)+' mm');
    setText('apiHealth','live');
    renderProfiles(data.profiles,pick(data.tuning,'active_step',-1));
    pushChartSample(data);
    renderCharts();
    if(isRecording){
      recordRows.push({
        ms:Date.now(),
        step:pick(data.tuning,'active_step',''),
        offset:pick(data.tuning,'active_offset',''),
        pitch:fmt(pick(data.imu,'pitch',null),4),
        roll:fmt(pick(data.imu,'roll',null),4),
        yaw:fmt(pick(data.imu,'yaw',null),4),
        acc_x:fmt(pick(data.imu,'acc_x',null),5),
        acc_y:fmt(pick(data.imu,'acc_y',null),5),
        acc_z:fmt(pick(data.imu,'acc_z',null),5),
        gyro_x:fmt(pick(data.imu,'gyro_x',null),4),
        gyro_y:fmt(pick(data.imu,'gyro_y',null),4),
        gyro_z:fmt(pick(data.imu,'gyro_z',null),4),
        temp_c:fmt(pick(data.imu,'temp_c',null),3),
        balance_point:fmt(pick(data.tuning,'current_pitch_target',null),4),
        current_height:fmt(pick(data.tuning,'current_height',null),4),
        vel_kp:fmt(pick(data.control,'vel_kp',null),5),
        balance_kp:fmt(pick(data.control,'balance_kp',null),5),
        balance_kd:fmt(pick(data.control,'balance_kd',null),5),
        balance_ki:fmt(pick(data.control,'balance_ki',null),5),
        robot_kp:fmt(pick(data.control,'robot_kp',null),5),
        speed_limit:pick(data.control,'speed_limit',''),
        wheel_left_target:fmt(pick(data.control,'wheel_left_target',null),4),
        wheel_right_target:fmt(pick(data.control,'wheel_right_target',null),4)
      });
      updateRecordCount();
    }
  }catch(error){
    setText('apiHealth','offline');
  }
}
activatePanel('status');
refreshStatus();
setInterval(refreshStatus,1000);
</script>)HTML";
  html += "</div></body></html>";

  return html;
}

String WiFiManagerPortal::buildNetworkOptions()
{
  return cachedNetworkOptions_;
}

String WiFiManagerPortal::getModeLabel() const
{
  if (WiFi.status() == WL_CONNECTED)
  {
    return apModeActive_ ? "STA + AP" : "STA";
  }

  if (apModeActive_)
  {
    return "AP";
  }

  return "Disconnected";
}

String WiFiManagerPortal::getConnectionDetails() const
{
  if (WiFi.status() == WL_CONNECTED)
  {
    return "SSID: " + WiFi.SSID() + ", IP: " + WiFi.localIP().toString();
  }

  if (apModeActive_)
  {
    return "AP IP: " + WiFi.softAPIP().toString();
  }

  return "No active network link";
}

String WiFiManagerPortal::getHostUrl() const
{
  return mdnsHostname_ + ".local";
}

String WiFiManagerPortal::getStaUrl() const
{
  if (WiFi.status() == WL_CONNECTED)
  {
    return WiFi.localIP().toString();
  }

  return "Not connected";
}

String WiFiManagerPortal::htmlEscape(const String &value)
{
  String escaped;
  escaped.reserve(value.length() + 8);

  for (size_t i = 0; i < value.length(); ++i)
  {
    const char ch = value[i];
    switch (ch)
    {
    case '&':
      escaped += "&amp;";
      break;
    case '<':
      escaped += "&lt;";
      break;
    case '>':
      escaped += "&gt;";
      break;
    case '"':
      escaped += "&quot;";
      break;
    case '\'':
      escaped += "&#39;";
      break;
    default:
      escaped += ch;
      break;
    }
  }

  return escaped;
}

String WiFiManagerPortal::jsonEscape(const String &value)
{
  String escaped;
  escaped.reserve(value.length() + 8);

  for (size_t i = 0; i < value.length(); ++i)
  {
    const char ch = value[i];
    switch (ch)
    {
    case '\\':
      escaped += "\\\\";
      break;
    case '"':
      escaped += "\\\"";
      break;
    case '\n':
      escaped += "\\n";
      break;
    case '\r':
      escaped += "\\r";
      break;
    case '\t':
      escaped += "\\t";
      break;
    default:
      escaped += ch;
      break;
    }
  }

  return escaped;
}

String WiFiManagerPortal::trimToLength(const String &value, size_t maxLen)
{
  if (value.length() <= maxLen)
  {
    return value;
  }
  return value.substring(0, maxLen);
}

String WiFiManagerPortal::sanitizeHostLabel(const String &value)
{
  String out;
  out.reserve(value.length());

  for (size_t i = 0; i < value.length(); ++i)
  {
    const char ch = value[i];
    const bool isLower = ch >= 'a' && ch <= 'z';
    const bool isUpper = ch >= 'A' && ch <= 'Z';
    const bool isDigit = ch >= '0' && ch <= '9';
    if (isLower || isUpper || isDigit)
    {
      out += static_cast<char>(tolower(ch));
    }
    else if ((ch == ' ' || ch == '_' || ch == '-') && !out.endsWith("-"))
    {
      out += '-';
    }
  }

  while (out.startsWith("-"))
  {
    out.remove(0, 1);
  }
  while (out.endsWith("-"))
  {
    out.remove(out.length() - 1, 1);
  }

  return trimToLength(out, kMaxHostLabelLen);
}

String WiFiManagerPortal::sanitizeDisplayName(const String &value)
{
  String out;
  out.reserve(value.length());

  for (size_t i = 0; i < value.length(); ++i)
  {
    const char ch = value[i];
    const bool isLetter = (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z');
    const bool isDigit = ch >= '0' && ch <= '9';
    const bool isSpace = ch == ' ';
    const bool isSafePunct = ch == '-' || ch == '_';
    if (isLetter || isDigit || isSpace || isSafePunct)
    {
      out += ch;
    }
  }

  out.trim();
  return trimToLength(out, kMaxDisplayNameLen);
}
