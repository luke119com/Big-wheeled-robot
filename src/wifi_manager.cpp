#include "wifi_manager.h"

#include "device_tuning.h"
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
constexpr const char *kPrefsInitPitchKey = "init_pitch";
constexpr const char *kPrefsLeftYKey = "left_y";
constexpr const char *kPrefsRightYKey = "right_y";
constexpr const char *kPrefsMapH0Key = "map_h0";
constexpr const char *kPrefsMapH1Key = "map_h1";
constexpr const char *kPrefsMapH2Key = "map_h2";
constexpr const char *kPrefsMapP0Key = "map_p0";
constexpr const char *kPrefsMapP1Key = "map_p1";
constexpr const char *kPrefsMapP2Key = "map_p2";
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
  config.initPitch = g_preferences.isKey(kPrefsInitPitchKey) ? g_preferences.getFloat(kPrefsInitPitchKey, config.initPitch) : config.initPitch;
  config.leftHeight = g_preferences.isKey(kPrefsLeftYKey) ? g_preferences.getFloat(kPrefsLeftYKey, config.leftHeight) : config.leftHeight;
  config.rightHeight = g_preferences.isKey(kPrefsRightYKey) ? g_preferences.getFloat(kPrefsRightYKey, config.rightHeight) : config.rightHeight;
  config.pitchMap[0].height = g_preferences.isKey(kPrefsMapH0Key) ? g_preferences.getFloat(kPrefsMapH0Key, config.pitchMap[0].height) : config.pitchMap[0].height;
  config.pitchMap[1].height = g_preferences.isKey(kPrefsMapH1Key) ? g_preferences.getFloat(kPrefsMapH1Key, config.pitchMap[1].height) : config.pitchMap[1].height;
  config.pitchMap[2].height = g_preferences.isKey(kPrefsMapH2Key) ? g_preferences.getFloat(kPrefsMapH2Key, config.pitchMap[2].height) : config.pitchMap[2].height;
  config.pitchMap[0].pitch = g_preferences.isKey(kPrefsMapP0Key) ? g_preferences.getFloat(kPrefsMapP0Key, config.pitchMap[0].pitch) : config.pitchMap[0].pitch;
  config.pitchMap[1].pitch = g_preferences.isKey(kPrefsMapP1Key) ? g_preferences.getFloat(kPrefsMapP1Key, config.pitchMap[1].pitch) : config.pitchMap[1].pitch;
  config.pitchMap[2].pitch = g_preferences.isKey(kPrefsMapP2Key) ? g_preferences.getFloat(kPrefsMapP2Key, config.pitchMap[2].pitch) : config.pitchMap[2].pitch;
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
  g_preferences.putFloat(kPrefsInitPitchKey, config.initPitch);
  g_preferences.putFloat(kPrefsLeftYKey, config.leftHeight);
  g_preferences.putFloat(kPrefsRightYKey, config.rightHeight);
  g_preferences.putFloat(kPrefsMapH0Key, config.pitchMap[0].height);
  g_preferences.putFloat(kPrefsMapH1Key, config.pitchMap[1].height);
  g_preferences.putFloat(kPrefsMapH2Key, config.pitchMap[2].height);
  g_preferences.putFloat(kPrefsMapP0Key, config.pitchMap[0].pitch);
  g_preferences.putFloat(kPrefsMapP1Key, config.pitchMap[1].pitch);
  g_preferences.putFloat(kPrefsMapP2Key, config.pitchMap[2].pitch);
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
  config.initPitch = readArgFloat("init_pitch", config.initPitch);
  config.leftHeight = readArgFloat("left_height", config.leftHeight);
  config.rightHeight = readArgFloat("right_height", config.rightHeight);
  config.pitchMap[0].height = readArgFloat("map_height_0", config.pitchMap[0].height);
  config.pitchMap[1].height = readArgFloat("map_height_1", config.pitchMap[1].height);
  config.pitchMap[2].height = readArgFloat("map_height_2", config.pitchMap[2].height);
  config.pitchMap[0].pitch = readArgFloat("map_pitch_0", config.pitchMap[0].pitch);
  config.pitchMap[1].pitch = readArgFloat("map_pitch_1", config.pitchMap[1].pitch);
  config.pitchMap[2].pitch = readArgFloat("map_pitch_2", config.pitchMap[2].pitch);

  applyDeviceTuningConfig(config);
  saveTuningConfig();
  statusMessage_ = "Tuning values updated.";
  g_server.send(200, "text/html; charset=utf-8", buildRootPage());
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

  String json;
  json.reserve(1400);
  json += "{";
  json += "\"mode\":\"" + jsonEscape(getModeLabel()) + "\",";
  json += "\"status\":\"" + jsonEscape(statusMessage_) + "\",";
  json += "\"network\":\"" + jsonEscape(getConnectionDetails()) + "\",";
  json += "\"display_name\":\"" + jsonEscape(displayName_) + "\",";
  json += "\"imu\":{";
  json += "\"roll\":" + String(roll, 3) + ",";
  json += "\"pitch\":" + String(pitch, 3) + ",";
  json += "\"yaw\":" + String(yaw, 3) + ",";
  json += "\"gyro_y\":" + String(gyroY, 3);
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
  json += "\"init_pitch\":" + String(config.initPitch, 3) + ",";
  json += "\"balance_offset\":" + String(balance_offset, 3) + ",";
  json += "\"current_height\":" + String(getCurrentHeightForPitchControl(), 3) + ",";
  json += "\"current_pitch_target\":" + String(getCurrentPitchTarget(), 3) + ",";
  json += "\"height_offset\":" + String(static_cast<float>(ZeparamremoteValue), 3) + ",";
  json += "\"left_height\":" + String(leftY, 3) + ",";
  json += "\"right_height\":" + String(rightY, 3) + ",";
  json += "\"left_target_y\":" + String(Y1, 3) + ",";
  json += "\"right_target_y\":" + String(y2, 3);
  json += "},";
  json += "\"ik\":{";
  json += "\"serial_mode\":" + String(serialFootPoseMode ? "true" : "false") + ",";
  json += "\"left_x\":" + String(x1, 3) + ",";
  json += "\"right_x\":" + String(x2, 3) + ",";
  json += "\"serial_left_y\":" + String(serialLeftTargetY, 3) + ",";
  json += "\"serial_right_y\":" + String(serialRightTargetY, 3);
  json += "},";
  json += "\"pitch_map\":[";
  for (int i = 0; i < 3; ++i)
  {
    if (i > 0)
    {
      json += ",";
    }
    json += "{\"height\":" + String(config.pitchMap[i].height, 3) + ",\"pitch\":" + String(config.pitchMap[i].pitch, 3) + "}";
  }
  json += "]}";
  return json;
}

String WiFiManagerPortal::buildRootPage()
{
  const DeviceTuningConfig config = getDeviceTuningConfig();

  String html;
  html.reserve(18000);

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
.mono{font-family:Consolas,"Courier New",monospace}
label{display:block;margin:14px 0 8px;font-weight:700;font-size:14px}input,select{width:100%;padding:12px 14px;border-radius:14px;border:1px solid var(--line);background:#fff;color:var(--text);font-size:15px}
.btn-row{display:flex;gap:10px;flex-wrap:wrap;margin-top:18px}.btn{display:inline-flex;align-items:center;justify-content:center;min-width:180px;padding:12px 18px;border:0;border-radius:14px;background:linear-gradient(135deg,var(--accent),#155e75);color:#fff;font-weight:700;cursor:pointer}.btn.secondary{background:#415954}
.map-grid{display:grid;grid-template-columns:1.1fr 1fr;gap:10px;align-items:end}.hint{margin-top:10px;font-size:13px}.subtle{color:var(--muted);font-size:13px}
@media(max-width:980px){.hero,.stat-grid{grid-template-columns:1fr}.col-4,.col-6,.col-8,.col-12{grid-column:span 12}.kv-grid{grid-template-columns:1fr}.map-grid{grid-template-columns:1fr}}
</style>)HTML";
  html += "</head><body><div class='app'>";
  html += "<div class='hero'>";
  html += "<section class='surface'><p class='eyebrow'>Embedded Control Portal</p><h1>SF Wheel Console</h1>";
  html += "<p>Live telemetry, pitch tuning, and Wi-Fi setup for the robot in one page.</p>";
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
  html += "<div class='stat-card'><span>Pitch Target</span><strong id='statPitchTarget'>--</strong></div>";
  html += "<div class='stat-card'><span>Wheel Mode</span><strong id='statSerialMode'>--</strong></div>";
  html += "</div></div>";
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
  html += "<div class='kv'><span>Init Pitch</span><strong id='tuningInitPitch'>--</strong></div>";
  html += "<div class='kv'><span>Balance Offset</span><strong id='tuningBalanceOffset'>--</strong></div>";
  html += "<div class='kv'><span>Left Base Height</span><strong id='tuningLeftHeight'>--</strong></div>";
  html += "<div class='kv'><span>Right Base Height</span><strong id='tuningRightHeight'>--</strong></div>";
  html += "<div class='kv'><span>Left Target Y</span><strong id='tuningLeftTargetY'>--</strong></div>";
  html += "<div class='kv'><span>Right Target Y</span><strong id='tuningRightTargetY'>--</strong></div>";
  html += "</div></div>";
  html += "<div class='col-6 surface'><h2>IK Status</h2><div class='kv-grid'>";
  html += "<div class='kv'><span>Left X</span><strong id='ikLeftX'>--</strong></div>";
  html += "<div class='kv'><span>Right X</span><strong id='ikRightX'>--</strong></div>";
  html += "<div class='kv'><span>Serial Left Y</span><strong id='ikSerialLeftY'>--</strong></div>";
  html += "<div class='kv'><span>Serial Right Y</span><strong id='ikSerialRightY'>--</strong></div>";
  html += "</div><p class='hint'>API: <span id='apiHealth'>waiting</span></p></div>";
  html += "<div class='col-12 surface'><h2>Height To Pitch Map</h2><div id='pitchMapStatus' class='kv-grid'></div></div>";
  html += "</div></section>";

  html += "<section id='panel-tuning' class='panel'><div class='grid'>";
  html += "<div class='col-12 surface'><h2>Tuning Setup</h2><p>Base height is the left/right wheel stance. The pitch target used by balance control is <span class='mono'>init_pitch + interpolated(height map)</span>.</p>";
  html += "<form method='post' action='/tuning'><div class='grid'>";
  html += "<div class='col-4'><label for='init_pitch'>Init Pitch (deg)</label><input id='init_pitch' name='init_pitch' type='number' step='0.01' value='" + String(config.initPitch, 2) + "'></div>";
  html += "<div class='col-4'><label for='left_height'>Left Height (mm)</label><input id='left_height' name='left_height' type='number' step='0.1' value='" + String(config.leftHeight, 1) + "'></div>";
  html += "<div class='col-4'><label for='right_height'>Right Height (mm)</label><input id='right_height' name='right_height' type='number' step='0.1' value='" + String(config.rightHeight, 1) + "'></div>";
  html += "<div class='col-12'><h3>Height To Pitch Points</h3><p class='subtle'>Use three points. The firmware sorts them by height and linearly interpolates between them.</p></div>";
  for (int i = 0; i < 3; ++i)
  {
    html += "<div class='col-12'><div class='map-grid'>";
    html += "<div><label for='map_height_" + String(i) + "'>Point " + String(i + 1) + " Height (mm)</label><input id='map_height_" + String(i) + "' name='map_height_" + String(i) + "' type='number' step='0.1' value='" + String(config.pitchMap[i].height, 1) + "'></div>";
    html += "<div><label for='map_pitch_" + String(i) + "'>Point " + String(i + 1) + " Pitch (deg)</label><input id='map_pitch_" + String(i) + "' name='map_pitch_" + String(i) + "' type='number' step='0.01' value='" + String(config.pitchMap[i].pitch, 2) + "'></div>";
    html += "</div></div>";
  }
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
function renderPitchMap(points){
  const host=document.getElementById('pitchMapStatus');
  if(!host){return;}
  host.innerHTML=(points||[]).map((point,index)=>'<div class="kv"><span>Point '+(index+1)+'</span><strong>'+fmt(point.height,1)+' mm -> '+fmt(point.pitch,2)+' deg</strong></div>').join('');
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
    setText('statSerialMode',pick(data.ik,'serial_mode',false)?'SERIAL IK':'AUTO');
    setText('imuRoll',fmt(pick(data.imu,'roll',null),2)+' deg');
    setText('imuYaw',fmt(pick(data.imu,'yaw',null),2)+' deg');
    setText('imuGyroY',fmt(pick(data.imu,'gyro_y',null),2)+' deg/s');
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
    setText('tuningInitPitch',fmt(pick(data.tuning,'init_pitch',null),2)+' deg');
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
    renderPitchMap(data.pitch_map);
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
