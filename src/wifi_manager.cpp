#include "wifi_manager.h"

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
}

WiFiManagerPortal wifiManagerPortal;

void WiFiManagerPortal::begin()
{
  WiFi.persistent(false);
  WiFi.setSleep(false);

  initIdentity();
  setupRoutes();
  loadPreferences();
  refreshIdentity();

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
  g_server.on("/save", HTTP_POST, [this]() { handleSave(); });
  g_server.on("/forget", HTTP_POST, [this]() { handleForget(); });
  g_server.onNotFound([this]() { handleNotFound(); });
}

void WiFiManagerPortal::loadPreferences()
{
  if (!g_preferences.begin(kPrefsNamespace, false))
  {
    statusMessage_ = "Failed to open NVS. Wi-Fi settings will not persist.";
    savedSsid_ = "";
    savedPassword_ = "";
    savedDeviceName_ = "";
    return;
  }

  savedSsid_ = g_preferences.getString(kPrefsSsidKey, "");
  savedPassword_ = g_preferences.getString(kPrefsPassKey, "");
  savedDeviceName_ = g_preferences.getString(kPrefsNameKey, "");
}

void WiFiManagerPortal::saveCredentials(const String &ssid, const String &password)
{
  savedSsid_ = ssid;
  savedPassword_ = password;
  g_preferences.putString(kPrefsSsidKey, ssid);
  g_preferences.putString(kPrefsPassKey, password);
}

void WiFiManagerPortal::saveDeviceName(const String &deviceName)
{
  savedDeviceName_ = deviceName;
  g_preferences.putString(kPrefsNameKey, deviceName);
  refreshIdentity();
}

void WiFiManagerPortal::clearCredentials()
{
  savedSsid_ = "";
  savedPassword_ = "";
  g_preferences.remove(kPrefsSsidKey);
  g_preferences.remove(kPrefsPassKey);
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

  const String apBase = hostBase.isEmpty() ? "SF_Wheel" : hostBase;
  String apName = apBase;
  apName.replace("-", "_");
  apName.toUpperCase();
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
                "<p><a href='/'>Back</a></p></body></html>");
}

void WiFiManagerPortal::handleForget()
{
  clearCredentials();
  statusMessage_ = "Stored Wi-Fi settings cleared. AP portal is active.";
  WiFi.disconnect(false, false);
  startAccessPoint();
  g_server.send(200, "text/html; charset=utf-8", buildRootPage());
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

String WiFiManagerPortal::buildRootPage()
{
  String html;
  html.reserve(7000);

  html += "<!DOCTYPE html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>SF Bipedal Wheel Wi-Fi</title>";
  html += "<style>";
  html += "body{font-family:Arial,Helvetica,sans-serif;background:#f5f7fb;color:#1f2937;margin:0;padding:24px;}";
  html += ".card{max-width:880px;margin:0 auto;background:#fff;border-radius:16px;padding:24px;box-shadow:0 10px 30px rgba(15,23,42,.08);}";
  html += "h1{margin-top:0;font-size:28px;}p{line-height:1.5;}label{display:block;margin:14px 0 6px;font-weight:600;}";
  html += "input,select,button{width:100%;padding:12px;border-radius:10px;border:1px solid #cbd5e1;font-size:16px;box-sizing:border-box;}";
  html += "button{background:#0f766e;color:#fff;border:none;font-weight:700;cursor:pointer;margin-top:16px;}";
  html += ".secondary{background:#475569;}.status{padding:12px 14px;border-radius:12px;background:#ecfeff;margin:16px 0;}";
  html += ".grid{display:grid;grid-template-columns:1fr;gap:14px;}@media(min-width:780px){.grid{grid-template-columns:1fr 1fr;}}";
  html += ".mono{font-family:Consolas,monospace;}.meta{display:grid;grid-template-columns:1fr;gap:10px;margin:18px 0;}@media(min-width:780px){.meta{grid-template-columns:1fr 1fr;}}";
  html += ".meta div{background:#f8fafc;border:1px solid #e2e8f0;border-radius:12px;padding:12px;}";
  html += "</style></head><body><div class='card'>";
  html += "<h1>Wi-Fi Setup</h1>";
  html += "<div class='status'><strong>Mode:</strong> " + htmlEscape(getModeLabel()) + "<br>";
  html += "<strong>Status:</strong> " + htmlEscape(statusMessage_) + "<br>";
  html += "<strong>Network:</strong> " + htmlEscape(getConnectionDetails()) + "</div>";
  html += "<div class='meta'>";
  html += "<div><strong>Device Name</strong><br><span class='mono'>" + htmlEscape(displayName_) + "</span></div>";
  html += "<div><strong>mDNS Hostname</strong><br><span class='mono'>" + htmlEscape(mdnsHostname_) + ".local</span></div>";
  html += "<div><strong>STA URL</strong><br><span class='mono'>" + htmlEscape(getStaUrl()) + "</span></div>";
  html += "<div><strong>mDNS URL</strong><br><span class='mono'>" + htmlEscape(getHostUrl()) + "</span></div>";
  html += "<div><strong>AP SSID</strong><br><span class='mono'>" + htmlEscape(apSsid_) + "</span></div>";
  html += "<div><strong>AP Password</strong><br><span class='mono'>" + htmlEscape(String(kPortalApPassword)) + "</span></div>";
  html += "<div><strong>MAC Address</strong><br><span class='mono'>" + htmlEscape(macAddress_) + "</span></div>";
  html += "<div><strong>Unique ID</strong><br><span class='mono'>" + htmlEscape(chipSuffix_) + "</span></div>";
  html += "</div>";
  html += "<div class='grid'><div>";
  html += "<form method='post' action='/save'>";
  html += "<label for='device_name'>Custom Device Name</label>";
  html += "<input id='device_name' name='device_name' value='" + htmlEscape(savedDeviceName_) + "' placeholder='Example: left-leg'>";
  html += "<label for='ssid'>Wi-Fi SSID</label>";
  html += "<input id='ssid' name='ssid' value='" + htmlEscape(savedSsid_) + "' placeholder='Enter SSID' required>";
  html += "<label for='password'>Wi-Fi Password</label>";
  html += "<input id='password' name='password' type='password' placeholder='Enter password'>";
  html += "<button type='submit'>Save and Connect</button>";
  html += "</form></div><div>";
  html += "<label for='ssidList'>Nearby Wi-Fi</label>";
  html += "<select id='ssidList' onchange=\"document.getElementById('ssid').value=this.value;\">";
  html += buildNetworkOptions();
  html += "</select>";
  html += "<p>The custom name is used to build the AP SSID and mDNS hostname. The device suffix stays attached so each unit remains unique.</p>";
  html += "<form method='post' action='/forget'>";
  html += "<button type='submit' class='secondary'>Clear Stored Wi-Fi</button>";
  html += "</form></div></div>";
  html += "</div></body></html>";

  return html;
}

String WiFiManagerPortal::buildNetworkOptions()
{
  String options = "<option value=''>Select a scanned Wi-Fi network</option>";
  const int networkCount = WiFi.scanNetworks(false, true);

  if (networkCount <= 0)
  {
    options += "<option value=''>No Wi-Fi found</option>";
    return options;
  }

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

  WiFi.scanDelete();
  return options;
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
