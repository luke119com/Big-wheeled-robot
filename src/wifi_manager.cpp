#include "wifi_manager.h"

#include <DNSServer.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>

namespace
{
constexpr const char *kPrefsNamespace = "wifi";
constexpr const char *kPrefsSsidKey = "ssid";
constexpr const char *kPrefsPassKey = "password";
constexpr const char *kPortalApSsid = "SF_Bipedal_Wheel";
constexpr const char *kPortalApPassword = "87654321";
constexpr uint32_t kConnectTimeoutMs = 15000;
constexpr uint16_t kDnsPort = 53;
constexpr int kMaxScanResults = 12;

Preferences g_preferences;
WebServer g_server(80);
DNSServer g_dnsServer;
}

WiFiManagerPortal wifiManagerPortal;

void WiFiManagerPortal::begin()
{
  WiFi.persistent(false);
  WiFi.setSleep(false);
  setupRoutes();
  loadCredentials();

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

void WiFiManagerPortal::loadCredentials()
{
  if (!g_preferences.begin(kPrefsNamespace, false))
  {
    statusMessage_ = "Failed to open NVS. Wi-Fi settings will not persist.";
    savedSsid_ = "";
    savedPassword_ = "";
    return;
  }

  savedSsid_ = g_preferences.getString(kPrefsSsidKey, "");
  savedPassword_ = g_preferences.getString(kPrefsPassKey, "");
}

void WiFiManagerPortal::saveCredentials(const String &ssid, const String &password)
{
  savedSsid_ = ssid;
  savedPassword_ = password;
  g_preferences.putString(kPrefsSsidKey, ssid);
  g_preferences.putString(kPrefsPassKey, password);
}

void WiFiManagerPortal::clearCredentials()
{
  savedSsid_ = "";
  savedPassword_ = "";
  g_preferences.remove(kPrefsSsidKey);
  g_preferences.remove(kPrefsPassKey);
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
  return true;
}

void WiFiManagerPortal::startAccessPoint()
{
  WiFi.mode(WIFI_AP_STA);

  const IPAddress apIp(192, 168, 4, 1);
  const IPAddress gateway(192, 168, 4, 1);
  const IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(apIp, gateway, subnet);
  WiFi.softAP(kPortalApSsid, kPortalApPassword);

  if (!dnsActive_)
  {
    g_dnsServer.start(kDnsPort, "*", WiFi.softAPIP());
    dnsActive_ = true;
  }

  apModeActive_ = true;
  ensureWebServerRunning();

  Serial.printf("AP mode enabled. SSID: %s, password: %s, IP: %s\r\n",
                kPortalApSsid,
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

void WiFiManagerPortal::handleRoot()
{
  g_server.send(200, "text/html; charset=utf-8", buildRootPage());
}

void WiFiManagerPortal::handleSave()
{
  const String ssid = g_server.arg("ssid");
  const String password = g_server.arg("password");

  if (ssid.isEmpty())
  {
    statusMessage_ = "SSID is required.";
    g_server.send(400, "text/html; charset=utf-8", buildRootPage());
    return;
  }

  pendingSsid_ = ssid;
  pendingPassword_ = password;
  pendingCredentials_ = true;
  statusMessage_ = "Received new Wi-Fi settings. Trying to connect now.";

  g_server.send(200,
                "text/html; charset=utf-8",
                "<!DOCTYPE html><html><head><meta charset='utf-8'>"
                "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                "<title>Wi-Fi Applying</title></head><body><h1>Wi-Fi Applying</h1>"
                "<p>The device is trying to connect to the new Wi-Fi.</p>"
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
  html.reserve(5200);
  const String apInfo = apModeActive_
                            ? "AP portal: SSID <strong>" + htmlEscape(String(kPortalApSsid)) +
                                  "</strong>, password <strong>" + htmlEscape(String(kPortalApPassword)) +
                                  "</strong>, URL <strong>http://" + WiFi.softAPIP().toString() + "</strong>"
                            : "AP portal is disabled while STA is connected.";

  html += "<!DOCTYPE html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>SF Bipedal Wheel Wi-Fi</title>";
  html += "<style>";
  html += "body{font-family:Arial,Helvetica,sans-serif;background:#f5f7fb;color:#1f2937;margin:0;padding:24px;}";
  html += ".card{max-width:760px;margin:0 auto;background:#fff;border-radius:16px;padding:24px;box-shadow:0 10px 30px rgba(15,23,42,.08);}";
  html += "h1{margin-top:0;font-size:28px;}p{line-height:1.5;}label{display:block;margin:14px 0 6px;font-weight:600;}";
  html += "input,select,button{width:100%;padding:12px;border-radius:10px;border:1px solid #cbd5e1;font-size:16px;box-sizing:border-box;}";
  html += "button{background:#0f766e;color:#fff;border:none;font-weight:700;cursor:pointer;margin-top:16px;}";
  html += ".secondary{background:#475569;}.status{padding:12px 14px;border-radius:12px;background:#ecfeff;margin:16px 0;}";
  html += ".grid{display:grid;grid-template-columns:1fr;gap:14px;}@media(min-width:720px){.grid{grid-template-columns:1fr 1fr;}}";
  html += "</style></head><body><div class='card'>";
  html += "<h1>Wi-Fi Setup</h1>";
  html += "<div class='status'><strong>Mode:</strong> " + htmlEscape(getModeLabel()) + "<br>";
  html += "<strong>Status:</strong> " + htmlEscape(statusMessage_) + "<br>";
  html += "<strong>Details:</strong> " + htmlEscape(getConnectionDetails()) + "</div>";
  html += "<div class='grid'><div>";
  html += "<form method='post' action='/save'>";
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
  html += "<p>For a secured network, enter the password again before saving.</p>";
  html += "<form method='post' action='/forget'>";
  html += "<button type='submit' class='secondary'>Clear Stored Settings</button>";
  html += "</form></div></div>";
  html += "<p>";
  html += apInfo;
  html += "</p></div></body></html>";

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
