#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <Arduino.h>

class WiFiManagerPortal
{
public:
  void begin();
  void loop();

private:
  void setupRoutes();
  void loadPreferences();
  void saveCredentials(const String &ssid, const String &password);
  void saveDeviceName(const String &deviceName);
  void clearCredentials();
  void initIdentity();
  void refreshIdentity();
  bool connectToSavedNetwork();
  bool connectToNetwork(const String &ssid, const String &password, bool keepApAlive);
  void startAccessPoint();
  void stopAccessPoint();
  void ensureWebServerRunning();
  void startMdns();
  void stopMdns();
  void handleRoot();
  void handleSave();
  void handleForget();
  void handleNotFound();
  void processPendingCredentials();
  String buildRootPage();
  String buildNetworkOptions();
  String getModeLabel() const;
  String getConnectionDetails() const;
  String getHostUrl() const;
  String getStaUrl() const;
  static String htmlEscape(const String &value);
  static String trimToLength(const String &value, size_t maxLen);
  static String sanitizeHostLabel(const String &value);
  static String sanitizeDisplayName(const String &value);

  String savedSsid_;
  String savedPassword_;
  String savedDeviceName_;
  String pendingSsid_;
  String pendingPassword_;
  String pendingDeviceName_;
  String statusMessage_;
  String chipSuffix_;
  String macAddress_;
  String apSsid_;
  String mdnsHostname_;
  String displayName_;
  bool apModeActive_ = false;
  bool dnsActive_ = false;
  bool mdnsActive_ = false;
  bool serverStarted_ = false;
  bool pendingCredentials_ = false;
};

extern WiFiManagerPortal wifiManagerPortal;

#endif
