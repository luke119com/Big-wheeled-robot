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
  void loadCredentials();
  void saveCredentials(const String &ssid, const String &password);
  void clearCredentials();
  bool connectToSavedNetwork();
  bool connectToNetwork(const String &ssid, const String &password, bool keepApAlive);
  void startAccessPoint();
  void stopAccessPoint();
  void ensureWebServerRunning();
  void handleRoot();
  void handleSave();
  void handleForget();
  void handleNotFound();
  void processPendingCredentials();
  String buildRootPage();
  String buildNetworkOptions();
  String getModeLabel() const;
  String getConnectionDetails() const;
  static String htmlEscape(const String &value);

  String savedSsid_;
  String savedPassword_;
  String pendingSsid_;
  String pendingPassword_;
  String statusMessage_;
  bool apModeActive_ = false;
  bool dnsActive_ = false;
  bool serverStarted_ = false;
  bool pendingCredentials_ = false;
};

extern WiFiManagerPortal wifiManagerPortal;

#endif
