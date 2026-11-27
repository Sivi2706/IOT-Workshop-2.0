// Firebase HTTPS transport and Nano-compatible anonymous authentication.
// Students use firebasePut() / firebaseGet() from main.cpp.
#include "firebase.h"
#include "secrets.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <time.h>
#include <ctype.h>
#include <string.h>

static const char GOOGLE_ROOTS[] PROGMEM = R"GTS(
-----BEGIN CERTIFICATE-----
MIIFVzCCAz+gAwIBAgINAgPlk28xsBNJiGuiFzANBgkqhkiG9w0BAQwFADBHMQsw
CQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZpY2VzIExMQzEU
MBIGA1UEAxMLR1RTIFJvb3QgUjEwHhcNMTYwNjIyMDAwMDAwWhcNMzYwNjIyMDAw
MDAwWjBHMQswCQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZp
Y2VzIExMQzEUMBIGA1UEAxMLR1RTIFJvb3QgUjEwggIiMA0GCSqGSIb3DQEBAQUA
A4ICDwAwggIKAoICAQC2EQKLHuOhd5s73L+UPreVp0A8of2C+X0yBoJx9vaMf/vo
27xqLpeXo4xL+Sv2sfnOhB2x+cWX3u+58qPpvBKJXqeqUqv4IyfLpLGcY9vXmX7w
Cl7raKb0xlpHDU0QM+NOsROjyBhsS+z8CZDfnWQpJSMHobTSPS5g4M/SCYe7zUjw
TcLCeoiKu7rPWRnWr4+wB7CeMfGCwcDfLqZtbBkOtdh+JhpFAz2weaSUKK0Pfybl
qAj+lug8aJRT7oM6iCsVlgmy4HqMLnXWnOunVmSPlk9orj2XwoSPwLxAwAtcvfaH
szVsrBhQf4TgTM2S0yDpM7xSma8ytSmzJSq0SPly4cpk9+aCEI3oncKKiPo4Zor8
Y/kB+Xj9e1x3+naH+uzfsQ55lVe0vSbv1gHR6xYKu44LtcXFilWr06zqkUspzBmk
MiVOKvFlRNACzqrOSbTqn3yDsEB750Orp2yjj32JgfpMpf/VjsPOS+C12LOORc92
wO1AK/1TD7Cn1TsNsYqiA94xrcx36m97PtbfkSIS5r762DL8EGMUUXLeXdYWk70p
aDPvOmbsB4om3xPXV2V4J95eSRQAogB/mqghtqmxlbCluQ0WEdrHbEg8QOB+DVrN
VjzRlwW5y0vtOUucxD/SVRNuJLDWcfr0wbrM7Rv1/oFB2ACYPTrIrnqYNxgFlQID
AQABo0IwQDAOBgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4E
FgQU5K8rJnEaK0gnhS9SZizv8IkTcT4wDQYJKoZIhvcNAQEMBQADggIBAJ+qQibb
C5u+/x6Wki4+omVKapi6Ist9wTrYggoGxval3sBOh2Z5ofmmWJyq+bXmYOfg6LEe
QkEzCzc9zolwFcq1JKjPa7XSQCGYzyI0zzvFIoTgxQ6KfF2I5DUkzps+GlQebtuy
h6f88/qBVRRiClmpIgUxPoLW7ttXNLwzldMXG+gnoot7TiYaelpkttGsN/H9oPM4
7HLwEXWdyzRSjeZ2axfG34arJ45JK3VmgRAhpuo+9K4l/3wV3s6MJT/KYnAK9y8J
ZgfIPxz88NtFMN9iiMG1D53Dn0reWVlHxYciNuaCp+0KueIHoI17eko8cdLiA6Ef
MgfdG+RCzgwARWGAtQsgWSl4vflVy2PFPEz0tv/bal8xa5meLMFrUKTX5hgUvYU/
Z6tGn6D/Qqc6f1zLXbBwHSs09dR2CQzreExZBfMzQsNhFRAbd03OIozUhfJFfbdT
6u9AWpQKXCBfTkBdYiJ23//OYb2MI3jSNwLgjt7RETeJ9r/tSQdirpLsQBqvFAnZ
0E6yove+7u7Y/9waLd64NnHi/Hm3lCXRSHNboTXns5lndcEZOitHTtNCjv0xyBZm
2tIMPNuzjsmhDYAPexZ3FL//2wmUspO8IFgV6dtxQ/PeEMMA3KgqlbbC1j+Qa3bb
bP6MvPJwNQzcmRk13NfIRmPVNnGuV/u3gm3c
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIIFVzCCAz+gAwIBAgINAgPlrsWNBCUaqxElqjANBgkqhkiG9w0BAQwFADBHMQsw
CQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZpY2VzIExMQzEU
MBIGA1UEAxMLR1RTIFJvb3QgUjIwHhcNMTYwNjIyMDAwMDAwWhcNMzYwNjIyMDAw
MDAwWjBHMQswCQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZp
Y2VzIExMQzEUMBIGA1UEAxMLR1RTIFJvb3QgUjIwggIiMA0GCSqGSIb3DQEBAQUA
A4ICDwAwggIKAoICAQDO3v2m++zsFDQ8BwZabFn3GTXd98GdVarTzTukk3LvCvpt
nfbwhYBboUhSnznFt+4orO/LdmgUud+tAWyZH8QiHZ/+cnfgLFuv5AS/T3KgGjSY
6Dlo7JUle3ah5mm5hRm9iYz+re026nO8/4Piy33B0s5Ks40FnotJk9/BW9BuXvAu
MC6C/Pq8tBcKSOWIm8Wba96wyrQD8Nr0kLhlZPdcTK3ofmZemde4wj7I0BOdre7k
RXuJVfeKH2JShBKzwkCX44ofR5GmdFrS+LFjKBC4swm4VndAoiaYecb+3yXuPuWg
f9RhD1FLPD+M2uFwdNjCaKH5wQzpoeJ/u1U8dgbuak7MkogwTZq9TwtImoS1mKPV
+3PBV2HdKFZ1E66HjucMUQkQdYhMvI35ezzUIkgfKtzra7tEscszcTJGr61K8Yzo
dDqs5xoic4DSMPclQsciOzsSrZYuxsN2B6ogtzVJV+mSSeh2FnIxZyuWfoqjx5RW
Ir9qS34BIbIjMt/kmkRtWVtd9QCgHJvGeJeNkP+byKq0rxFROV7Z+2et1VsRnTKa
G73VululycslaVNVJ1zgyjbLiGH7HrfQy+4W+9OmTN6SpdTi3/UGVN4unUu0kzCq
gc7dGtxRcw1PcOnlthYhGXmy5okLdWTK1au8CcEYof/UVKGFPP0UJAOyh9OktwID
AQABo0IwQDAOBgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4E
FgQUu//KjiOfT5nK2+JopqUVJxce2Q4wDQYJKoZIhvcNAQEMBQADggIBAB/Kzt3H
vqGf2SdMC9wXmBFqiN495nFWcrKeGk6c1SuYJF2ba3uwM4IJvd8lRuqYnrYb/oM8
0mJhwQTtzuDFycgTE1XnqGOtjHsB/ncw4c5omwX4Eu55MaBBRTUoCnGkJE+M3DyC
B19m3H0Q/gxhswWV7uGugQ+o+MePTagjAiZrHYNSVc61LwDKgEDg4XSsYPWHgJ2u
NmSRXbBoGOqKYcl3qJfEycel/FVL8/B/uWU9J2jQzGv6U53hkRrJXRqWbTKH7QMg
yALOWr7Z6v2yTcQvG99fevX4i8buMTolUVVnjWQye+mew4K6Ki3pHrTgSAai/Gev
HyICc/sgCq+dVEuhzf9gR7A/Xe8bVr2XIZYtCtFenTgCR2y59PYjJbigapordwj6
xLEokCZYCDzifqrXPW+6MYgKBesntaFJ7qBFVHvmJ2WZICGoo7z7GJa7Um8M7YNR
TOlZ4iBgxcJlkoKM8xAfDoqXvneCbT+PHV28SSe9zE8P4c52hgQjxcCMElv924Sg
JPFI/2R80L5cFtHvma3AH/vLrrw4IgYmZNralw4/KBVEqE8AyvCazM90arQ+POuV
7LXTWtiBmelDGDfrs7vRWGJB82bSj6p4lVQgw1oudCvV0b4YacCs1aTPObpRhANl
6WLAYv7YTVWW4tAR+kg0Eeye7QUd5MjWHYbL
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIICCTCCAY6gAwIBAgINAgPluILrIPglJ209ZjAKBggqhkjOPQQDAzBHMQswCQYD
VQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZpY2VzIExMQzEUMBIG
A1UEAxMLR1RTIFJvb3QgUjMwHhcNMTYwNjIyMDAwMDAwWhcNMzYwNjIyMDAwMDAw
WjBHMQswCQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZpY2Vz
IExMQzEUMBIGA1UEAxMLR1RTIFJvb3QgUjMwdjAQBgcqhkjOPQIBBgUrgQQAIgNi
AAQfTzOHMymKoYTey8chWEGJ6ladK0uFxh1MJ7x/JlFyb+Kf1qPKzEUURout736G
jOyxfi//qXGdGIRFBEFVbivqJn+7kAHjSxm65FSWRQmx1WyRRK2EE46ajA2ADDL2
4CejQjBAMA4GA1UdDwEB/wQEAwIBhjAPBgNVHRMBAf8EBTADAQH/MB0GA1UdDgQW
BBTB8Sa6oC2uhYHP0/EqEr24Cmf9vDAKBggqhkjOPQQDAwNpADBmAjEA9uEglRR7
VKOQFhG/hMjqb2sXnh5GmCCbn9MN2azTL818+FsuVbu/3ZL3pAzcMeGiAjEA/Jdm
ZuVDFhOD3cffL74UOO0BzrEXGhF16b0DjyZ+hOXJYKaV11RZt+cRLInUue4X
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIICCTCCAY6gAwIBAgINAgPlwGjvYxqccpBQUjAKBggqhkjOPQQDAzBHMQswCQYD
VQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZpY2VzIExMQzEUMBIG
A1UEAxMLR1RTIFJvb3QgUjQwHhcNMTYwNjIyMDAwMDAwWhcNMzYwNjIyMDAwMDAw
WjBHMQswCQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZpY2Vz
IExMQzEUMBIGA1UEAxMLR1RTIFJvb3QgUjQwdjAQBgcqhkjOPQIBBgUrgQQAIgNi
AATzdHOnaItgrkO4NcWBMHtLSZ37wWHO5t5GvWvVYRg1rkDdc/eJkTBa6zzuhXyi
QHY7qca4R9gq55KRanPpsXI5nymfopjTX15YhmUPoYRlBtHci8nHc8iMai/lxKvR
HYqjQjBAMA4GA1UdDwEB/wQEAwIBhjAPBgNVHRMBAf8EBTADAQH/MB0GA1UdDgQW
BBSATNbrdP9JNqPV2Py1PsVq8JQdjDAKBggqhkjOPQQDAwNpADBmAjEA6ED/g94D
9J+uHXqnLrmvT/aDHQ4thQEd0dlq7A/Cr8deVl5c1RxYIigL9zC2L7F8AjEA8GE8
p/SgguMh1YQdc4acLa/KNJvxn7kjNuK8YAOdgLOaVsjh4rsUecrNIdSUtUlD
-----END CERTIFICATE-----
)GTS";


static Preferences prefs;
static bool prefsOpen = false;
static String idToken, refreshToken, uid;
static uint32_t renewAtMs = 0;
static bool signedIn = false;

// Send HTTP over server-verified TLS. Does NOT use setInsecure().
// For Firebase PUT ?print=silent, HTTP 204 has NO response body.
static int request(const String& url, const char* method,
                   const String& outgoing, String& incoming,
                   const char* contentType = "application/json") {
  WiFiClientSecure client;
  client.setCACert(GOOGLE_ROOTS);
  HTTPClient http;
  incoming = "";
  if (!http.begin(client, url)) return -100;
  http.setTimeout(8000);
  http.setReuse(false);
  if (strcmp(method, "GET") != 0) http.addHeader("Content-Type", contentType);
  int status;
  if (strcmp(method, "GET") == 0) status = http.GET();
  else status = http.sendRequest(method, (uint8_t*)outgoing.c_str(), outgoing.length());
  // Avoid getString() on HTTP 204 to prevent the long blocking delay.
  if (status > 0 && status != 204 && status != 304) incoming = http.getString();
  http.end();
  if (status < 200 || status >= 300) {
    Serial.printf("# HTTPS %s returned %d\n",method,status);
    if (status > 0 && incoming.length())
      Serial.println(incoming.substring(0,160));
  }
  return status;
}

static String encode(const String& s) {
  String out;
  const char* hex="0123456789ABCDEF";
  for (size_t i=0; i<s.length(); ++i) {
    uint8_t c=(uint8_t)s[i];
    if (isalnum(c) || c=='-' || c=='_' || c=='.' || c=='~') out += (char)c;
    else { out+='%'; out+=hex[c>>4]; out+=hex[c&15]; }
  }
  return out;
}

static bool parseAuth(const String& json, bool fromRefresh) {
  JsonDocument doc;
  if (deserializeJson(doc,json)) return false;
  String newId = doc[fromRefresh ? "id_token" : "idToken"].as<String>();
  String newRefresh = doc[fromRefresh ? "refresh_token" : "refreshToken"].as<String>();
  String newUid = doc[fromRefresh ? "user_id" : "localId"].as<String>();
  int duration = doc[fromRefresh ? "expires_in" : "expiresIn"].as<String>().toInt();
  if (!duration) duration=3600;
  if (newId.isEmpty() || newRefresh.isEmpty() || newUid.isEmpty()) return false;
  idToken=newId; refreshToken=newRefresh; uid=newUid;
  prefs.putString("refresh",refreshToken);
  renewAtMs = millis() + (uint32_t)(duration>180 ? duration-120 : 60)*1000UL;
  signedIn=true;
  Serial.println("# Firebase UID: " + uid);
  return true;
}

static bool signIn() {
  String reply;
  String prior = prefs.isKey("refresh") ? prefs.getString("refresh", "") : "";
  if (prior.length()) {
    String url=String("https://securetoken.googleapis.com/v1/token?key=")+FIREBASE_API_KEY;
    String body=String("grant_type=refresh_token&refresh_token=")+encode(prior);
    int status=request(url,"POST",body,reply,"application/x-www-form-urlencoded");
    if (status>=200 && status<300 && parseAuth(reply,true)) return true;
    if (status!=400 && status!=401) return false;  // keep existing identity on network error
    Serial.println("# Refresh token expired/revoked; creating anonymous login");
  }
  String url=String("https://identitytoolkit.googleapis.com/v1/accounts:signUp?key=")+FIREBASE_API_KEY;
  int status=request(url,"POST","{\"returnSecureToken\":true}",reply);
  return status>=200 && status<300 && parseAuth(reply,false);
}

bool wifiConnect(uint32_t timeoutMs) {
  if (WiFi.status()==WL_CONNECTED) return true;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID,WIFI_PASSWORD);
  Serial.print("# Connecting Wi-Fi");
  uint32_t start=millis();
  while (WiFi.status()!=WL_CONNECTED && millis()-start<timeoutMs) {
    Serial.print('.'); delay(350);
  }
  Serial.println();
  if (WiFi.status()!=WL_CONNECTED) { Serial.println("# Wi-Fi failed");return false; }
  Serial.println("# Wi-Fi connected: " + WiFi.localIP().toString());
  return true;
}

static bool syncClock() {
  // TLS certificate validation requires the correct current time.
  if (time(nullptr) > 1793491200) return true; // after 2026-11-01 UTC
  configTime(0,0,"pool.ntp.org","time.google.com");
  uint32_t start=millis();
  while (time(nullptr) <= 1793491200 && millis()-start<15000) delay(200);
  if (time(nullptr) <= 1793491200) { Serial.println("# NTP time unavailable");return false; }
  time_t now=time(nullptr);
  Serial.print("# UTC clock: ");Serial.print(ctime(&now));
  return true;
}

bool firebaseBegin() {
  if (!prefsOpen) {prefsOpen=prefs.begin("fb_anon",false);}
  if (!prefsOpen) {Serial.println("# Cannot open NVS token storage");return false;}
  if (!wifiConnect()) return false;
  if (!syncClock()) return false;
  if (!signIn()) {Serial.println("# Firebase login failed");return false;}
  return true;
}

bool firebaseReady() {
  if (WiFi.status()!=WL_CONNECTED) {
    signedIn=false;
    if (!wifiConnect(6000)) return false;
  }
  if (!syncClock()) return false;
  if (signedIn && (int32_t)(millis()-renewAtMs)<0) return true;
  if (!prefsOpen) return firebaseBegin();
  signedIn=false;
  return signIn();
}

String firebaseUID() {return uid;}

static String dataURL(const String& path, bool silent) {
  String p=path;
  if (!p.startsWith("/")) p="/"+p;
  return String(FIREBASE_DB_URL)+p+".json?auth="+idToken+
     (silent ? "&print=silent" : "");
}

int firebasePut(const String& path, const String& json) {
  if (!firebaseReady()) return -200;
  String response;
  int status=request(dataURL(path,true),"PUT",json,response);
  if (status==401) signedIn=false;
  return status;
}

int firebaseGet(const String& path, String& response) {
  response="";
  if (!firebaseReady()) return -200;
  int status=request(dataURL(path,false),"GET","",response);
  if (status==401) signedIn=false;
  return status;
}
