#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif


void usb2_phex(uint8_t n) {
	Serial.printf("%02X", n);
}
void usb2_phex32(uint32_t n) {
	Serial.printf("%08X", n);
}

void usb2_print(const char *psz) {
	Serial.print(psz);
}
#ifdef __cplusplus
}
#endif

