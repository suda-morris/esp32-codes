#include <stdio.h>
#include "qrcode.h"


void app_main()
{
    // Create the QR code
    QRCode qrcode;
    uint8_t qrcodeData[qrcode_getBufferSize(3)];
    qrcode_initText(&qrcode, qrcodeData, 3, 0, "HELLO WORLD");

    // Top quiet zone
    printf("\n\n\n\n");

    for (uint8_t y = 0; y < qrcode.size; y++) {
        // Left quiet zone
        printf("        ");
        // Each horizontal module
        for (uint8_t x = 0; x < qrcode.size; x++) {
            // Print each module (UTF-8 \u2588 is a solid block)
            printf(qrcode_getModule(&qrcode, x, y) ? "\u2588\u2588" : "  ");

        }
        printf("\r\n");
    }
    // Bottom quiet zone
    printf("\n\n\n\n");
}
