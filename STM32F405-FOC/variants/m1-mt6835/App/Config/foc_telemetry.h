#ifndef FOC_TELEMETRY_H
#define FOC_TELEMETRY_H

/* USB line coding and logging cadence are independent of control timing.
 * USART2 keeps its existing 2 Mbps setting and 2 kHz frame cadence. */
#define FOC_USB_BAUDRATE 1000000u
#ifndef FOC_USB_DIVIDER
#define FOC_USB_DIVIDER 2u
#endif
#if FOC_USB_DIVIDER < 1 || FOC_USB_DIVIDER > 1000
#error FOC_USB_DIVIDER must be in 1..1000
#endif

#endif
