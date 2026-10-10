# Defaults shared by the top-level and standalone firmware entry points.
# Explicit -D values and preset cacheVariables still override these defaults.
set(FOC_PORT "M1" CACHE STRING "Three-phase power connector: M0 or M1")
set(FOC_ENCODER "TLE5012B" CACHE STRING "Angle sensor model")
set(FOC_MOTOR "L4225_6S420_A" CACHE STRING "Motor model")
set(FOC_LOAD_PROFILE "FREE_SHAFT" CACHE STRING "Mechanical load profile: FREE_SHAFT or PLASTIC_ARM")
set(FOC_INSTALLATION_ID "4" CACHE STRING "Physical sensor/magnet/phase installation: 1..15")
set(FOC_PROTECTION "WARN" CACHE STRING "Threshold policy: WARN or TRIP")
set(FOC_USB_DIVIDER "2" CACHE STRING "USB frame every N current samples")
