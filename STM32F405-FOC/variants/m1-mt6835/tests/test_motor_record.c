#include "bsp_motor_record.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    record_t record = {.version = 2u,
        .poles = (FOC_CALIBRATION_ID << 16) | FOC_POLE_PAIRS,
        .cal = {.zero = 1.0f, .direction = 1}, .magic = 0x464f4331u};
    record.checksum = checksum(&record);
    assert(record_valid(&record));
    record.poles ^= 1u << 24; /* Another board, same pole count. */
    record.checksum = checksum(&record);
    assert(!record_valid(&record));
    record.poles ^= 1u << 24;
    record.poles ^= 1u << 16; /* Another motor or encoder installation. */
    record.checksum = checksum(&record);
    assert(!record_valid(&record));
    record.version = 1u; record.poles = 7u;
    record.checksum = checksum(&record);
#ifdef FOC_BOARD_M0
    assert(!record_valid(&record));
#else
    assert(record_valid(&record)); /* Original M1 Flash records remain usable. */
#endif
    puts("PASS: calibration identity and legacy compatibility");
    return 0;
}
