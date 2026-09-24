#include "bsp_motor_record.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    record_t record = {.version = 3u,
        .poles = (FOC_CALIBRATION_ID << 16) | FOC_POLE_PAIRS,
        .cal = {.zero = 1.0f, .direction = 1}, .magic = 0x464f4331u};
    record.checksum = checksum(&record);
    assert(record_valid(&record));
    record.poles ^= 1u << 28; /* Another power port. */
    record.checksum = checksum(&record);
    assert(!record_valid(&record));
    record.poles ^= 1u << 28;
    record.poles ^= 1u << 24; /* Another motor model. */
    record.checksum = checksum(&record);
    assert(!record_valid(&record));
    record.poles ^= 1u << 24;
    record.poles ^= 1u << 20; /* Another encoder model. */
    record.checksum = checksum(&record);
    assert(!record_valid(&record));
    record.poles ^= 1u << 20;
    record.poles ^= 1u << 16; /* Another magnet/phase installation. */
    record.checksum = checksum(&record);
    assert(!record_valid(&record));
    record.poles ^= 1u << 16;
    record.version = 2u;
    record.checksum = checksum(&record);
    assert(!record_valid(&record));
    record.version = 1u; record.poles = 7u;
    record.checksum = checksum(&record);
    assert(!record_valid(&record));
    puts("PASS: four-part calibration identity and legacy rejection");
    return 0;
}
