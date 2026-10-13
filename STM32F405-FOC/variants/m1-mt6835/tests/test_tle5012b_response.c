#include "tle5012b_protocol.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    /* Frames captured from the two wired encoders during the dual-board test. */
    assert(tle5012b_angle_sample_valid_sensor(0x8021u, 0xa3c1u, 0xfe93u, 0u));
    assert(tle5012b_angle_sample_valid_sensor(0x8021u, 0x89c5u, 0xf750u, 3u));
    assert(!tle5012b_angle_sample_valid_sensor(0x8021u, 0x89c5u, 0xf750u, 0u));
    assert(!tle5012b_angle_sample_valid_sensor(0x8021u, 0x89c5u, 0xf751u, 3u));
    assert(!tle5012b_angle_sample_valid_sensor(0x8021u, 0x89c5u, 0xf750u, 4u));
    for (unsigned bits = 0; bits < 16u; ++bits) {
        unsigned number = tle5012b_safety_sensor_number((uint16_t)(bits << 8));
        if (bits == 14u || bits == 13u || bits == 11u || bits == 7u) {
            assert(number < 4u);
            assert(tle5012b_safety_sensor_response((uint16_t)(bits << 8), number));
        } else assert(number == 4u);
    }
    assert(tle5012b_safety_sensor_number(0xfe2au) == 0u);
    assert(tle5012b_safety_sensor_number(0xf750u) == 3u);
    puts("tle5012b response ok");
    return 0;
}
