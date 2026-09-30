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
    puts("tle5012b response ok");
    return 0;
}
