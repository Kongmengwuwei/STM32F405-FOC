#include "tle5012b_protocol.h"
#include <assert.h>
#include <stdio.h>

static uint8_t reference_crc(uint16_t command,uint16_t data) {
    uint8_t crc=0xff;
    const uint8_t bytes[4]={(uint8_t)(command>>8),(uint8_t)command,
                           (uint8_t)(data>>8),(uint8_t)data};
    for(unsigned n=0;n<4;n++) {
        crc^=bytes[n];
        for(unsigned bit=0;bit<8;bit++)
            crc=(uint8_t)((crc<<1)^((crc&0x80)?0x1d:0));
    }
    return crc^0xff;
}
int main(void)
{
    const uint16_t commands[]={0x8021,0x8001,0};
    for(unsigned n=0;n<3;n++)for(unsigned data=0;data<65536;data++)
        assert(tle5012b_crc8(commands[n],(uint16_t)data)==reference_crc(commands[n],(uint16_t)data));
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
