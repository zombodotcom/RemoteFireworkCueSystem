#include "check.h"
#include "crc32.h"

void test_crc32_known_vector() {
    const char* s = "123456789";
    CHECK_EQ(fw::crc32(reinterpret_cast<const uint8_t*>(s), 9), 0xCBF43926u);
}
void test_crc32_empty_is_zero() {
    CHECK_EQ(fw::crc32(nullptr, 0), 0x00000000u);
}

int main() {
    RUN(test_crc32_known_vector);
    RUN(test_crc32_empty_is_zero);
    return REPORT();
}
