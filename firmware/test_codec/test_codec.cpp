#include "check.h"
#include "expander_codec.h"
using namespace fwbox;

void test_alloff_word_per_polarity() {
    CHECK_EQ(allOffWord(FireLevel::ACTIVE_HIGH), 0x0000u);
    CHECK_EQ(allOffWord(FireLevel::ACTIVE_LOW),  0xFFFFu);
}
void test_apply_active_high() {
    uint16_t w = allOffWord(FireLevel::ACTIVE_HIGH); // 0x0000
    w = applyChannel(w, 0, true, FireLevel::ACTIVE_HIGH);
    CHECK_EQ(w, 0x0001u);                              // bit0 high = on
    w = applyChannel(w, 0, false, FireLevel::ACTIVE_HIGH);
    CHECK_EQ(w, 0x0000u);
    w = applyChannel(w, 15, true, FireLevel::ACTIVE_HIGH);
    CHECK_EQ(w, 0x8000u);
}
void test_apply_active_low() {
    uint16_t w = allOffWord(FireLevel::ACTIVE_LOW);  // 0xFFFF
    w = applyChannel(w, 0, true, FireLevel::ACTIVE_LOW);
    CHECK_EQ(w, 0xFFFEu);                              // bit0 low = on
    w = applyChannel(w, 0, false, FireLevel::ACTIVE_LOW);
    CHECK_EQ(w, 0xFFFFu);
}
void test_out_of_range_no_change() {
    uint16_t w = 0x1234u;
    CHECK_EQ(applyChannel(w, 16, true, FireLevel::ACTIVE_HIGH), 0x1234u);
}

int main() {
    RUN(test_alloff_word_per_polarity);
    RUN(test_apply_active_high);
    RUN(test_apply_active_low);
    RUN(test_out_of_range_no_change);
    return REPORT();
}
