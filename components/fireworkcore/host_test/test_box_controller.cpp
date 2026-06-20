#include "check.h"
#include "box_controller.h"
#include "fake_channel_driver.h"
using namespace fw;

void test_begin_is_safe_and_alloff() {
    FakeChannelDriver drv;
    BoxController box(drv, BoxConfig{});
    box.begin();
    CHECK_EQ((int)box.state(), (int)BoxState::SAFE);   // never boot armed
    CHECK(drv.allOffCount >= 1);                        // outputs forced off at boot
    CHECK_EQ(drv.countOn(), 0);
}

int main() {
    RUN(test_begin_is_safe_and_alloff);
    return REPORT();
}
