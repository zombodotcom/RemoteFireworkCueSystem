#include "check.h"
#include "box_link.h"
#include <vector>
using namespace fw;
struct Sent { uint8_t box; CommandPacket pkt; };
struct FakeTransport : public Transport {
    std::vector<Sent> sent;
    void send(uint8_t boxId, const CommandPacket& p) override { sent.push_back({boxId, p}); }
};
static int countType(FakeTransport& t, MsgType ty){int n=0;for(auto&s:t.sent)if((MsgType)s.pkt.type==ty)n++;return n;}

void test_fire_sends_once_and_pends() {
    FakeTransport t; BoxLink link(t);
    uint32_t id = link.fire(0, 3, 0);
    CHECK_EQ(countType(t, MsgType::FIRE), 1);
    CHECK(link.pendingAck());
    CHECK(id != 0);
}
void test_ack_clears_pending() {
    FakeTransport t; BoxLink link(t);
    uint32_t id = link.fire(0, 3, 0);
    link.onAck(id, 10);
    CHECK(!link.pendingAck());
}
void test_resend_same_id_after_timeout() {
    FakeTransport t; BoxLink link(t);            // default ackTimeoutMs=120
    uint32_t id = link.fire(0, 3, 0);
    link.tick(50);  CHECK_EQ(countType(t, MsgType::FIRE), 1);   // not yet
    link.tick(130); CHECK_EQ(countType(t, MsgType::FIRE), 2);   // resent
    CHECK_EQ(t.sent.back().pkt.id, id);                          // SAME id (box dedups)
}
void test_gives_up_after_max_retries() {
    FakeTransport t; BoxLink link(t);            // maxRetries=3
    uint32_t id = link.fire(0, 3, 0);
    uint32_t now = 0;
    for (int i=0;i<5;i++){ now += 130; link.tick(now); }
    CHECK_EQ(countType(t, MsgType::FIRE), 4);    // 1 initial + 3 retries
    CHECK(!link.pendingAck());
    CHECK_EQ(link.lastFailedId(), id);
}
void test_control_msgs_send_immediately() {
    FakeTransport t; BoxLink link(t);
    link.arm(99, 0); link.disarm(0); link.estop(0); link.heartbeat(0);
    CHECK_EQ(countType(t, MsgType::ARM), 1);
    CHECK_EQ(countType(t, MsgType::DISARM), 1);
    CHECK_EQ(countType(t, MsgType::ESTOP), 1);
    CHECK_EQ(countType(t, MsgType::HEARTBEAT), 1);
}
int main(){RUN(test_fire_sends_once_and_pends);RUN(test_ack_clears_pending);RUN(test_resend_same_id_after_timeout);RUN(test_gives_up_after_max_retries);RUN(test_control_msgs_send_immediately);return REPORT();}
