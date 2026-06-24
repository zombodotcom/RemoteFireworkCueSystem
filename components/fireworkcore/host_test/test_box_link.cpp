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

struct RecObs : public BoxLinkObserver {
    int sent=0, ack=0, retry=0, failed=0;
    uint32_t lastLatency=0, lastAttempt=0, lastMax=0;
    void onFireSent(uint8_t, uint8_t, uint32_t) override { sent++; }
    void onFireAck(uint8_t, uint8_t, uint32_t, uint32_t lat) override { ack++; lastLatency=lat; }
    void onFireRetry(uint8_t, uint8_t, uint32_t, uint8_t a, uint8_t m) override { retry++; lastAttempt=a; lastMax=m; }
    void onFireFailed(uint8_t, uint8_t, uint32_t) override { failed++; }
};
void test_observer_fire_and_ack_latency() {
    FakeTransport t; BoxLink link(t); RecObs o; link.setObserver(&o);
    uint32_t id = link.fire(0, 3, 100);
    CHECK_EQ(o.sent, 1);
    link.onAck(id, 112);                 // 12 ms after send
    CHECK_EQ(o.ack, 1);
    CHECK_EQ((int)o.lastLatency, 12);
}
void test_observer_retry_then_fail() {
    FakeTransport t; BoxLink link(t); RecObs o; link.setObserver(&o);  // ackTimeout=120, maxRetries=3
    link.fire(0, 3, 0);
    uint32_t now = 0;
    for (int i=0;i<5;i++){ now += 130; link.tick(now); }
    CHECK_EQ(o.retry, 3);                // 3 retries
    CHECK_EQ((int)o.lastMax, 3);
    CHECK_EQ((int)o.lastAttempt, 3);
    CHECK_EQ(o.failed, 1);               // then gave up once
}

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
int main(){RUN(test_fire_sends_once_and_pends);RUN(test_ack_clears_pending);RUN(test_resend_same_id_after_timeout);RUN(test_gives_up_after_max_retries);RUN(test_control_msgs_send_immediately);RUN(test_observer_fire_and_ack_latency);RUN(test_observer_retry_then_fail);return REPORT();}
