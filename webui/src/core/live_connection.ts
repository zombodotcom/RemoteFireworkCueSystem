import type { SystemConnection } from "./connection";
import { snapshot } from "../stores";

/**
 * LiveConnection — maps SystemConnection to the ESP32 HTTP JSON API.
 *
 * Used when the app is served from the controller (production build).
 * The factory in connection.ts picks SimConnection in DEV and this in PROD.
 *
 * Hardware notes:
 *  - setSwitch / clearEstop have no API equivalent: the physical arm switch
 *    and e-stop reset are operator-controlled hardware actions.
 *  - heartbeat is driven by the start() interval; the UI's manual heartbeat()
 *    call is a no-op here (interval handles it).
 *  - loadSequence stores triples locally; startSequence POSTs /api/run.
 */

const HEARTBEAT_INTERVAL_MS = 1000;
const STATUS_POLL_INTERVAL_MS = 400;

// Minimal subset of what /api/status returns (matches handle_status in web_server.cpp).
interface StatusResponse {
  armed: boolean;
  seqRunning: boolean;
  lastFailedBox: number | null;
}

export class LiveConnection implements SystemConnection {
  private heartbeatTimer: ReturnType<typeof setInterval> | null = null;
  private statusTimer: ReturnType<typeof setInterval> | null = null;
  private connected = true;
  private pendingSteps: number[] = [];

  // ── fire-and-forget POST helpers ──────────────────────────────────────────

  private async post(path: string, body?: object): Promise<void> {
    try {
      await fetch(path, {
        method: "POST",
        headers: body ? { "Content-Type": "application/json" } : {},
        body: body ? JSON.stringify(body) : undefined,
      });
    } catch {
      // Network blips are silently swallowed; the UI will see the next status poll.
    }
  }

  // ── SystemConnection interface ────────────────────────────────────────────

  /** No-op: the physical arm switch is operator hardware, not an API call. */
  setSwitch(_box: number, _on: boolean): void { /* hardware-only */ }

  /** Heartbeat is driven by the start() interval; this is a manual no-op. */
  heartbeat(): void { /* driven by interval */ }

  arm():    void { void this.post("/api/arm"); }
  disarm(): void { void this.post("/api/disarm"); }
  estop():  void { void this.post("/api/estop"); }

  /** No-op: clearEstop has no API equivalent; the arm command re-arms after estop. */
  clearEstop(): void { /* no API endpoint */ }

  fire(box: number, ch: number): void {
    void this.post("/api/fire", { box, channel: ch });
  }

  /**
   * Stores triples locally; returns the cue count (floored).
   * The real device limit is checked server-side on /api/run; we report the
   * submitted count here and the UI may see truncation only if the POST fails.
   */
  loadSequence(triples: number[]): number {
    this.pendingSteps = triples.slice();
    return Math.floor(triples.length / 3);
  }

  /**
   * POSTs the last-loaded sequence to /api/run as [[timeMs,box,channel],...].
   */
  startSequence(): void {
    const triples = this.pendingSteps;
    const steps: [number, number, number][] = [];
    for (let i = 0; i + 2 < triples.length; i += 3) {
      steps.push([triples[i], triples[i + 1], triples[i + 2]]);
    }
    void this.post("/api/run", { steps });
  }

  stopSequence(): void { void this.post("/api/stop"); }

  /** Begin heartbeat + status-poll intervals. */
  start(): void {
    if (this.heartbeatTimer) return;

    this.heartbeatTimer = setInterval(() => {
      if (this.connected) void this.post("/api/heartbeat");
    }, HEARTBEAT_INTERVAL_MS);

    this.statusTimer = setInterval(() => {
      void this.pollStatus();
    }, STATUS_POLL_INTERVAL_MS);

    // Poll immediately so the UI snaps to live state on mount.
    void this.pollStatus();
  }

  /** Clear both intervals. */
  stop(): void {
    if (this.heartbeatTimer) { clearInterval(this.heartbeatTimer); this.heartbeatTimer = null; }
    if (this.statusTimer)    { clearInterval(this.statusTimer);    this.statusTimer    = null; }
  }

  /** When false, stop sending heartbeats (simulate operator disconnect). */
  setConnected(on: boolean): void {
    this.connected = on;
    if (!on) this.stop();
  }

  // ── status polling ────────────────────────────────────────────────────────

  private async pollStatus(): Promise<void> {
    try {
      const res = await fetch("/api/status");
      if (!res.ok) return;
      const data = (await res.json()) as StatusResponse;
      this.applyStatus(data);
    } catch {
      // Network error — leave snapshot stale; UI shows last known state.
    }
  }

  /**
   * Map the controller's flat status JSON into the Snapshot/BoxView shape the UI reads.
   *
   * The live controller does not report per-box or per-channel state beyond global
   * armed + seqRunning, so we synthesise a single "box 0" placeholder that mirrors
   * the global state. The channel grid stays at not-firing (hardware fires physically;
   * the browser has no feedback channel for that).
   *
   * Field mapping (from handle_status in web_server.cpp):
   *   armed         → boxes[0].armed
   *   seqRunning    → snapshot.seqRunning
   *   lastFailedBox → ignored (no BoxView field for it)
   */
  private applyStatus(s: StatusResponse): void {
    const CHANNELS = 16;
    const channels = Array.from({ length: CHANNELS }, () => ({ firing: false, msLeft: 0 }));

    snapshot.set({
      now: Date.now(),
      seqRunning: s.seqRunning,
      boxes: [
        {
          id: 0,
          switchOn: s.armed,   // proxy: if armed, switch is logically on
          armed: s.armed,
          estopped: false,     // no API field; estop is only observable on hardware
          canFire: s.armed && !s.seqRunning,
          channels,
        },
      ],
    });
  }
}
