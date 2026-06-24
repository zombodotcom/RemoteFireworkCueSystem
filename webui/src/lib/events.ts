export type Severity = 0 | 1 | 2;
export interface LogEvent { seq: number; t: number; sev: Severity; msg: string; }

/** Merge incoming events into existing: dedup by seq, sort ascending, keep newest `cap`. */
export function mergeEvents(existing: LogEvent[], incoming: LogEvent[], cap = 200): LogEvent[] {
  const bySeq = new Map<number, LogEvent>();
  for (const ev of existing) bySeq.set(ev.seq, ev);
  for (const ev of incoming) bySeq.set(ev.seq, ev);   // incoming wins on collision
  const merged = [...bySeq.values()].sort((a, b) => a.seq - b.seq);
  return merged.length > cap ? merged.slice(merged.length - cap) : merged;
}

export function maxSeq(events: LogEvent[]): number {
  let m = 0;
  for (const ev of events) if (ev.seq > m) m = ev.seq;
  return m;
}
