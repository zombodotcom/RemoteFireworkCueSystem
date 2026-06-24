import { describe, it, expect } from "vitest";
import { mergeEvents, maxSeq, type LogEvent } from "../src/lib/events";

const e = (seq: number, sev: 0 | 1 | 2 = 0): LogEvent => ({ seq, t: seq * 10, sev, msg: `m${seq}` });

describe("mergeEvents", () => {
  it("appends new events sorted by seq", () => {
    const r = mergeEvents([e(1), e(2)], [e(3), e(4)]);
    expect(r.map((x) => x.seq)).toEqual([1, 2, 3, 4]);
  });
  it("dedups by seq (incoming wins, no duplicates)", () => {
    const r = mergeEvents([e(1), e(2)], [e(2), e(3)]);
    expect(r.map((x) => x.seq)).toEqual([1, 2, 3]);
  });
  it("caps to the newest N", () => {
    const existing = [e(1), e(2), e(3)];
    const r = mergeEvents(existing, [e(4), e(5)], 3);
    expect(r.map((x) => x.seq)).toEqual([3, 4, 5]);
  });
  it("maxSeq returns the highest seq, 0 when empty", () => {
    expect(maxSeq([e(2), e(5), e(3)])).toBe(5);
    expect(maxSeq([])).toBe(0);
  });
});
