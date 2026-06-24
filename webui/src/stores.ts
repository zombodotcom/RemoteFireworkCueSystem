import { writable } from "svelte/store";
import { loadConfig, saveConfig, type ShowConfig } from "./lib/config";
import type { SystemConnection } from "./core/connection";
import type { LogEvent } from "./lib/events";

export interface ChannelView { firing: boolean; msLeft: number; fired: boolean; }
export interface BoxView { id: number; switchOn: boolean; armed: boolean; estopped: boolean; canFire: boolean; linkAlive: boolean; rssi: number | null; channels: ChannelView[]; }
export interface Snapshot { now: number; seqRunning: boolean; boxes: BoxView[]; }

export interface Diag {
  uptimeMs: number; freeHeap: number; apClients: number;
  fired: number; acked: number; failed: number; retries: number; lastAckMs: number;
}
export interface Fault { active: boolean; msg: string; }

export const snapshot = writable<Snapshot>({ now: 0, seqRunning: false, boxes: [] });

export const config = writable<ShowConfig>(loadConfig());
config.subscribe((c) => { try { saveConfig(c); } catch { /* storage unavailable */ } });

export const connection = writable<SystemConnection | null>(null);

export const events = writable<LogEvent[]>([]);
export const diag = writable<Diag | null>(null);
export const fault = writable<Fault>({ active: false, msg: "" });
