import { writable } from "svelte/store";
import { loadConfig, saveConfig, type ShowConfig } from "./lib/config";

export interface ChannelView { firing: boolean; msLeft: number; }
export interface BoxView { id: number; switchOn: boolean; armed: boolean; estopped: boolean; canFire: boolean; channels: ChannelView[]; }
export interface Snapshot { now: number; seqRunning: boolean; boxes: BoxView[]; }

export const snapshot = writable<Snapshot>({ now: 0, seqRunning: false, boxes: [] });

export const config = writable<ShowConfig>(loadConfig());
config.subscribe((c) => { try { saveConfig(c); } catch { /* storage unavailable */ } });
