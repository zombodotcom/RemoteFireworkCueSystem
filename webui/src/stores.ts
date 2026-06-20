import { writable } from "svelte/store";

export interface ChannelView { firing: boolean; msLeft: number; }
export interface BoxView { id: number; switchOn: boolean; armed: boolean; estopped: boolean; canFire: boolean; channels: ChannelView[]; }
export interface Snapshot { now: number; seqRunning: boolean; boxes: BoxView[]; }

export const snapshot = writable<Snapshot>({ now: 0, seqRunning: false, boxes: [] });
