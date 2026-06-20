export interface ChannelCfg { id: string; label: string; boxId: number; channel: number; }
export interface GroupCfg { id: string; label: string; members: string[]; }
export interface SeqStepCfg { timeMs: number; targetType: "channel" | "group"; targetId: string; }
export interface SequenceCfg { id: string; label: string; steps: SeqStepCfg[]; }
export interface ShowConfig {
  version: number;
  channels: ChannelCfg[];
  groups: GroupCfg[];
  sequences: SequenceCfg[];
}

const STORAGE_KEY = "fw.show.config";
const CHANNELS_PER_BOX = 16;
const BOXES = 2;

export function defaultConfig(): ShowConfig {
  const channels: ChannelCfg[] = [];
  for (let boxId = 0; boxId < BOXES; boxId++) {
    for (let channel = 0; channel < CHANNELS_PER_BOX; channel++) {
      const idx = boxId * CHANNELS_PER_BOX + channel;
      channels.push({ id: `c${idx}`, label: `Box ${boxId} · Ch ${channel}`, boxId, channel });
    }
  }
  return { version: 1, channels, groups: [], sequences: [] };
}

export function validateConfig(cfg: ShowConfig): string[] {
  const errors: string[] = [];
  const ids = new Set<string>();
  for (const ch of cfg.channels) {
    if (ids.has(ch.id)) errors.push(`Duplicate channel id "${ch.id}"`);
    ids.add(ch.id);
    if (ch.boxId < 0 || ch.boxId >= BOXES) errors.push(`Channel "${ch.id}" has invalid box ${ch.boxId}`);
    if (ch.channel < 0 || ch.channel >= CHANNELS_PER_BOX) errors.push(`Channel "${ch.id}" has invalid channel ${ch.channel}`);
  }
  const groupIds = new Set<string>();
  for (const g of cfg.groups) {
    if (groupIds.has(g.id)) errors.push(`Duplicate group id "${g.id}"`);
    groupIds.add(g.id);
    for (const m of g.members) if (!ids.has(m)) errors.push(`Group "${g.label}" references unknown channel "${m}"`);
  }
  for (const seq of cfg.sequences) {
    for (const step of seq.steps) {
      if (step.timeMs < 0) errors.push(`Sequence "${seq.label}" has a negative time`);
      if (step.targetType === "channel" && !ids.has(step.targetId))
        errors.push(`Sequence "${seq.label}" references unknown channel "${step.targetId}"`);
      if (step.targetType === "group" && !groupIds.has(step.targetId))
        errors.push(`Sequence "${seq.label}" references unknown group "${step.targetId}"`);
    }
  }
  return errors;
}

export function expandSequence(cfg: ShowConfig, seqId: string): number[] {
  const seq = cfg.sequences.find((s) => s.id === seqId);
  if (!seq) return [];
  const chById = new Map(cfg.channels.map((c) => [c.id, c]));
  const grpById = new Map(cfg.groups.map((g) => [g.id, g]));
  // Stable sort by time; preserve original order for equal times.
  const ordered = seq.steps.map((s, i) => ({ s, i }))
    .sort((a, b) => a.s.timeMs - b.s.timeMs || a.i - b.i)
    .map((x) => x.s);
  const out: number[] = [];
  const pushChannel = (timeMs: number, channelId: string) => {
    const ch = chById.get(channelId);
    if (ch) out.push(timeMs, ch.boxId, ch.channel);
  };
  for (const step of ordered) {
    if (step.targetType === "channel") pushChannel(step.timeMs, step.targetId);
    else {
      const g = grpById.get(step.targetId);
      if (g) for (const m of g.members) pushChannel(step.timeMs, m);
    }
  }
  return out;
}

export function exportConfig(cfg: ShowConfig): string {
  return JSON.stringify(cfg, null, 2);
}

export function importConfig(json: string): ShowConfig {
  const cfg = JSON.parse(json) as ShowConfig;
  const errors = validateConfig(cfg);
  if (errors.length) throw new Error("Invalid config: " + errors.join("; "));
  return cfg;
}

export function saveConfig(cfg: ShowConfig, storage: Storage = globalThis.localStorage): void {
  storage.setItem(STORAGE_KEY, JSON.stringify(cfg));
}

export function loadConfig(storage: Storage = globalThis.localStorage): ShowConfig {
  const raw = storage.getItem(STORAGE_KEY);
  if (!raw) return defaultConfig();
  try {
    const cfg = JSON.parse(raw) as ShowConfig;
    if (validateConfig(cfg).length) return defaultConfig();
    return cfg;
  } catch {
    return defaultConfig();
  }
}
