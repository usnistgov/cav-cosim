import { Immutable, MessageEvent, PanelExtensionContext, SettingsTreeAction, Time } from "@foxglove/extension";
import { ReactElement, useCallback, useEffect, useLayoutEffect, useMemo, useRef, useState } from "react";
import { createRoot } from "react-dom/client";

type PanelState = {
  topics: string;          // comma-separated list of string topics
  windowSeconds: number;   // following-view width to match Plot panels
  leftPadPx: number;       // left margin to align with Plot y-axis area
  rightPadPx: number;      // right margin (small for parity with Plot)
  bottomPadPx: number;     // space reserved for x-axis tick labels
  rowGapPx: number;        // vertical gap between topic rows
  showTopicLabel: boolean; // print short topic label inside the bar
};

const DEFAULT_STATE: PanelState = {
  topics: "/metrics/perception_state, /metrics/state",
  windowSeconds: 20,
  leftPadPx: 50,
  rightPadPx: 8,
  bottomPadPx: 20,
  rowGapPx: 6,
  showTopicLabel: true,
};

// Color palette assigned in first-seen order per topic.
const COLOR_PALETTE = [
  "#6b7280", // gray   — usually NO_DETECTION / CRUISE
  "#3b82f6", // blue   — DETECTED
  "#ef4444", // red    — IN_PATH / BRAKE
  "#f59e0b", // amber
  "#10b981", // green
  "#8b5cf6", // purple
  "#ec4899", // pink
  "#0ea5e9", // sky
];

type StateEvent = { t: number; v: string };
type TopicSeries = {
  topic: string;
  events: StateEvent[];           // sorted by t
  colorByValue: Map<string, string>;
};

function timeToSeconds(t: Time): number {
  return t.sec + t.nsec * 1e-9;
}

function shortTopic(topic: string): string {
  const parts = topic.split("/").filter(Boolean);
  return (parts[parts.length - 1] ?? topic).toUpperCase();
}

function StateTimelinePanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [state, setState] = useState<PanelState>(() => ({
    ...DEFAULT_STATE,
    ...(context.initialState as Partial<PanelState> | undefined),
  }));
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const seriesRef = useRef<Map<string, TopicSeries>>(new Map());
  const currentTimeRef = useRef<number>(0);
  const [, forceRender] = useState(0);

  const topicList = useMemo(
    () => state.topics.split(",").map((t) => t.trim()).filter((t) => t.length > 0),
    [state.topics],
  );

  // Reset series when topic list changes.
  useEffect(() => {
    const next = new Map<string, TopicSeries>();
    for (const t of topicList) {
      const prev = seriesRef.current.get(t);
      next.set(t, prev ?? { topic: t, events: [], colorByValue: new Map() });
    }
    seriesRef.current = next;
  }, [topicList]);

  const settingsActionHandler = useCallback(
    (action: SettingsTreeAction) => {
      if (action.action !== "update") {
        return;
      }
      const fieldName = action.payload.path[action.payload.path.length - 1] as keyof PanelState;
      if (!fieldName) {
        return;
      }
      const v = action.payload.value as PanelState[keyof PanelState];
      setState((prev) => {
        const next = { ...prev, [fieldName]: v };
        context.saveState(next);
        return next;
      });
    },
    [context],
  );

  useEffect(() => {
    context.updatePanelSettingsEditor({
      actionHandler: settingsActionHandler,
      nodes: {
        general: {
          label: "General",
          fields: {
            topics: {
              label: "Topics (comma-separated)",
              input: "string",
              value: state.topics,
              placeholder: "/metrics/perception_state, /metrics/state",
            },
            windowSeconds: { label: "Window (s)", input: "number", value: state.windowSeconds, min: 1 },
            showTopicLabel: { label: "Show topic label", input: "boolean", value: state.showTopicLabel },
          },
        },
        layout: {
          label: "Layout",
          fields: {
            leftPadPx: { label: "Left padding (px)", input: "number", value: state.leftPadPx, min: 0 },
            rightPadPx: { label: "Right padding (px)", input: "number", value: state.rightPadPx, min: 0 },
            bottomPadPx: { label: "Bottom padding (px)", input: "number", value: state.bottomPadPx, min: 0 },
            rowGapPx: { label: "Row gap (px)", input: "number", value: state.rowGapPx, min: 0 },
          },
        },
      },
    });
  }, [context, settingsActionHandler, state]);

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);

      // Track playback / live time as the right edge of the window.
      const ct = renderState.currentTime;
      if (ct) {
        currentTimeRef.current = timeToSeconds(ct);
      }

      const frame = renderState.currentFrame as Immutable<MessageEvent[]> | undefined;
      if (frame && frame.length > 0) {
        for (const m of frame) {
          const series = seriesRef.current.get(m.topic);
          if (!series) {
            continue;
          }
          const raw = (m.message as { data?: unknown }).data;
          const v = typeof raw === "string" ? raw : String(raw ?? "");
          if (!v) {
            continue;
          }
          const t = timeToSeconds(m.receiveTime);
          // Update right-edge time from latest message in case currentTime lags.
          if (t > currentTimeRef.current) {
            currentTimeRef.current = t;
          }
          if (!series.colorByValue.has(v)) {
            const idx = series.colorByValue.size;
            series.colorByValue.set(v, COLOR_PALETTE[idx % COLOR_PALETTE.length]!);
          }
          const last = series.events[series.events.length - 1];
          if (!last || last.v !== v) {
            series.events.push({ t, v });
          }
          // Prune anything older than the window + slack.
          const cutoff = currentTimeRef.current - state.windowSeconds - 5;
          while (series.events.length > 1 && series.events[1]!.t < cutoff) {
            series.events.shift();
          }
        }
      }

      forceRender((n) => n + 1);
    };
    context.watch("currentFrame");
    context.watch("currentTime");
    context.watch("topics");

    if (topicList.length > 0) {
      context.subscribe(topicList.map((t) => ({ topic: t })));
    }

    return () => {
      try {
        context.unsubscribeAll();
      } catch {
        /* noop */
      }
    };
  }, [context, topicList, state.windowSeconds]);

  // Draw the canvas after each render.
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) {
      return;
    }
    const ctx = canvas.getContext("2d");
    if (!ctx) {
      return;
    }

    const dpr = window.devicePixelRatio || 1;
    const cssW = canvas.clientWidth;
    const cssH = canvas.clientHeight;
    if (canvas.width !== cssW * dpr || canvas.height !== cssH * dpr) {
      canvas.width = cssW * dpr;
      canvas.height = cssH * dpr;
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, cssW, cssH);

    const { leftPadPx, rightPadPx, bottomPadPx, rowGapPx, windowSeconds, showTopicLabel } = state;
    const plotX = leftPadPx;
    const plotW = Math.max(10, cssW - leftPadPx - rightPadPx);
    const plotTop = 4;
    const plotBottom = cssH - bottomPadPx;
    const plotH = Math.max(10, plotBottom - plotTop);

    const tEnd = currentTimeRef.current;
    const tStart = tEnd - windowSeconds;

    const N = topicList.length;
    if (N === 0) {
      ctx.fillStyle = "#888";
      ctx.font = "12px system-ui, sans-serif";
      ctx.fillText("Configure topics in the settings panel", 12, cssH / 2);
      return;
    }

    const rowH = (plotH - rowGapPx * (N - 1)) / N;

    // Background
    ctx.fillStyle = "#1e1e1e";
    ctx.fillRect(0, 0, cssW, cssH);

    // Vertical grid (5 ticks)
    ctx.strokeStyle = "#3a3a3a";
    ctx.lineWidth = 1;
    ctx.fillStyle = "#9ca3af";
    ctx.font = "10px system-ui, sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "top";
    const ticks = 5;
    for (let i = 0; i <= ticks; i++) {
      const frac = i / ticks;
      const x = plotX + frac * plotW;
      ctx.beginPath();
      ctx.moveTo(x, plotTop);
      ctx.lineTo(x, plotBottom);
      ctx.stroke();
      const tickT = tStart + frac * windowSeconds;
      ctx.fillText(tickT.toFixed(1), x, plotBottom + 4);
    }

    // Draw each topic row
    topicList.forEach((topic, ri) => {
      const series = seriesRef.current.get(topic);
      const y = plotTop + ri * (rowH + rowGapPx);

      // Row baseline
      ctx.fillStyle = "#262626";
      ctx.fillRect(plotX, y, plotW, rowH);

      if (series && series.events.length > 0) {
        for (let i = 0; i < series.events.length; i++) {
          const ev = series.events[i]!;
          const nextT = (series.events[i + 1]?.t ?? tEnd);
          if (nextT < tStart || ev.t > tEnd) {
            continue;
          }
          const segStart = Math.max(ev.t, tStart);
          const segEnd = Math.min(nextT, tEnd);
          const x1 = plotX + ((segStart - tStart) / windowSeconds) * plotW;
          const x2 = plotX + ((segEnd - tStart) / windowSeconds) * plotW;
          const w = Math.max(1, x2 - x1);
          ctx.fillStyle = series.colorByValue.get(ev.v) ?? "#6b7280";
          ctx.fillRect(x1, y, w, rowH);

          // State value label, only if segment is wide enough
          if (w >= 50 && rowH >= 14) {
            ctx.fillStyle = "#ffffff";
            ctx.font = "11px system-ui, sans-serif";
            ctx.textAlign = "left";
            ctx.textBaseline = "middle";
            ctx.fillText(ev.v, x1 + 6, y + rowH / 2);
          }
        }
      }

      // Topic label (left side)
      if (showTopicLabel) {
        ctx.fillStyle = "#e5e7eb";
        ctx.font = "11px system-ui, sans-serif";
        ctx.textAlign = "right";
        ctx.textBaseline = "middle";
        ctx.fillText(shortTopic(topic), plotX - 6, y + rowH / 2);
      }
    });

    // X-axis title
    ctx.fillStyle = "#6b7280";
    ctx.font = "10px system-ui, sans-serif";
    ctx.textAlign = "right";
    ctx.textBaseline = "bottom";
    ctx.fillText("t (s)", cssW - 4, cssH - 2);
  });

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  return (
    <div style={{ width: "100%", height: "100%", background: "#1e1e1e" }}>
      <canvas ref={canvasRef} style={{ width: "100%", height: "100%", display: "block" }} />
    </div>
  );
}

export function initStateTimelinePanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<StateTimelinePanel context={context} />);
  return () => root.unmount();
}
