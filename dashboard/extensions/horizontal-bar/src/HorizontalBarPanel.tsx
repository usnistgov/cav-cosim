import { Immutable, MessageEvent, PanelExtensionContext, SettingsTreeAction } from "@foxglove/extension";
import { ReactElement, useCallback, useEffect, useLayoutEffect, useMemo, useRef, useState } from "react";
import { createRoot } from "react-dom/client";

type PanelState = {
  topic: string;
  minValue: number;
  maxValue: number;
  color: string;
  label: string;
};

const DEFAULT_STATE: PanelState = {
  topic: "",
  minValue: 0,
  maxValue: 1,
  color: "#4caf50",
  label: "",
};

function HorizontalBarPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [state, setState] = useState<PanelState>(() => ({
    ...DEFAULT_STATE,
    ...(context.initialState as Partial<PanelState> | undefined),
  }));
  const [value, setValue] = useState<number | undefined>(undefined);
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const msgCountRef = useRef(0);
  const [msgCount, setMsgCount] = useState(0);

  // Strip trailing ".data" so user can paste either "/topic" or "/topic.data".
  const rawTopic = useMemo(() => state.topic.replace(/\.data$/, ""), [state.topic]);

  const settingsActionHandler = useCallback((action: SettingsTreeAction) => {
    if (action.action !== "update") return;
    const fieldName = action.payload.path[action.payload.path.length - 1] as keyof PanelState;
    if (!fieldName) return;
    const v = action.payload.value as PanelState[keyof PanelState];
    setState((prev) => {
      const next = { ...prev, [fieldName]: v };
      context.saveState(next);
      return next;
    });
  }, [context]);

  useEffect(() => {
    context.updatePanelSettingsEditor({
      actionHandler: settingsActionHandler,
      nodes: {
        general: {
          label: "General",
          fields: {
            topic: { label: "Topic", input: "string", value: state.topic, placeholder: "/metrics/throttle" },
            minValue: { label: "Min value", input: "number", value: state.minValue },
            maxValue: { label: "Max value", input: "number", value: state.maxValue },
            color: { label: "Bar color", input: "rgb", value: state.color },
            label: { label: "Label", input: "string", value: state.label },
          },
        },
      },
    });
  }, [context, settingsActionHandler, state]);

  // Set up render handler and subscription in a single layout effect (matches
  // Foxglove's example pattern so messages arrive on the first render).
  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);
      const frame = renderState.currentFrame as Immutable<MessageEvent[]> | undefined;
      if (frame && frame.length > 0) {
        for (let i = frame.length - 1; i >= 0; i--) {
          const m = frame[i];
          if (m && m.topic === rawTopic) {
            const raw = (m.message as { data?: unknown }).data;
            const num = typeof raw === "number" ? raw : Number(raw);
            if (!Number.isNaN(num)) {
              setValue(num);
              msgCountRef.current += 1;
              setMsgCount(msgCountRef.current);
              break;
            }
          }
        }
      }
    };
    context.watch("currentFrame");
    context.watch("topics");

    if (rawTopic) {
      context.subscribe([{ topic: rawTopic }]);
    }

    return () => {
      try { context.unsubscribeAll(); } catch { /* noop */ }
    };
  }, [context, rawTopic]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  const range = state.maxValue - state.minValue;
  const pct = value === undefined || !isFinite(value) || range === 0
    ? 0
    : Math.max(0, Math.min(1, (value - state.minValue) / range)) * 100;
  const display = value === undefined ? "—" : (isFinite(value) ? value.toFixed(2) : "NaN");

  return (
    <div style={{
      width: "100%", height: "100%", padding: "12px 16px",
      display: "flex", flexDirection: "column", justifyContent: "center",
      boxSizing: "border-box", background: "#1e1e1e", color: "#eee",
      fontFamily: "system-ui, sans-serif",
    }}>
      {state.label && (
        <div style={{ fontSize: 14, marginBottom: 6, letterSpacing: 1, textTransform: "uppercase", opacity: 0.85 }}>
          {state.label}
        </div>
      )}
      <div style={{ display: "flex", alignItems: "center", gap: 12 }}>
        <div style={{
          flex: 1, height: 28, background: "#333",
          borderRadius: 4, overflow: "hidden", position: "relative",
        }}>
          <div style={{
            width: `${pct}%`, height: "100%", background: state.color,
            transition: "width 80ms linear",
          }} />
        </div>
        <div style={{ minWidth: 60, textAlign: "right", fontFamily: "monospace", fontSize: 18 }}>
          {display}
        </div>
      </div>
      <div style={{ fontSize: 10, opacity: 0.45, marginTop: 6, fontFamily: "monospace" }}>
        topic={rawTopic || "(none)"} · msgs={msgCount}
      </div>
    </div>
  );
}

export function initHorizontalBarPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<HorizontalBarPanel context={context} />);
  return () => root.unmount();
}
