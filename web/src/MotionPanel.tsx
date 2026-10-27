import { useEffect, useRef, useState } from "react";
import type { MotionState, StudyState } from "./panels";

interface Props {
  state: MotionState;
  enabled: boolean;
  /** Show `study` at `time`, in seconds: resolves once the kernel showed it. */
  onSeek: (study: string, time: number) => Promise<unknown>;
  /** Play `study` through and find what collides. */
  onCheck: (study: string) => void;
}

/** The time a study is shown at: as the kernel last showed it, or its start. */
function shownAt(state: MotionState, study: StudyState): number {
  return state.shown && state.shown[0] === study.name ? state.shown[1] : study.start;
}

/** One study: a time slider to scrub, play and pause, and a collision check. */
function Study({ study, state, enabled, onSeek, onCheck }: Props & { study: StudyState }) {
  const [playing, setPlaying] = useState(false);
  const [time, setTime] = useState(shownAt(state, study));
  const playingRef = useRef(false);
  // The latest way to seek: a playing loop outlives the render it began in.
  const seek = useRef(onSeek);
  seek.current = onSeek;
  useEffect(() => {
    if (!playing) setTime(shownAt(state, study));
  }, [state, study, playing]);

  // Playing: each frame shown after the one before, at the time that has
  // passed since — a big assembly plays slower, never out of order.
  useEffect(() => {
    playingRef.current = playing;
    if (!playing) return;
    let cancelled = false;
    const { name, start, end } = study;
    const span = end - start;
    const from = time >= end ? start : time;
    const began = performance.now();
    void (async () => {
      while (!cancelled && playingRef.current) {
        const t = Math.min(end, from + (performance.now() - began) / 1000);
        setTime(t);
        await seek.current(name, t);
        if (t >= end || span <= 0) break;
      }
      if (!cancelled) setPlaying(false);
    })();
    return () => {
      cancelled = true;
    };
    // `time` is where play starts: read once, when it starts.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [playing, study.name, study.start, study.end]);

  const checked = state.checked && state.checked.study === study.name ? state.checked : null;
  const step = study.frames > 1 ? (study.end - study.start) / (study.frames - 1) : 0.01;
  return (
    <div className="motion-study">
      <div className="parameter-row" title={`drives ${study.drivers.join(", ")}`}>
        <span className="parameter-name">{study.name}</span>
        <button
          type="button"
          className="motion-play"
          disabled={!enabled}
          aria-label={playing ? "Pause" : "Play"}
          onClick={() => setPlaying((p) => !p)}
        >
          {playing ? "❚❚" : "▶"}
        </button>
        <input
          type="range"
          className="parameter-formula motion-scrub"
          aria-label={`time of ${study.name}`}
          min={study.start}
          max={study.end}
          step={step / 4}
          value={time}
          disabled={!enabled || playing}
          onChange={(e) => {
            const t = Number(e.currentTarget.value);
            setTime(t);
            void onSeek(study.name, t);
          }}
        />
        <span className="parameter-value">{time.toFixed(2)} s</span>
      </div>
      <div className="parameter-row">
        <button type="button" disabled={!enabled || playing} onClick={() => onCheck(study.name)}>
          Check collisions
        </button>
        {checked && (
          <span className="parameter-value">
            {checked.collisions.length === 0
              ? `no collision in ${study.frames} frames`
              : `${checked.collisions.length} collision${checked.collisions.length === 1 ? "" : "s"}`}
          </span>
        )}
      </div>
      {checked?.collisions.map((c) => (
        <div
          className="parameter-row failed"
          key={`${c.time} ${c.a} ${c.b}`}
          onClick={() => enabled && void onSeek(study.name, c.time)}
          title="Show it then"
        >
          <span className="parameter-value">
            {c.time.toFixed(2)} s: {c.a} × {c.b}
          </span>
        </div>
      ))}
    </div>
  );
}

/** The program's motion studies, each to play, scrub and check for collisions. */
export function MotionPanel(props: Props) {
  return (
    <div className="parameters">
      {props.state.studies.map((study) => (
        <Study key={study.name} study={study} {...props} />
      ))}
    </div>
  );
}
