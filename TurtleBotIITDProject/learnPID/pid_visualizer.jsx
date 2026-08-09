import React, { useState, useRef } from "react";
import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  Legend,
  ReferenceLine,
  ResponsiveContainer,
} from "recharts";

export default function PIDVisualizer() {
  const [a, setA] = useState(30);
  const [b, setB] = useState(20);
  const [Kp, setKp] = useState(2);
  const [Ki, setKi] = useState(0.5);
  const [Kd, setKd] = useState(0.1);

  const [feedbackInput, setFeedbackInput] = useState("0");
  const [history, setHistory] = useState([]);
  const [mode, setMode] = useState("manual"); // "manual" | "auto"

  const integralRef = useRef(0);
  const prevErrorRef = useRef(0);
  const stepRef = useRef(0);
  const plantRef = useRef(0); // simulated plant state for auto mode

  const setpoint = Number(a) + Number(b);
  const INTEGRAL_LIMIT = 50;

  function computeStep(feedback) {
    const dt = 1;
    const error = setpoint - feedback;

    let integral = integralRef.current + error * dt;
    if (integral > INTEGRAL_LIMIT) integral = INTEGRAL_LIMIT;
    if (integral < -INTEGRAL_LIMIT) integral = -INTEGRAL_LIMIT;

    const derivative = (error - prevErrorRef.current) / dt;

    const P = Kp * error;
    const I = Ki * integral;
    const D = Kd * derivative;
    const output = P + I + D;

    integralRef.current = integral;
    prevErrorRef.current = error;
    stepRef.current += 1;

    return {
      step: stepRef.current,
      setpoint,
      feedback: Number(feedback.toFixed(3)),
      error: Number(error.toFixed(3)),
      P: Number(P.toFixed(3)),
      I: Number(I.toFixed(3)),
      D: Number(D.toFixed(3)),
      output: Number(output.toFixed(3)),
    };
  }

  function handleAddManual() {
    const fb = parseFloat(feedbackInput);
    if (Number.isNaN(fb)) return;
    const point = computeStep(fb);
    setHistory((h) => [...h, point]);
  }

  function handleReset() {
    integralRef.current = 0;
    prevErrorRef.current = 0;
    stepRef.current = 0;
    plantRef.current = 0;
    setHistory([]);
  }

  // Simple first-order lag "plant": feedback chases last output.
  // Lets you see a classic PID step-response without typing 40 numbers by hand.
  function handleRunAuto(steps = 40) {
    handleReset();
    const lag = 0.25; // how quickly the plant responds to output
    let plant = 0;
    integralRef.current = 0;
    prevErrorRef.current = 0;
    stepRef.current = 0;
    const points = [];
    for (let i = 0; i < steps; i++) {
      const point = computeStep(plant);
      plant = plant + (point.output - plant) * lag;
      points.push(point);
    }
    plantRef.current = plant;
    setHistory(points);
  }

  const last = history[history.length - 1];

  return (
    <div className="min-h-full w-full bg-slate-950 text-slate-200 p-5 rounded-xl">
      <div className="mb-4">
        <div className="text-xs uppercase tracking-widest text-cyan-400/70 font-mono">
          control loop bench
        </div>
        <h2 className="text-xl font-semibold text-slate-100">PID Response Visualizer</h2>
        <p className="text-sm text-slate-400 mt-1">
          Setpoint = a + b. Feed it error either by hand or by running the simulated plant, and watch P / I / D shape the output.
        </p>
      </div>

      {/* Config panel */}
      <div className="grid grid-cols-2 sm:grid-cols-5 gap-3 mb-4">
        <LabeledNumber label="a" value={a} onChange={setA} />
        <LabeledNumber label="b" value={b} onChange={setB} />
        <LabeledNumber label="Kp" value={Kp} onChange={setKp} step={0.1} />
        <LabeledNumber label="Ki" value={Ki} onChange={setKi} step={0.1} />
        <LabeledNumber label="Kd" value={Kd} onChange={setKd} step={0.05} />
      </div>

      <div className="flex flex-wrap items-center gap-2 mb-4 font-mono text-sm">
        <span className="px-2 py-1 rounded bg-slate-900 border border-slate-800">
          setpoint = <span className="text-amber-400">{setpoint}</span>
        </span>
        {last && (
          <>
            <span className="px-2 py-1 rounded bg-slate-900 border border-slate-800">
              step {last.step}
            </span>
            <span className="px-2 py-1 rounded bg-slate-900 border border-slate-800">
              error <span className="text-rose-400">{last.error}</span>
            </span>
            <span className="px-2 py-1 rounded bg-slate-900 border border-slate-800">
              output <span className="text-cyan-300">{last.output}</span>
            </span>
          </>
        )}
      </div>

      {/* Mode controls */}
      <div className="flex flex-wrap gap-2 mb-4">
        <button
          onClick={() => setMode("manual")}
          className={`px-3 py-1.5 rounded-md text-sm font-medium border ${
            mode === "manual"
              ? "bg-cyan-500/20 border-cyan-500 text-cyan-300"
              : "bg-slate-900 border-slate-800 text-slate-400"
          }`}
        >
          Manual feedback
        </button>
        <button
          onClick={() => setMode("auto")}
          className={`px-3 py-1.5 rounded-md text-sm font-medium border ${
            mode === "auto"
              ? "bg-amber-500/20 border-amber-500 text-amber-300"
              : "bg-slate-900 border-slate-800 text-slate-400"
          }`}
        >
          Auto plant simulation
        </button>

        {mode === "manual" ? (
          <div className="flex gap-2 ml-auto">
            <input
              value={feedbackInput}
              onChange={(e) => setFeedbackInput(e.target.value)}
              onKeyDown={(e) => e.key === "Enter" && handleAddManual()}
              placeholder="feedback value"
              className="w-32 px-2 py-1.5 rounded-md bg-slate-900 border border-slate-700 font-mono text-sm text-slate-100 focus:outline-none focus:ring-2 focus:ring-cyan-500"
            />
            <button
              onClick={handleAddManual}
              className="px-3 py-1.5 rounded-md text-sm font-medium bg-cyan-600 hover:bg-cyan-500 text-slate-950"
            >
              Add point
            </button>
          </div>
        ) : (
          <button
            onClick={() => handleRunAuto(40)}
            className="ml-auto px-3 py-1.5 rounded-md text-sm font-medium bg-amber-600 hover:bg-amber-500 text-slate-950"
          >
            Run 40-step response
          </button>
        )}

        <button
          onClick={handleReset}
          className="px-3 py-1.5 rounded-md text-sm font-medium bg-slate-900 border border-slate-800 text-slate-400 hover:text-slate-200"
        >
          Reset
        </button>
      </div>

      {/* Chart */}
      <div
        className="bg-slate-900/60 border border-slate-800 rounded-lg p-3"
        style={{ filter: "drop-shadow(0 0 4px rgba(45,212,191,0.25))" }}
      >
        <ResponsiveContainer width="100%" height={280}>
          <LineChart data={history} margin={{ top: 10, right: 10, left: -10, bottom: 0 }}>
            <CartesianGrid stroke="#1e293b" strokeDasharray="3 3" />
            <XAxis
              dataKey="step"
              stroke="#64748b"
              tick={{ fontSize: 11, fontFamily: "monospace" }}
              label={{ value: "step", position: "insideBottom", offset: -2, fill: "#64748b", fontSize: 11 }}
            />
            <YAxis stroke="#64748b" tick={{ fontSize: 11, fontFamily: "monospace" }} />
            <Tooltip
              contentStyle={{
                background: "#0f172a",
                border: "1px solid #1e293b",
                fontFamily: "monospace",
                fontSize: 12,
              }}
              labelStyle={{ color: "#94a3b8" }}
            />
            <Legend wrapperStyle={{ fontFamily: "monospace", fontSize: 12 }} />
            <ReferenceLine
              y={setpoint}
              stroke="#f59e0b"
              strokeDasharray="6 4"
              label={{ value: "setpoint", position: "insideTopRight", fill: "#f59e0b", fontSize: 11 }}
            />
            <Line
              type="monotone"
              dataKey="feedback"
              name="feedback"
              stroke="#22d3ee"
              strokeWidth={2}
              dot={{ r: 2 }}
              isAnimationActive={false}
            />
            <Line
              type="monotone"
              dataKey="output"
              name="output (PID)"
              stroke="#f472b6"
              strokeWidth={2}
              dot={{ r: 2 }}
              isAnimationActive={false}
            />
          </LineChart>
        </ResponsiveContainer>
      </div>

      {/* Term breakdown table */}
      {history.length > 0 && (
        <div className="mt-4 overflow-x-auto">
          <table className="w-full text-xs font-mono border-collapse">
            <thead>
              <tr className="text-slate-500 border-b border-slate-800">
                <th className="text-left py-1 pr-3">step</th>
                <th className="text-left py-1 pr-3">feedback</th>
                <th className="text-left py-1 pr-3">error</th>
                <th className="text-left py-1 pr-3 text-teal-400">P</th>
                <th className="text-left py-1 pr-3 text-violet-400">I</th>
                <th className="text-left py-1 pr-3 text-rose-400">D</th>
                <th className="text-left py-1 pr-3 text-pink-300">output</th>
              </tr>
            </thead>
            <tbody>
              {history.slice(-8).map((h) => (
                <tr key={h.step} className="border-b border-slate-900 text-slate-300">
                  <td className="py-1 pr-3">{h.step}</td>
                  <td className="py-1 pr-3">{h.feedback}</td>
                  <td className="py-1 pr-3">{h.error}</td>
                  <td className="py-1 pr-3 text-teal-400">{h.P}</td>
                  <td className="py-1 pr-3 text-violet-400">{h.I}</td>
                  <td className="py-1 pr-3 text-rose-400">{h.D}</td>
                  <td className="py-1 pr-3 text-pink-300">{h.output}</td>
                </tr>
              ))}
            </tbody>
          </table>
          <div className="text-slate-500 text-xs mt-1 font-mono">showing last 8 of {history.length} points</div>
        </div>
      )}
    </div>
  );
}

function LabeledNumber({ label, value, onChange, step = 1 }) {
  return (
    <label className="flex flex-col gap-1">
      <span className="text-xs font-mono text-slate-500">{label}</span>
      <input
        type="number"
        step={step}
        value={value}
        onChange={(e) => onChange(parseFloat(e.target.value))}
        className="px-2 py-1.5 rounded-md bg-slate-900 border border-slate-700 font-mono text-sm text-slate-100 focus:outline-none focus:ring-2 focus:ring-cyan-500"
      />
    </label>
  );
}
