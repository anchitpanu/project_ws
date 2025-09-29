"use client";

import React, { useEffect, useMemo, useRef, useState } from "react";
import ROSLIB from "roslib";
import { motion } from "framer-motion";
import { Camera, Gauge, Map, PlugZap, Satellite, Signal, ActivitySquare } from "lucide-react";

/**
 * Robot Live Dashboard (Next.js App Router)
 * - Place in app/page.tsx
 * - Requires TailwindCSS (create-next-app --tailwind)
 * - Robot side must run:
 *   - rosbridge_server (WebSocket, ws://<robot-ip>:9090)
 *   - web_video_server (HTTP MJPEG, http://<robot-ip>:8080/stream?topic=/image_raw)
 */

type Pose2D = { x: number; y: number; yaw: number };

export default function Page() {
  // ---------- Config (edit defaults to your LAN IP / topics) ----------
  const [wsUrl, setWsUrl] = useState("ws://192.168.1.50:9090");
  const [mjpegUrl, setMjpegUrl] = useState("http://192.168.1.50:8080/stream?topic=/quin/image_raw");

  const [rangeTopic, setRangeTopic] = useState("/quin/range");          // sensor_msgs/Range
  const [odomTopic, setOdomTopic] = useState("/odom");                  // nav_msgs/Odometry
  const [twistTopic, setTwistTopic] = useState("/quin/debug/motor");    // geometry_msgs/Twist

  // ---------- ROS connection ----------
  const rosRef = useRef<ROSLIB.Ros | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [autoReconnect, setAutoReconnect] = useState(true);

  useEffect(() => {
    return () => {
      try { rosRef.current?.close(); } catch {}
    };
  }, []);

  const connect = () => {
    try { rosRef.current?.close(); } catch {}
    const ros = new ROSLIB.Ros({ url: wsUrl });
    rosRef.current = ros;

    ros.on("connection", () => setIsConnected(true));
    ros.on("close", () => {
      setIsConnected(false);
      if (autoReconnect) setTimeout(() => connect(), 1500);
    });
    ros.on("error", () => setIsConnected(false));
  };

  const disconnect = () => {
    setAutoReconnect(false);
    try { rosRef.current?.close(); } catch {}
  };

  // ---------- Subscriptions: Range ----------
  const [range, setRange] = useState<{ distance: number; frame_id: string; stamp: number } | null>(null);
  const rangeSubRef = useRef<ROSLIB.Topic | null>(null);

  useEffect(() => {
    if (!isConnected || !rosRef.current) return;
    rangeSubRef.current?.unsubscribe();
    const sub = new ROSLIB.Topic({
      ros: rosRef.current,
      name: rangeTopic,
      messageType: "sensor_msgs/Range",
    });
    sub.subscribe((msg: any) => {
      setRange({
        distance: msg?.range ?? NaN,
        frame_id: msg?.header?.frame_id ?? "",
        stamp: Date.now(),
      });
    });
    rangeSubRef.current = sub;
    return () => sub.unsubscribe();
  }, [isConnected, rangeTopic]);

  // ---------- Subscriptions: Twist ----------
  const [twist, setTwist] = useState<{ lx: number; ly: number; lz: number; az: number; stamp: number } | null>(null);
  const twistSubRef = useRef<ROSLIB.Topic | null>(null);

  useEffect(() => {
    if (!isConnected || !rosRef.current) return;
    twistSubRef.current?.unsubscribe();
    const sub = new ROSLIB.Topic({
      ros: rosRef.current,
      name: twistTopic,
      messageType: "geometry_msgs/Twist",
    });
    sub.subscribe((msg: any) => {
      setTwist({
        lx: msg?.linear?.x ?? 0,
        ly: msg?.linear?.y ?? 0,
        lz: msg?.linear?.z ?? 0,
        az: msg?.angular?.z ?? 0,
        stamp: Date.now(),
      });
    });
    twistSubRef.current = sub;
    return () => sub.unsubscribe();
  }, [isConnected, twistTopic]);

  // ---------- Subscriptions: Odometry ----------
  const [pose, setPose] = useState<Pose2D | null>(null);
  const [path, setPath] = useState<Pose2D[]>([]);
  const odomSubRef = useRef<ROSLIB.Topic | null>(null);

  useEffect(() => {
    if (!isConnected || !rosRef.current) return;
    odomSubRef.current?.unsubscribe();
    const sub = new ROSLIB.Topic({
      ros: rosRef.current,
      name: odomTopic,
      messageType: "nav_msgs/Odometry",
    });
    sub.subscribe((msg: any) => {
      const p = msg?.pose?.pose;
      const px = p?.position?.x ?? 0;
      const py = p?.position?.y ?? 0;
      const q = p?.orientation ?? { w: 1, x: 0, y: 0, z: 0 };
      const yaw = quatToYaw(q);
      const cur = { x: px, y: py, yaw };
      setPose(cur);
      setPath((prev) => {
        const next = [...prev, cur];
        return next.slice(-2000); // keep last N points
      });
    });
    odomSubRef.current = sub;
    return () => sub.unsubscribe();
  }, [isConnected, odomTopic]);

  // ---------- Optional publisher (Twist) ----------
  const [enablePub, setEnablePub] = useState(false);
  const [linX, setLinX] = useState(0);
  const [angZ, setAngZ] = useState(0);
  const pubRef = useRef<ROSLIB.Topic | null>(null);

  useEffect(() => {
    if (!isConnected || !rosRef.current || !enablePub) {
      if (pubRef.current) pubRef.current.unadvertise();
      pubRef.current = null;
      return;
    }
    const t = new ROSLIB.Topic({
      ros: rosRef.current,
      name: twistTopic,
      messageType: "geometry_msgs/Twist",
    });
    t.advertise();
    pubRef.current = t;
    return () => t.unadvertise();
  }, [isConnected, enablePub, twistTopic]);

  const publishOnce = () => {
    if (!pubRef.current) return;
    const msg = new ROSLIB.Message({
      linear: { x: linX, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: angZ },
    });
    pubRef.current.publish(msg);
  };

  // ---------- Helpers ----------
  const [scale, setScale] = useState(60); // px per meter
  const [showGrid, setShowGrid] = useState(true);

  const pathD = useMemo(() => {
    if (!path.length) return "";
    return path.map((p) => `${p.x * scale},${-p.y * scale}`).join(" ");
  }, [path, scale]);

  function quatToYaw(q: { w: number; x: number; y: number; z: number }) {
    const { w, x, y, z } = q;
    const siny_cosp = 2 * (w * z + x * y);
    const cosy_cosp = 1 - 2 * (y * y + z * z);
    return Math.atan2(siny_cosp, cosy_cosp);
  }
  const fmt = (n?: number, d = 2) => (n === undefined || n === null || Number.isNaN(n) ? "-" : Number(n).toFixed(d));

  // ---------- UI ----------
  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-50 to-slate-100 p-4 sm:p-6 md:p-8">
      <div className="mx-auto max-w-7xl space-y-6">
        {/* Header */}
        <header className="flex items-center justify-between">
          <div className="flex items-center gap-3">
            <PlugZap className={`h-6 w-6 ${isConnected ? "text-green-600" : "text-slate-400"}`} />
            <h1 className="text-2xl font-semibold">Robot Live Dashboard</h1>
          </div>
          <div className="flex items-center gap-2">
            <Signal className={`h-5 w-5 ${isConnected ? "text-green-600" : "text-red-500"}`} />
            <span className="text-sm text-slate-600">{isConnected ? "Connected" : "Disconnected"}</span>
          </div>
        </header>

        {/* Connection */}
        <section className="rounded-2xl border bg-white p-4 shadow-sm">
          <div className="mb-3 flex items-center gap-2 text-base font-semibold">
            <Satellite className="h-4 w-4" />
            <span>Connection</span>
          </div>

          <div className="grid gap-3 md:grid-cols-3">
            <LabeledInput label="rosbridge WebSocket URL" value={wsUrl} onChange={setWsUrl} placeholder="ws://<robot-ip>:9090" />
            <LabeledInput label="MJPEG Stream URL" value={mjpegUrl} onChange={setMjpegUrl} placeholder="http://<robot-ip>:8080/stream?topic=/image_raw" />
            <div className="flex items-end gap-2">
              <button
                onClick={connect}
                disabled={isConnected}
                className="flex-1 rounded-xl bg-slate-900 px-4 py-2 text-white hover:bg-slate-800 disabled:cursor-not-allowed disabled:opacity-50"
              >
                Connect
              </button>
              <button onClick={disconnect} className="flex-1 rounded-xl border px-4 py-2 hover:bg-slate-50">
                Disconnect
              </button>
            </div>
          </div>

          <div className="mt-3 grid gap-3 md:grid-cols-3">
            <LabeledInput label="Range topic" value={rangeTopic} onChange={setRangeTopic} />
            <LabeledInput label="Odometry topic" value={odomTopic} onChange={setOdomTopic} />
            <LabeledInput label="Debug/Twist topic" value={twistTopic} onChange={setTwistTopic} />
          </div>

          <div className="mt-3 flex items-center gap-2">
            <input id="autorec" type="checkbox" checked={autoReconnect} onChange={(e) => setAutoReconnect(e.target.checked)} className="h-4 w-4" />
            <label htmlFor="autorec" className="text-sm text-slate-600">Auto reconnect</label>
          </div>
        </section>

        {/* Grid */}
        <div className="grid gap-6 md:grid-cols-2 xl:grid-cols-3">
          {/* Video */}
          <section className="rounded-2xl border bg-white p-4 shadow-sm">
            <div className="mb-2 flex items-center gap-2 text-base font-semibold">
              <Camera className="h-4 w-4" />
              <span>Live Camera</span>
            </div>
            <div className="aspect-video w-full overflow-hidden rounded-xl border bg-black">
              <img src={mjpegUrl} alt="Robot camera" className="h-full w-full object-contain" />
            </div>
            <p className="mt-2 text-xs text-slate-500">Uses <code>web_video_server</code> (HTTP :8080, MJPEG).</p>
          </section>

          {/* Range */}
          <section className="rounded-2xl border bg-white p-4 shadow-sm">
            <div className="mb-2 flex items-center gap-2 text-base font-semibold">
              <Gauge className="h-4 w-4" />
              <span>Distance (Range)</span>
            </div>
            <div className="flex items-end justify-between gap-4">
              <motion.div
                key={range?.distance ?? "na"}
                initial={{ scale: 0.9, opacity: 0.7 }}
                animate={{ scale: 1, opacity: 1 }}
                transition={{ type: "spring", stiffness: 160, damping: 15 }}
                className="text-5xl font-semibold tracking-tight"
              >
                {range ? `${fmt(range.distance, 2)} m` : "-"}
              </motion.div>
              <div className="text-right text-xs text-slate-500">
                <div>frame: {range?.frame_id || "-"}</div>
                <div>updated: {range ? new Date(range.stamp).toLocaleTimeString() : "-"}</div>
              </div>
            </div>
            <div className="mt-3 text-xs text-slate-500">
              subscribe: <code>{rangeTopic}</code> (sensor_msgs/Range)
            </div>
          </section>

          {/* Twist */}
          <section className="rounded-2xl border bg-white p-4 shadow-sm">
            <div className="mb-2 flex items-center gap-2 text-base font-semibold">
              <ActivitySquare className="h-4 w-4" />
              <span>Movement (Twist)</span>
            </div>
            <div className="grid grid-cols-2 gap-3">
              <InfoTile label="Linear X" value={fmt(twist?.lx, 2)} unit="m/s" />
              <InfoTile label="Linear Y" value={fmt(twist?.ly, 2)} unit="m/s" />
              <InfoTile label="Linear Z" value={fmt(twist?.lz, 2)} unit="m/s" />
              <InfoTile label="Angular Z" value={fmt(twist?.az, 2)} unit="rad/s" />
            </div>
            <div className="mt-3 text-xs text-slate-500">
              subscribe: <code>{twistTopic}</code> (geometry_msgs/Twist)
            </div>
          </section>

          {/* Path */}
          <section className="rounded-2xl border bg-white p-4 shadow-sm md:col-span-2 xl:col-span-1">
            <div className="mb-2 flex items-center gap-2 text-base font-semibold">
              <Map className="h-4 w-4" />
              <span>Path (Odometry)</span>
            </div>

            <div className="mb-2 flex items-center justify-between">
              <div className="text-xs text-slate-600">Scale: {scale} px/m</div>
              <div className="flex items-center gap-3">
                <input type="range" min={10} max={200} step={5} value={scale} onChange={(e) => setScale(Number(e.target.value))} />
                <label className="flex items-center gap-2 text-xs text-slate-600">
                  <input type="checkbox" checked={showGrid} onChange={(e) => setShowGrid(e.target.checked)} />
                  Show grid
                </label>
              </div>
            </div>

            <div className="relative h-[320px] w-full overflow-hidden rounded-xl border bg-white">
              <svg viewBox="-400 -240 800 480" className="h-full w-full">
                {showGrid && <GridSVG />}
                <polyline points={pathD} fill="none" stroke="currentColor" strokeWidth={2} className="text-blue-600" strokeOpacity={0.9} />
                {pose && <RobotArrow pose={pose} scale={scale} />}
              </svg>
            </div>

            <div className="mt-2 text-xs text-slate-500">
              subscribe: <code>{odomTopic}</code> (nav_msgs/Odometry)
            </div>
          </section>

          {/* Publisher */}
          <section className="rounded-2xl border bg-white p-4 shadow-sm">
            <div className="mb-2 text-base font-semibold">Quick Controls (Publish Twist)</div>
            <label className="mb-2 flex items-center gap-2 text-sm text-slate-600">
              <input type="checkbox" checked={enablePub} onChange={(e) => setEnablePub(e.target.checked)} />
              Enable local publisher to <code className="ml-1">{twistTopic}</code>
            </label>

            <div className="grid gap-3 md:grid-cols-2">
              <RangeInput label="Linear X (m/s)" value={linX} setValue={setLinX} min={-1} max={1} step={0.02} />
              <RangeInput label="Angular Z (rad/s)" value={angZ} setValue={setAngZ} min={-2} max={2} step={0.02} />
            </div>

            <div className="mt-3 flex gap-2">
              <button
                onClick={publishOnce}
                disabled={!enablePub || !isConnected}
                className="rounded-xl bg-slate-900 px-4 py-2 text-white hover:bg-slate-800 disabled:cursor-not-allowed disabled:opacity-50"
              >
                Publish Once
              </button>
              <button onClick={() => { setLinX(0); setAngZ(0); }} className="rounded-xl border px-4 py-2 hover:bg-slate-50">
                Reset
              </button>
            </div>

            <p className="mt-2 text-xs text-slate-500">Disabled by default for safety. Enable before sending commands.</p>
          </section>
        </div>

        <footer className="pt-2 text-center text-xs text-slate-500">
          Works with rosbridge + web_video_server. Adjust URLs & topics in the UI.
        </footer>
      </div>
    </div>
  );
}

/* ---------- Small UI helpers ---------- */

function LabeledInput({
  label, value, onChange, placeholder,
}: { label: string; value: string; onChange: (v: string) => void; placeholder?: string }) {
  return (
    <div className="flex flex-col gap-1">
      <label className="text-xs text-slate-600">{label}</label>
      <input
        value={value}
        onChange={(e) => onChange(e.target.value)}
        placeholder={placeholder}
        className="rounded-xl border px-3 py-2 outline-none focus:ring"
      />
    </div>
  );
}

function InfoTile({ label, value, unit }: { label: string; value: string | number; unit?: string }) {
  return (
    <div className="rounded-2xl border p-3">
      <div className="text-xs text-slate-500">{label}</div>
      <div className="mt-1 text-2xl font-semibold">
        {value} {unit ? <span className="text-sm font-normal text-slate-500">{unit}</span> : null}
      </div>
    </div>
  );
}

function GridSVG() {
  const lines: JSX.Element[] = [];
  for (let i = -400; i <= 400; i += 40) lines.push(<line key={`v${i}`} x1={i} y1={-240} x2={i} y2={240} stroke="#e5e7eb" strokeWidth={1} />);
  for (let j = -240; j <= 240; j += 40) lines.push(<line key={`h${j}`} x1={-400} y1={j} x2={400} y2={j} stroke="#e5e7eb" strokeWidth={1} />);
  return <g>{lines}</g>;
}

function RobotArrow({ pose, scale }: { pose: { x: number; y: number; yaw: number }; scale: number }) {
  const x = pose.x * scale;
  const y = -pose.y * scale;
  const size = 14;
  const heading = pose.yaw;
  const points = `${-size},${size} ${size},${0} ${-size},${-size}`;
  return (
    <g transform={`translate(${x},${y}) rotate(${(heading * 180) / Math.PI})`}>
      <polygon points={points} fill="#0ea5e9" stroke="#0369a1" strokeWidth={1.5} />
    </g>
  );
}

function RangeInput({
  label, value, setValue, min, max, step,
}: { label: string; value: number; setValue: (v: number) => void; min: number; max: number; step: number }) {
  return (
    <div className="flex flex-col gap-1">
      <label className="text-xs text-slate-600">
        {label}: {value.toFixed(2)}
      </label>
      <input type="range" min={min} max={max} step={step} value={value} onChange={(e) => setValue(Number(e.target.value))} />
    </div>
  );
}
