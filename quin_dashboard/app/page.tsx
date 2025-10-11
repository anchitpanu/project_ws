"use client";

import React, { useEffect, useRef, useState } from "react";
import ROSLIB from "roslib";
import { Camera, Gauge, PlugZap, Satellite, Signal } from "lucide-react";

type RosLike = ROSLIB.Ros | null;

export default function Page() {
  // ---- Endpoints ----
  const [wsUrl, setWsUrl] = useState("ws://192.168.1.100:9090");
  const [mjpegUrl, setMjpegUrl] = useState(
    "http://192.168.1.100:8081/stream?topic=/quin/image_raw"
  );

  // ---- Fixed topic: read directly from /quin/debug/encoder (Float32 in cm) ----
  const distanceTopic = "/quin/debug/encoder";

  // ---- ROS connection ----
  const rosRef = useRef<RosLike>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [autoReconnect, setAutoReconnect] = useState(true);

  useEffect(() => {
    return () => {
      try {
        rosRef.current?.close();
      } catch {}
    };
  }, []);

  const connect = () => {
    try {
      rosRef.current?.close();
    } catch {}
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
    try {
      rosRef.current?.close();
    } catch {}
  };

  // ---- Distance subscriber (std_msgs/msg/Float32 in cm -> show as meters) ----
  const [distanceM, setDistanceM] = useState<number | null>(null);
  const [distanceUpdatedAt, setDistanceUpdatedAt] = useState<number | null>(null);
  const distanceSubRef = useRef<ROSLIB.Topic | null>(null);

  useEffect(() => {
    if (!isConnected || !rosRef.current) return;

    distanceSubRef.current?.unsubscribe();

    const sub = new ROSLIB.Topic({
      ros: rosRef.current,
      name: distanceTopic,
      messageType: "std_msgs/msg/Float32", // ROS 2 type (สำคัญ)
      throttle_rate: 100, // ms
      queue_size: 1,
    });

    console.log("[distance] subscribing to", distanceTopic);

    sub.subscribe((msg: any) => {
      const cm = Number(msg?.data);
      const m = Number.isFinite(cm) ? cm / 100.0 : null; // cm -> m
      setDistanceM(m);
      setDistanceUpdatedAt(Date.now());
    });

    distanceSubRef.current = sub;
    return () => {
      console.log("[distance] unsubscribe");
      sub.unsubscribe();
    };
  }, [isConnected]);

  // ---- Helpers ----
  const fmt = (n: number | null, d = 3) => (n == null ? "-" : n.toFixed(d));

  // ---- UI ----
  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-50 to-slate-100 p-4 sm:p-6 md:p-8 text-slate-900">
      <div className="mx-auto max-w-5xl space-y-6">
        {/* Header */}
        <header className="flex items-center justify-between">
          <div className="flex items-center gap-3">
            <PlugZap className={`h-6 w-6 ${isConnected ? "text-green-600" : "text-slate-400"}`} />
            <h1 className="text-2xl font-semibold">Robot Monitor</h1>
          </div>
          <div className="flex items-center gap-2">
            <Signal className={`h-5 w-5 ${isConnected ? "text-green-600" : "text-red-500"}`} />
            <span className="text-sm">{isConnected ? "Connected" : "Disconnected"}</span>
          </div>
        </header>

        {/* Connection */}
        <section className="rounded-2xl border bg-white p-4 shadow-sm">
          <div className="mb-3 flex items-center gap-2 text-base font-semibold">
            <Satellite className="h-4 w-4" />
            <span>Connection</span>
          </div>

          <div className="grid gap-3 md:grid-cols-3">
            <LabeledInput
              label="rosbridge WebSocket URL"
              value={wsUrl}
              onChange={setWsUrl}
              placeholder="ws://<robot-ip>:9090"
            />
            <LabeledInput
              label="MJPEG Stream URL"
              value={mjpegUrl}
              onChange={setMjpegUrl}
              placeholder="http://<robot-ip>:8080/stream?topic=/image_raw"
            />
            <div className="flex items-end gap-2">
              <button
                onClick={connect}
                disabled={isConnected}
                className="flex-1 rounded-xl bg-slate-900 px-4 py-2 text-white hover:bg-slate-800 disabled:cursor-not-allowed disabled:opacity-50"
              >
                Connect
              </button>
              <button
                onClick={disconnect}
                className="flex-1 rounded-xl border px-4 py-2 hover:bg-slate-50"
              >
                Disconnect
              </button>
            </div>
          </div>

          <label className="mt-3 flex items-center gap-2 text-sm">
            <input
              type="checkbox"
              checked={autoReconnect}
              onChange={(e) => setAutoReconnect(e.target.checked)}
            />
            Auto reconnect
          </label>

          <p className="mt-2 text-xs">
            Distance source: <code>{distanceTopic}</code> (<code>std_msgs/Float32</code> in <b>cm</b>,
            UI shows <b>m</b>)
          </p>
        </section>

        {/* Grid: Camera + KPI */}
        <div className="grid gap-6 md:grid-cols-2">
          {/* Camera */}
          <section className="rounded-2xl border bg-white p-4 shadow-sm md:col-span-2">
            <div className="mb-2 flex items-center gap-2 text-base font-semibold">
              <Camera className="h-4 w-4" />
              <span>Live Camera</span>
            </div>
            <div className="aspect-video w-full overflow-hidden rounded-xl border bg-black">
              <img src={mjpegUrl} alt="Robot camera" className="h-full w-full object-contain" />
            </div>
            <p className="mt-2 text-xs">
              From <code>web_video_server</code> (HTTP :8080, MJPEG). Ensure topic name matches your camera topic.
            </p>
          </section>

          {/* Distance */}
          <KPI
            icon={<Gauge className="h-4 w-4" />}
            title="Distance (from encoder)"
            value={fmt(distanceM, 3)}
            unit="m"
            updatedAt={distanceUpdatedAt}
            note={`subscribe: ${distanceTopic} (Float32 in cm → shown as m)`}
          />
        </div>
      </div>
    </div>
  );
}

/* ---------- Small UI helpers ---------- */

function LabeledInput({
  label,
  value,
  onChange,
  placeholder,
}: {
  label: string;
  value: string;
  onChange: (v: string) => void;
  placeholder?: string;
}) {
  return (
    <div className="flex flex-col gap-1">
      <label className="text-xs">{label}</label>
      <input
        value={value}
        onChange={(e) => onChange(e.target.value)}
        placeholder={placeholder}
        className="rounded-xl border px-3 py-2 outline-none focus:ring placeholder-slate-700"
      />
    </div>
  );
}

function KPI({
  icon,
  title,
  value,
  unit,
  updatedAt,
  note,
}: {
  icon: React.ReactNode;
  title: string;
  value: string | number;
  unit?: string;
  updatedAt: number | null;
  note?: string;
}) {
  return (
    <section className="rounded-2xl border bg-white p-4 shadow-sm">
      <div className="mb-2 flex items-center gap-2 text-base font-semibold">
        {icon}
        <span>{title}</span>
      </div>
      <div className="flex items-end justify-between gap-4">
        <div className="text-5xl font-semibold tracking-tight">
          {value} {unit ? <span className="text-sm font-normal">{unit}</span> : null}
        </div>
        <div className="text-right text-xs">
          <div>updated: {updatedAt ? new Date(updatedAt).toLocaleTimeString() : "-"}</div>
        </div>
      </div>
      {note && <div className="mt-3 text-xs">{note}</div>}
    </section>
  );
}
