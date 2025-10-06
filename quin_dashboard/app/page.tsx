"use client";

import React, { useEffect, useRef, useState } from "react";
import ROSLIB from "roslib";
import { Camera, Gauge, Sprout, PlugZap, Satellite, Signal } from "lucide-react";

/**
 * Minimal Robot Monitor:
 * - Camera (MJPEG via web_video_server)
 * - Distance from encoder (std_msgs/Float32)  -> meters
 * - Plant count (std_msgs/UInt32)             -> number of planted seedlings
 *
 * Place in: app/page.tsx
 * Robot side must run:
 *   ros2 launch rosbridge_server rosbridge_websocket_launch.xml   # ws://<robot-ip>:9090
 *   ros2 run web_video_server web_video_server                     # http://<robot-ip>:8080
 */

type RosLike = ROSLIB.Ros | null;

export default function Page() {
  // ---- Configurable endpoints (edit defaults; can change in UI) ----
  const [wsUrl, setWsUrl] = useState("ws://192.168.1.50:9090");
  const [mjpegUrl, setMjpegUrl] = useState("http://192.168.1.50:8080/stream?topic=/quin/image_raw");

  // ---- Topic names (adjust to your actual topics) ----
  const [distanceTopic, setDistanceTopic] = useState("/quin/debug/encoder");      // std_msgs/Float32
  const [plantCountTopic, setPlantCountTopic] = useState("/quin/plant_count");    // std_msgs/UInt32

  // ---- ROS connection ----
  const rosRef = useRef<RosLike>(null);
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

  // ---- Distance subscriber (std_msgs/Float32) ----
  const [distance, setDistance] = useState<number | null>(null);
  const [distanceUpdatedAt, setDistanceUpdatedAt] = useState<number | null>(null);
  const distanceSubRef = useRef<ROSLIB.Topic | null>(null);

  useEffect(() => {
    if (!isConnected || !rosRef.current) return;
    distanceSubRef.current?.unsubscribe();
    const sub = new ROSLIB.Topic({
      ros: rosRef.current,
      name: distanceTopic,
      messageType: "std_msgs/Float32",
    });
    sub.subscribe((msg: any) => {
      const v = typeof msg?.data === "number" ? msg.data : NaN;
      setDistance(Number.isFinite(v) ? v : null);
      setDistanceUpdatedAt(Date.now());
    });
    distanceSubRef.current = sub;
    return () => sub.unsubscribe();
  }, [isConnected, distanceTopic]);

  // ---- Plant count subscriber (std_msgs/UInt32 or Int32) ----
  const [plantCount, setPlantCount] = useState<number | null>(null);
  const [plantUpdatedAt, setPlantUpdatedAt] = useState<number | null>(null);
  const plantSubRef = useRef<ROSLIB.Topic | null>(null);

  useEffect(() => {
    if (!isConnected || !rosRef.current) return;
    plantSubRef.current?.unsubscribe();
    // Try UInt32; if your message is Int32, change messageType below to "std_msgs/Int32"
    const sub = new ROSLIB.Topic({
      ros: rosRef.current,
      name: plantCountTopic,
      messageType: "std_msgs/UInt32",
    });
    sub.subscribe((msg: any) => {
      const v = Number(msg?.data);
      setPlantCount(Number.isFinite(v) ? v : null);
      setPlantUpdatedAt(Date.now());
    });
    plantSubRef.current = sub;
    return () => sub.unsubscribe();
  }, [isConnected, plantCountTopic]);

  // ---- Helpers ----
  const fmt = (n: number | null, d = 2) =>
    n === null || Number.isNaN(n) ? "-" : n.toFixed(d);

  // ---- UI ----
  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-50 to-slate-100 p-4 sm:p-6 md:p-8">
      <div className="mx-auto max-w-5xl space-y-6">
        {/* Header */}
        <header className="flex items-center justify-between">
          <div className="flex items-center gap-3">
            <PlugZap className={`h-6 w-6 ${isConnected ? "text-green-600" : "text-slate-400"}`} />
            <h1 className="text-2xl font-semibold">Robot Monitor</h1>
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

          <div className="mt-3 grid gap-3 md:grid-cols-2">
            <LabeledInput
              label="Distance topic (std_msgs/Float32)"
              value={distanceTopic}
              onChange={setDistanceTopic}
              placeholder="/quin/encoder/distance_m"
            />
            <LabeledInput
              label="Plant count topic (std_msgs/UInt32)"
              value={plantCountTopic}
              onChange={setPlantCountTopic}
              placeholder="/quin/plant_count"
            />
          </div>

          <label className="mt-3 flex items-center gap-2 text-sm text-slate-600">
            <input
              type="checkbox"
              checked={autoReconnect}
              onChange={(e) => setAutoReconnect(e.target.checked)}
            />
            Auto reconnect
          </label>
        </section>

        {/* Grid: Camera + Two KPIs */}
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
            <p className="mt-2 text-xs text-slate-500">
              From <code>web_video_server</code> (HTTP :8080, MJPEG). Ensure topic name matches your camera topic.
            </p>
          </section>

          {/* Distance */}
          <KPI
            icon={<Gauge className="h-4 w-4" />}
            title="Distance (from encoder)"
            value={fmt(distance, 3)}
            unit="m"
            updatedAt={distanceUpdatedAt}
            note={`subscribe: ${distanceTopic} (std_msgs/Float32)`}
          />

          {/* Plant Count */}
          <KPI
            icon={<Sprout className="h-4 w-4" />}
            title="Plants Planted"
            value={plantCount === null ? "-" : plantCount}
            unit=""
            updatedAt={plantUpdatedAt}
            note={`subscribe: ${plantCountTopic} (std_msgs/UInt32)`}
          />
        </div>
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

function KPI({
  icon, title, value, unit, updatedAt, note,
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
          {value} {unit ? <span className="text-sm font-normal text-slate-500">{unit}</span> : null}
        </div>
        <div className="text-right text-xs text-slate-500">
          <div>updated: {updatedAt ? new Date(updatedAt).toLocaleTimeString() : "-"}</div>
        </div>
      </div>
      {note && <div className="mt-3 text-xs text-slate-500">{note}</div>}
    </section>
  );
}
