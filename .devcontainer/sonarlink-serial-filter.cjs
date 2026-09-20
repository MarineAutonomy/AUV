#!/usr/bin/env node
/**
 * SonarLink probes every serial port that has a USB vendorId. That resets
 * Arduinos / glitches other FTDI gear on the hub.
 *
 * Preload this before sonarlink so SerialPort.list() only returns allowed
 * adapters (Blue Robotics Ping360 + Ping2 serial numbers from udev rules).
 *
 * Override with SONARLINK_SERIAL_ALLOW=serial1,serial2
 */
"use strict";

const Module = require("module");
const path = require("path");

const ALLOW = new Set(
  String(process.env.SONARLINK_SERIAL_ALLOW || "DK0GXSZA,DP05HK4F")
    .split(",")
    .map((s) => s.trim())
    .filter(Boolean)
);

function filterPorts(ports) {
  return (ports || []).filter(
    (p) => p && p.serialNumber && ALLOW.has(String(p.serialNumber))
  );
}

function wrapList(obj, label) {
  if (!obj || typeof obj.list !== "function" || obj.list.__mavFiltered) return false;
  const orig = obj.list.bind(obj);
  const wrapped = async function patchedList(...args) {
    return filterPorts(await orig(...args));
  };
  wrapped.__mavFiltered = true;
  obj.list = wrapped;
  console.log(`[sonarlink-serial-filter] ${label}: allow=${[...ALLOW].join(",")}`);
  return true;
}

function loadBindings() {
  const candidates = [
    "/usr/local/lib/node_modules/@cs/sonarlink/node_modules/@serialport/bindings-cpp",
    "/usr/local/lib/node_modules/@serialport/bindings-cpp",
  ];
  for (const cand of candidates) {
    try {
      return require(cand);
    } catch (_) {}
  }
  const req = Module.createRequire(
    path.join("/usr/local/lib/node_modules/@cs/sonarlink", "package.json")
  );
  return req("@serialport/bindings-cpp");
}

try {
  const bindings = loadBindings();

  // New autoDetect() instances must be wrapped too.
  if (typeof bindings.autoDetect === "function" && !bindings.autoDetect.__mavFiltered) {
    const origDetect = bindings.autoDetect.bind(bindings);
    const wrappedDetect = function (...args) {
      const inst = origDetect(...args);
      wrapList(inst, "autoDetect-instance");
      return inst;
    };
    wrappedDetect.__mavFiltered = true;
    bindings.autoDetect = wrappedDetect;
  }

  wrapList(bindings.LinuxBinding, "LinuxBinding");
  if (bindings.LinuxBinding && bindings.LinuxBinding.prototype) {
    wrapList(bindings.LinuxBinding.prototype, "LinuxBinding.prototype");
  }

  // Wrap whatever instance exists right now.
  try {
    wrapList(bindings.autoDetect(), "autoDetect()-now");
  } catch (_) {}
} catch (e) {
  console.warn("[sonarlink-serial-filter] failed:", e && e.message ? e.message : e);
}

// Also intercept serialport package when it loads.
const origLoad = Module._load;
Module._load = function (request, parent, isMain) {
  const exported = origLoad.apply(this, arguments);
  if (request === "serialport" || request.endsWith("/serialport")) {
    try {
      if (exported && typeof exported.SerialPort?.list === "function") {
        wrapList(exported.SerialPort, "serialport.SerialPort");
      }
      if (exported && typeof exported.list === "function") {
        wrapList(exported, "serialport");
      }
    } catch (_) {}
  }
  return exported;
};
