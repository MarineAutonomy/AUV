'use strict';

const params = new URLSearchParams(window.location.search);
const ROSBRIDGE_PORT = params.get('rosbridge_port') || '9090';
const TOPIC_NAME = params.get('topic') || '/frontscan450/range';
const MAX_ROWS = parseInt(params.get('max_rows') || '512', 10);

const rosbridgeUrl = `ws://${window.location.hostname}:${ROSBRIDGE_PORT}`;
document.getElementById('rosbridge-url').textContent = rosbridgeUrl;
document.getElementById('web-port').textContent = window.location.port || '9002';

const rosStatusEl = document.getElementById('ros-status');
const pingInfoEl = document.getElementById('ping-info');

function dbToRgb(value, minDb, maxDb) {
  let t = (value - minDb) / (maxDb - minDb);
  if (!Number.isFinite(t)) {
    t = 0;
  }
  t = Math.max(0, Math.min(1, t));

  const r = Math.floor(255 * Math.max(0, Math.min(1, (t - 0.5) * 2)));
  const g = Math.floor(255 * Math.max(0, Math.min(1, t * 1.4)));
  const b = Math.floor(255 * Math.max(0, Math.min(1, 1 - t * 1.2)));
  return [r, g, b];
}

class WaterfallPlot {
  constructor(canvas, maxRows) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d', { alpha: false });
    this.maxRows = maxRows;
    this.width = 0;
    this.height = maxRows;
    this.rangeLabel = '';
  }

  clear() {
    if (this.width > 0) {
      this.ctx.fillStyle = '#010409';
      this.ctx.fillRect(0, 0, this.width, this.height);
    }
  }

  addRow(samples, minDb, maxDb, startMm, lengthMm) {
    const n = samples.length;
    if (n === 0) {
      return;
    }

    if (this.width !== n) {
      this.width = n;
      this.canvas.width = n;
      this.canvas.height = this.height;
      this.clear();
    }

    this.rangeLabel = `${(startMm / 1000).toFixed(1)}–${((startMm + lengthMm) / 1000).toFixed(1)} m`;

    this.ctx.drawImage(
      this.canvas,
      0, 0, this.width, this.height - 1,
      0, 1, this.width, this.height - 1
    );

    const imageData = this.ctx.createImageData(n, 1);
    const data = imageData.data;

    for (let i = 0; i < n; i++) {
      const [r, g, b] = dbToRgb(samples[i], minDb, maxDb);
      const idx = i * 4;
      data[idx] = r;
      data[idx + 1] = g;
      data[idx + 2] = b;
      data[idx + 3] = 255;
    }

    this.ctx.putImageData(imageData, 0, 0);
  }
}

class ProfilePlot {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d', { alpha: false });
  }

  draw(samples, minDb, maxDb) {
    const w = this.canvas.clientWidth || 400;
    const h = this.canvas.clientHeight || 200;
    this.canvas.width = w;
    this.canvas.height = h;

    this.ctx.fillStyle = '#010409';
    this.ctx.fillRect(0, 0, w, h);

    if (samples.length < 2) {
      return;
    }

    this.ctx.beginPath();
    for (let i = 0; i < samples.length; i++) {
      const x = (i / (samples.length - 1)) * (w - 1);
      let t = (samples[i] - minDb) / (maxDb - minDb);
      if (!Number.isFinite(t)) {
        t = 0;
      }
      t = Math.max(0, Math.min(1, t));
      const y = h - 4 - t * (h - 8);
      if (i === 0) {
        this.ctx.moveTo(x, y);
      } else {
        this.ctx.lineTo(x, y);
      }
    }
    this.ctx.strokeStyle = '#58a6ff';
    this.ctx.lineWidth = 1.5;
    this.ctx.stroke();
  }
}

const scanPlot = new WaterfallPlot(document.getElementById('scan-canvas'), MAX_ROWS);
const profilePlot = new ProfilePlot(document.getElementById('profile-canvas'));

document.getElementById('clear-btn').addEventListener('click', () => {
  scanPlot.clear();
});

function setRosStatus(connected, text) {
  rosStatusEl.textContent = text;
  rosStatusEl.classList.toggle('connected', connected);
  rosStatusEl.classList.toggle('disconnected', !connected);
}

const ros = new ROSLIB.Ros({
  url: rosbridgeUrl,
  groovyCompatibilityMode: false,
});

let rangeTopic = null;

function subscribeToRange() {
  if (rangeTopic) {
    rangeTopic.unsubscribe();
  }

  rangeTopic = new ROSLIB.Topic({
    ros,
    name: TOPIC_NAME,
    messageType: 'frontscan_ros2/msg/FrontScanSonar',
    compression: 'none',
  });

  rangeTopic.subscribe((msg) => {
    const data = msg.scaled_data || [];

    scanPlot.addRow(
      data,
      msg.min_pwr_db,
      msg.max_pwr_db,
      msg.start_mm,
      msg.length_mm
    );

    profilePlot.draw(data, msg.min_pwr_db, msg.max_pwr_db);

    pingInfoEl.textContent =
      `Ping #${msg.ping_number} · ${data.length} bins · ${scanPlot.rangeLabel}`;
  });
}

ros.on('connection', () => {
  setRosStatus(true, 'ROS: connected');
  subscribeToRange();
});

ros.on('error', () => {
  setRosStatus(false, 'ROS: error');
});

ros.on('close', () => {
  setRosStatus(false, 'ROS: disconnected');
});
