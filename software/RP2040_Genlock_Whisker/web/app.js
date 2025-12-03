class GenlockTester {
    constructor() {
        this.port = null;
        this.reader = null;
        this.keepReading = false;
        this.decoder = new TextDecoder();
        this.buffer = '';

        // DOM Elements
        this.connectBtn = document.getElementById('connectBtn');
        this.statusEl = document.getElementById('connectionStatus');
        this.signalTypeEl = document.getElementById('signalType');
        this.fpsEl = document.getElementById('fpsValue');

        this.linesCountEl = document.getElementById('linesCount');
        this.framesCountEl = document.getElementById('framesCount');
        this.hdDetectEl = document.getElementById('hdDetect');
        this.meanPeriodEl = document.getElementById('meanPeriod');

        this.rmsJitterEl = document.getElementById('rmsJitter');
        this.ppJitterEl = document.getElementById('ppJitter');
        this.rmsJitterPpmEl = document.getElementById('rmsJitterPpm');

        // Graphing
        this.canvas = document.getElementById('jitterChart');
        this.ctx = this.canvas.getContext('2d');
        this.jitterHistory = [];
        this.maxHistory = 100; // Number of points to show
        this.currentFormat = null;

        this.init();
        this.totalFrames = 0;
    }

    init() {
        // Initial status setup
        this.updateStatus(false);

        document.getElementById('resetGraphBtn').addEventListener('click', () => this.resetGraph());

        window.addEventListener('resize', () => this.resizeCanvas());
        this.resizeCanvas();
        this.drawGraph(); // Start animation loop
    }

    resizeCanvas() {
        this.canvas.width = this.canvas.parentElement.clientWidth;
        this.canvas.height = this.canvas.parentElement.clientHeight;
    }

    async connect() {
        if ('serial' in navigator) {
            try {
                this.port = await navigator.serial.requestPort();
                await this.port.open({ baudRate: 115200 });

                this.keepReading = true;
                this.updateStatus(true);
                this.readLoop();
            } catch (err) {
                console.error('There was an error opening the serial port:', err);
                alert('Error opening serial port. See console for details.');
            }
        } else {
            alert('Web Serial API not supported in this browser. Please use Chrome or Edge.');
        }
    }

    async readLoop() {
        while (this.port.readable && this.keepReading) {
            this.reader = this.port.readable.getReader();
            try {
                while (true) {
                    const { value, done } = await this.reader.read();
                    if (done) {
                        break;
                    }
                    if (value) {
                        this.processData(value);
                    }
                }
            } catch (error) {
                console.error('Read error:', error);
            } finally {
                this.reader.releaseLock();
            }
        }
    }

    processData(value) {
        this.buffer += this.decoder.decode(value);
        const lines = this.buffer.split('\n');

        // Process all complete lines
        for (let i = 0; i < lines.length - 1; i++) {
            this.parseLine(lines[i]);
        }

        // Keep the last partial line
        this.buffer = lines[lines.length - 1];
    }

    parseLine(line) {
        line = line.trim();
        if (!line) return;

        try {
            const data = JSON.parse(line);
            this.updateUI(data);
            this.updateGraphData(data);
            console.log('Parsed data:', data);
        } catch (e) {
            console.warn('JSON Parse Error:', e);
            console.log('Raw line:', line);
            // If we receive the old format, we might want to warn the user
            if (line.startsWith('Frames:')) {
                alert('Received data in old format. Please upload the new firmware to the RP2040.');
                this.keepReading = false; // Stop reading to avoid spamming alerts
            }
        }
    }

    updateUI(data) {
        // Always calculate format for graph color and auto-reset logic
        const result = this.detectFormat(data.lines, data.fps, data.hd_det);

        // Store current color for graph (immediate update)
        this.graphColor = result.color;

        // Auto-reset graph if format changes (immediate check)
        if (this.currentFormat !== null && this.currentFormat !== result.format) {
            console.log(`Format changed from ${this.currentFormat} to ${result.format}. Resetting graph.`);
            this.resetGraph();
        }
        this.currentFormat = result.format;

        // Accumulate total frames (always update, unthrottled)
        if (data.frames) {
            this.totalFrames = (this.totalFrames || 0) + data.frames;
            this.framesCountEl.textContent = this.totalFrames.toLocaleString();
        }

        // Throttle text updates to ~1Hz to avoid flickering numbers
        const now = Date.now();
        if (!this.lastTextUpdate || now - this.lastTextUpdate >= 1000) {
            this.signalTypeEl.textContent = data.type || '--';
            this.fpsEl.textContent = data.fps ? data.fps.toFixed(3) : '--';

            this.linesCountEl.textContent = data.lines || '0';



            this.hdDetectEl.textContent = data.hd_det === 0 ? 'TRI-LEVEL' : (data.hd_det === 1 ? 'BI-LEVEL' : '--');
            this.meanPeriodEl.textContent = data.period_us_mean ? data.period_us_mean.toFixed(4) + ' µs' : '0.0000 µs';

            const rms = data.jitter_rms_us || 0;
            const pp = data.jitter_pp_us || 0;

            if (rms < 1.0 && rms > 0) {
                this.rmsJitterEl.textContent = (rms * 1000).toFixed(1) + ' ns';
            } else {
                this.rmsJitterEl.textContent = rms.toFixed(4) + ' µs';
            }

            if (pp < 1.0 && pp > 0) {
                this.ppJitterEl.textContent = (pp * 1000).toFixed(1) + ' ns';
            } else {
                this.ppJitterEl.textContent = pp.toFixed(4) + ' µs';
            }

            this.rmsJitterPpmEl.textContent = data.jitter_rms_ppm ? data.jitter_rms_ppm.toFixed(1) : '0.0';

            const formatEl = document.getElementById('videoFormat');
            if (formatEl) {
                formatEl.textContent = result.format;
                formatEl.style.color = result.color;
            }

            this.lastTextUpdate = now;
        }
    }

    resetGraph() {
        this.jitterHistory = [];
    }

    detectFormat(lines, fps, hdDet) {
        if (!lines || !fps) return { format: '--', color: '#ff3333' }; // Red for no signal

        // Standard Broadcast Rates
        const rates = [
            { val: 23.976, label: '23.98' },
            { val: 24.000, label: '24' },
            { val: 25.000, label: '25' },
            { val: 29.970, label: '29.97' },
            { val: 30.000, label: '30' },
            { val: 50.000, label: '50' },
            { val: 59.940, label: '59.94' },
            { val: 60.000, label: '60' }
        ];

        // Find closest standard rate
        let bestRate = null;
        let minDiff = Infinity;

        for (const r of rates) {
            const diff = Math.abs(fps - r.val);
            if (diff < minDiff) {
                minDiff = diff;
                bestRate = r;
            }
        }

        // If within 0.1Hz of a standard rate, use the label. Otherwise use raw rounded.
        const fpsStr = (minDiff < 0.15) ? bestRate.label : Math.round(fps).toString();

        // Standard Line Counts (approximate)
        // 525 lines = NTSC / 480i
        // 625 lines = PAL / 576i
        // 750 lines = 720p
        // 1125 lines = 1080i / 1080p

        // Allow some tolerance for lines (e.g. +/- 10 lines)
        const isLines = (target) => Math.abs(lines - target) < 10;
        // For interlaced, lines per field might be reported. 
        // 1080i field = 562/563 lines. 480i field = 262/263 lines. 576i field = 312/313 lines.
        const isField = (target) => Math.abs(lines - (target / 2)) < 5;

        // Colors
        const COLOR_NTSC = '#00ff00'; // Green
        const COLOR_PAL = '#ffff00';  // Yellow
        const COLOR_PROG = '#0088ff'; // Blue
        const COLOR_INT = '#ff00ff';  // Magenta
        const COLOR_UNK = '#ffffff';  // White

        // --- SD Formats (Bi-Level) ---
        if (hdDet === 1) { // Bi-level
            if (isLines(525) || isField(525)) {
                return { format: "NTSC (525i)", color: COLOR_NTSC };
            }
            if (isLines(625) || isField(625)) {
                return { format: "PAL (625i)", color: COLOR_PAL };
            }
            // Fallback for SD
            return { format: `SD ${lines}L ${fpsStr}Hz`, color: COLOR_UNK };
        }

        // --- HD Formats (Tri-Level) ---
        if (hdDet === 0) { // Tri-level
            // 720p (Progressive, so lines should be ~750)
            if (isLines(750)) {
                return { format: `720p${fpsStr}`, color: COLOR_PROG };
            }

            // 1080i / 1080p
            if (isLines(1125) || isField(1125)) {
                // If we are seeing ~562 lines, it's definitely Interlaced (or PsF)
                if (isField(1125)) {
                    return { format: `1080i${fpsStr}`, color: COLOR_INT };
                }
                // If we see 1125 lines, it could be 1080p OR 1080i (if counted over 2 fields?)
                // But our firmware counts per VSYNC.
                // If VSYNC is 60Hz and lines is 1125 -> 1080p60
                // If VSYNC is 60Hz and lines is 562 -> 1080i60 (fields)

                return { format: `1080p${fpsStr}`, color: COLOR_PROG };
            }

            // Fallback for HD
            return { format: `HD ${lines}L ${fpsStr}Hz`, color: COLOR_UNK };
        }

        return { format: `${lines}L ${fpsStr}Hz`, color: COLOR_UNK };
    }

    updateStatus(connected) {
        if (connected) {
            this.statusEl.textContent = 'CONNECTED';
            this.statusEl.classList.remove('disconnected');
            this.statusEl.classList.add('connected');
            this.connectBtn.textContent = 'DISCONNECT';
            this.connectBtn.onclick = () => this.disconnect();
        } else {
            this.statusEl.textContent = 'DISCONNECTED';
            this.statusEl.classList.remove('connected');
            this.statusEl.classList.add('disconnected');
            this.connectBtn.textContent = 'CONNECT DEVICE';
            this.connectBtn.onclick = () => this.connect();
        }
    }

    async disconnect() {
        this.keepReading = false;
        if (this.reader) {
            await this.reader.cancel();
        }
        if (this.port) {
            await this.port.close();
        }
        this.updateStatus(false);
    }

    updateGraphData(data) {
        if (data.jitter_rms_us !== undefined) {
            this.jitterHistory.push(data.jitter_rms_us);
            if (this.jitterHistory.length > this.maxHistory) {
                this.jitterHistory.shift();
            }
        }
    }

    drawGraph() {
        const width = this.canvas.width;
        const height = this.canvas.height;
        const ctx = this.ctx;

        // Layout constants
        const paddingLeft = 100;
        const paddingBottom = 30;
        const graphWidth = width - paddingLeft;
        const graphHeight = height - paddingBottom;

        // Clear canvas
        ctx.clearRect(0, 0, width, height);

        // --- Draw Grid & Axes ---
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.1)';
        ctx.lineWidth = 1;
        ctx.fillStyle = '#8b8b95';
        ctx.font = '10px Roboto Mono';
        ctx.textAlign = 'right';
        ctx.textBaseline = 'middle';

        // Calculate Y scale
        // Check max value to decide unit (µs vs ns)
        let rawMax = Math.max(...this.jitterHistory, 0);
        let unit = 'µs';
        let scale = 1;

        // If max jitter is less than 1.0 µs, switch to nanoseconds
        if (rawMax < 1.0 && rawMax > 0) {
            unit = 'ns';
            scale = 1000;
        }

        let maxVal = rawMax * scale;

        // Ensure minimum range
        if (unit === 'µs') {
            maxVal = Math.max(maxVal, 1.5);
        } else {
            maxVal = Math.max(maxVal, 100); // Min 100ns range
        }

        // Add some headroom
        maxVal = maxVal * 1.2;

        // Draw horizontal grid lines (Y axis)
        const numGridLines = 5;
        for (let i = 0; i <= numGridLines; i++) {
            const val = (maxVal / numGridLines) * i;
            const y = height - paddingBottom - ((val / maxVal) * graphHeight);

            // Grid line
            ctx.beginPath();
            ctx.moveTo(paddingLeft, y);
            ctx.lineTo(width, y);
            ctx.stroke();

            // Label
            ctx.fillText(val.toFixed(2) + ' ' + unit, paddingLeft - 5, y);
        }

        // Draw vertical grid lines (X axis) - just visual guides
        const numVertLines = 10;
        ctx.textAlign = 'center';
        ctx.textBaseline = 'top';
        for (let i = 0; i <= numVertLines; i++) {
            const x = paddingLeft + (graphWidth / numVertLines) * i;

            ctx.beginPath();
            ctx.moveTo(x, 0);
            ctx.lineTo(x, height - paddingBottom);
            ctx.stroke();
        }

        // Axis Labels
        ctx.save();
        ctx.fillStyle = '#ffffff';
        ctx.font = '12px Outfit';

        // Y Axis Label (Rotated)
        ctx.translate(15, height / 2);
        ctx.rotate(-Math.PI / 2);
        ctx.textAlign = 'center';
        ctx.fillText(`JITTER (RMS ${unit})`, 0, 0);
        ctx.restore();

        // X Axis Label
        ctx.fillStyle = '#ffffff';
        ctx.textAlign = 'right';
        ctx.fillText('TIME (Latest →)', width - 10, height - 20);


        // --- Draw Data Line (Smooth) ---
        if (this.jitterHistory.length < 2) {
            requestAnimationFrame(() => this.drawGraph());
            return;
        }

        const stepX = graphWidth / (this.maxHistory - 1);

        ctx.beginPath();
        // Use dynamic color or default blue
        const lineColor = this.graphColor || '#00f2ff';
        ctx.strokeStyle = lineColor;
        ctx.lineWidth = 2;
        ctx.lineJoin = 'round';

        // Move to first point
        let x0 = paddingLeft;
        let y0 = height - paddingBottom - ((this.jitterHistory[0] * scale / maxVal) * graphHeight);
        ctx.moveTo(x0, y0);

        for (let i = 1; i < this.jitterHistory.length; i++) {
            const x = paddingLeft + (i * stepX);
            const val = this.jitterHistory[i] * scale;
            const y = height - paddingBottom - ((val / maxVal) * graphHeight);

            // Simple smoothing: use quadratic curve to midpoint
            // For the last point, we just draw to it
            // Actually, a better looking smooth line uses midpoints as control points
            // But standard quadratic curve strategy:
            // curve from (prevX, prevY) to (currX, currY) ? No.
            // curve from (prevX, prevY) to midpoint, using control point?

            // Strategy: Draw from previous midpoint to current midpoint, using current point as control?
            // Let's use a simpler "catmull-rom" like smoothing or just quadratic curves between midpoints.

            // We'll use the "midpoint" strategy for quadratic curves
            // Start at P0. 
            // Control point P1. End point (P1+P2)/2.

            // Let's just do simple lineTo for now, but if user wants "smooth", we can do:
            // ctx.quadraticCurveTo(prevX, prevY, midX, midY)

            // Let's try this:
            // P0 = previous point
            // P1 = current point
            // CP = P0
            // No, that's not right.

            // Let's use the previous point (x0, y0) and current point (x, y)
            // We need a control point.
            // Let's just stick to lineTo for "responsiveness" first, but user asked for "smooth".
            // Okay, let's implement a basic smoothing.

            // Actually, let's just use lineTo but since we have 10x data, it will look smoother naturally!
            // But user specifically asked "can the graph points be slightly les sharp".
            // So I will use quadraticCurveTo.

            // To do this properly with a stream of points:
            // We need to keep track of the previous point.
            // We can draw a curve from the *previous midpoint* to the *current midpoint* using the *current point* as a control point?
            // No, using the *previous point* as control point.

            // Let's do this:
            // Move to P0.
            // For i=1 to n-2:
            //   xc = (P[i].x + P[i+1].x) / 2
            //   yc = (P[i].y + P[i+1].y) / 2
            //   quadraticCurveTo(P[i].x, P[i].y, xc, yc)
            // quadraticCurveTo(P[n-1].x, P[n-1].y, P[n].x, P[n].y)

            // We need lookahead.
        }

        // Re-implementing loop for smoothing
        // We already moved to P0 (x0, y0)

        for (let i = 1; i < this.jitterHistory.length - 1; i++) {
            const x1 = paddingLeft + (i * stepX);
            const val1 = this.jitterHistory[i] * scale;
            const y1 = height - paddingBottom - ((val1 / maxVal) * graphHeight);

            const x2 = paddingLeft + ((i + 1) * stepX);
            const val2 = this.jitterHistory[i + 1] * scale;
            const y2 = height - paddingBottom - ((val2 / maxVal) * graphHeight);

            const midX = (x1 + x2) / 2;
            const midY = (y1 + y2) / 2;

            ctx.quadraticCurveTo(x1, y1, midX, midY);

            x0 = x1; // Keep track if needed, but loop handles it
            y0 = y1;
        }

        // Connect to the very last point
        const lastIdx = this.jitterHistory.length - 1;
        const lastX = paddingLeft + (lastIdx * stepX);
        const lastVal = this.jitterHistory[lastIdx] * scale;
        const lastY = height - paddingBottom - ((lastVal / maxVal) * graphHeight);

        // For the last segment, just draw a straight line or curve
        // Since we ended at the midpoint between n-2 and n-1
        // We need to curve through n-1 to n
        // But the loop goes up to length-2.
        // So i stops at length-2.
        // The last point processed in loop is P[length-2].
        // We drew a curve to midpoint(P[length-2], P[length-1]).
        // Now we need to draw from there to P[length-1].

        // Wait, the loop index i is the "control point" index.
        // We start at P0.
        // i=1: CP=P1, End=Mid(P1,P2).
        // ...
        // i=len-2: CP=P[len-2], End=Mid(P[len-2], P[len-1]).
        // Now we are at Mid(P[len-2], P[len-1]).
        // We need to reach P[len-1].

        ctx.quadraticCurveTo(
            paddingLeft + ((lastIdx - 1) * stepX),
            height - paddingBottom - ((this.jitterHistory[lastIdx - 1] * scale / maxVal) * graphHeight),
            lastX,
            lastY
        );

        ctx.stroke();

        // --- Draw Fill ---
        // Close the path for filling
        // lastX is already calculated above
        ctx.lineTo(lastX, height - paddingBottom); // Bottom right
        ctx.lineTo(paddingLeft, height - paddingBottom); // Bottom left
        ctx.closePath();

        // Convert hex to rgba for fill
        // Simple hack: assume hex is #RRGGBB
        let r = 0, g = 242, b = 255;
        if (lineColor.startsWith('#') && lineColor.length === 7) {
            r = parseInt(lineColor.substr(1, 2), 16);
            g = parseInt(lineColor.substr(3, 2), 16);
            b = parseInt(lineColor.substr(5, 2), 16);
        }

        ctx.fillStyle = `rgba(${r}, ${g}, ${b}, 0.1)`;
        ctx.fill();

        requestAnimationFrame(() => this.drawGraph());
    }
}

// Initialize app
document.addEventListener('DOMContentLoaded', () => {
    new GenlockTester();
});
