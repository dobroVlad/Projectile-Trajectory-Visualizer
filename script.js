class ProjectileSolver {
    constructor() {
        this.g = 9.81; // Gravity m/s^2
    }

    toRadians(degrees) {
        return degrees * Math.PI / 180;
    }

    toDegrees(radians) {
        return radians * 180 / Math.PI;
    }

    // Mode 1: Forward (v0, angle -> trajectory)
    solveForward(v0, angleDeg, targetY = 0, h0 = 0, targetX = null) {
        const theta = this.toRadians(angleDeg);
        const vx = v0 * Math.cos(theta);
        const vy = v0 * Math.sin(theta);

        let totalTime;

        // Force time calculation based on targetX if provided and valid
        if (targetX !== null && targetX !== undefined && Math.abs(vx) > 1e-4) {
            totalTime = targetX / vx;
        } else {
            // Standard ballistic calculation: time to hit targetY (or ground)
            const a = 0.5 * this.g;
            const b = -vy;
            const c = targetY - h0;
            const disc = b * b - 4 * a * c;

            if (disc < 0) {
                // Doesn't reach targetY, use time to ground (y=0)
                const disc0 = vy * vy + 2 * this.g * h0;
                totalTime = (vy + Math.sqrt(disc0)) / this.g;
            } else {
                // Take the positive (forward) root
                totalTime = (-b + Math.sqrt(disc)) / (2 * a);
            }
        }

        const maxHeight = h0 + (vy * vy) / (2 * this.g); // Absolute max height
        const range = vx * totalTime;

        // Impact velocity magnitude at targetY
        const vfy = vy - this.g * totalTime;
        const impactVelocity = Math.sqrt(vx * vx + vfy * vfy);

        return {
            v0,
            angle: angleDeg,
            totalTime,
            maxHeight,
            range,
            vx,
            vy,
            impactVelocity,
            h0
        };
    }

    // Mode 2: Inverse Angle (target x, y, v0 -> angles)
    solveInverseAngle(targetX, targetY, v0, h0 = 0) {
        // Adjust targetY relative to launch height
        const relativeTargetY = targetY - h0;

        const v2 = v0 * v0;
        const v4 = v2 * v2;
        const gx = this.g * targetX;

        const discriminant = v4 - this.g * (this.g * targetX * targetX + 2 * relativeTargetY * v2);

        if (discriminant < 0) {
            const maxReachableY = (v2 / (2 * this.g)) - ((this.g * targetX * targetX) / (2 * v2));
            return {
                solutions: [],
                error: `Target out of range! With v=${v0}m/s at x=${targetX}m, max height is ${(maxReachableY + h0).toFixed(2)}m.`
            };
        }

        const sqrtD = Math.sqrt(discriminant);
        const tanTheta1 = (v2 + sqrtD) / gx;
        const tanTheta2 = (v2 - sqrtD) / gx;

        const theta1 = Math.atan(tanTheta1);
        const theta2 = Math.atan(tanTheta2);

        const solutions = [];
        const minAngle = -Math.PI / 2;
        const maxAngle = Math.PI / 2;

        if (theta1 >= minAngle && theta1 <= maxAngle) {
            solutions.push(this.solveForward(v0, this.toDegrees(theta1), targetY, h0, targetX));
        }
        if (theta2 >= minAngle && theta2 <= maxAngle && Math.abs(theta1 - theta2) > 0.001) {
            solutions.push(this.solveForward(v0, this.toDegrees(theta2), targetY, h0, targetX));
        }

        return { solutions };
    }

    // Mode 3: Inverse Velocity (target x, y, angle -> v0)
    solveInverseVelocity(targetX, targetY, angleDeg, h0 = 0) {
        const relativeTargetY = targetY - h0;
        const theta = this.toRadians(angleDeg);
        const cosTheta = Math.cos(theta);
        const tanTheta = Math.tan(theta);
        const num = this.g * targetX * targetX;
        const den = 2 * cosTheta * cosTheta * (targetX * tanTheta - relativeTargetY);

        if (den <= 0) {
            return { error: "Impossible trajectory (target requires infinite or imaginary velocity)" };
        }

        const v0 = Math.sqrt(num / den);
        return this.solveForward(v0, angleDeg, targetY, h0, targetX);
    }

    solveMinimumVelocity(targetX, targetY) {
        const dist = Math.sqrt(targetX * targetX + targetY * targetY);
        const vMin = Math.sqrt(this.g * (targetY + dist));
        return vMin;
    }

    // Mode 4: Inverse Vx (target x, y, vx -> v0, angle)
    solveInverseVx(targetX, targetY, vx, h0 = 0) {
        if (Math.abs(vx) < 1e-6) {
            return { error: "Vx cannot be zero" };
        }

        // 1. Calculate time to reach target X
        // x = vx * t  =>  t = x / vx
        const t = targetX / vx;

        if (t < 0) {
            return { error: "Target is in opposite direction of velocity" };
        }

        // 2. Calculate required initial vertical velocity (vy0)
        // y = h0 + vy0 * t - 0.5 * g * t^2
        // targetY = h0 + vy0 * t - 0.5 * g * t^2
        // vy0 * t = targetY - h0 + 0.5 * g * t^2
        // vy0 = (targetY - h0)/t + 0.5 * g * t
        const vy0 = (targetY - h0) / t + 0.5 * this.g * t;

        // 3. Calculate total v0 and angle
        const v0 = Math.sqrt(vx * vx + vy0 * vy0);
        const angleRad = Math.atan2(vy0, vx);
        const angleDeg = this.toDegrees(angleRad);

        return this.solveForward(v0, angleDeg, targetY, h0, targetX);
    }

    // Mode 5: Inverse Time (target x, y, time -> v0, angle)
    solveInverseTime(targetX, targetY, time, h0 = 0) {
        if (time <= 0) {
            return { error: "Time must be positive" };
        }

        // 1. Calculate required horizontal velocity
        // x = vx * t => vx = x / t
        const vx = targetX / time;

        // 2. Calculate required initial vertical velocity (vy0)
        // y = h0 + vy0 * t - 0.5 * g * t^2
        // targetY - h0 + 0.5 * g * t^2 = vy0 * t
        // vy0 = (targetY - h0 + 0.5 * g * t^2) / t
        const vy0 = (targetY - h0 + 0.5 * this.g * time * time) / time;

        // 3. Combine to get v0 and angle
        const v0 = Math.sqrt(vx * vx + vy0 * vy0);
        const angleRad = Math.atan2(vy0, vx);
        const angleDeg = this.toDegrees(angleRad);

        return this.solveForward(v0, angleDeg, targetY, h0, targetX);
    }
}

class Visualizer {
    constructor(canvasId) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas.getContext('2d');

        this.theme = {
            axis: '#334155',
            trajectory1: '#3b82f6',
            trajectory2: '#f59e0b',
            trajectoryLight1: 'rgba(59, 130, 246, 0.7)',
            trajectoryLight2: 'rgba(245, 158, 11, 0.7)',
            target: '#ef4444',
            projectile: '#f43f5e',
            grid: '#1e293b',
            gridLine: 'rgba(100, 116, 139, 0.15)'
        };

        // Zoom and Pan state
        this.zoom = 1.0;
        this.panX = 0;
        this.panY = 0;
        this.isDragging = false;
        this.lastMouseX = 0;
        this.lastMouseY = 0;
        this.lastGravity = 9.81; // Initialize default gravity
        this.continuationDistance = 10; // Default continuation distance in meters

        this.resize();
        window.addEventListener('resize', () => this.resize());

        // Mouse wheel zoom
        this.canvas.addEventListener('wheel', (e) => {
            e.preventDefault();
            const zoomFactor = e.deltaY > 0 ? 0.9 : 1.1;
            this.zoom *= zoomFactor;
            this.zoom = Math.max(0.1, Math.min(10, this.zoom));
            this.redraw();
        });

        // Pan with mouse drag
        this.canvas.addEventListener('mousedown', (e) => {
            this.isDragging = true;
            this.lastMouseX = e.clientX;
            this.lastMouseY = e.clientY;
        });

        this.canvas.addEventListener('mousemove', (e) => {
            if (this.isDragging) {
                const dx = e.clientX - this.lastMouseX;
                const dy = e.clientY - this.lastMouseY;
                this.panX += dx;
                this.panY += dy;
                this.lastMouseX = e.clientX;
                this.lastMouseY = e.clientY;
                this.redraw();
            }
        });

        this.canvas.addEventListener('mouseup', () => {
            this.isDragging = false;
        });

        this.canvas.addEventListener('mouseleave', () => {
            this.isDragging = false;
            this.showTooltip = false;
            this.redraw();
        });

        // Track mouse for tooltip
        this.showTooltip = false;
        this.tooltipX = 0;
        this.tooltipY = 0;

        this.canvas.addEventListener('mousemove', (e) => {
            if (!this.isDragging && this.lastSimulation && this.lastSimulation.length > 0) {
                const rect = this.canvas.getBoundingClientRect();
                this.tooltipX = e.clientX - rect.left;
                this.tooltipY = e.clientY - rect.top;
                this.showTooltip = true;
                this.redraw();
            }
        });
    }

    redraw() {
        if (this.lastSimulation) {
            if (this.lastSavedTrajectories) {
                this.drawWithSaved(this.lastSimulation, this.lastTarget, this.lastHighlightIndex, this.lastGravity, this.lastSavedTrajectories);
            } else {
                this.draw(this.lastSimulation, this.lastTarget, this.lastHighlightIndex, this.lastGravity);
            }
        } else {
            this.drawGrid(); // Draw grid even if no simulation is present
        }
    }

    // Helper to get "nice" grid spacing values (1, 2, 5, 10, 20, 50, 100, etc.)
    getNiceGridSpacing(rawSpacing) {
        const magnitude = Math.pow(10, Math.floor(Math.log10(rawSpacing)));
        const normalized = rawSpacing / magnitude;

        let nice;
        if (normalized < 1.5) nice = 1;
        else if (normalized < 3) nice = 2;
        else if (normalized < 7) nice = 5;
        else nice = 10;

        return nice * magnitude;
    }

    resize() {
        const rect = this.canvas.parentElement.getBoundingClientRect();
        this.canvas.width = rect.width;
        this.canvas.height = rect.height;
        this.width = rect.width;
        this.height = rect.height;

        if (this.lastSimulation) {
            if (this.lastSavedTrajectories) {
                this.drawWithSaved(this.lastSimulation, this.lastTarget, this.lastHighlightIndex, this.lastGravity, this.lastSavedTrajectories);
            } else {
                this.draw(this.lastSimulation, this.lastTarget, this.lastHighlightIndex, this.lastGravity);
            }
        } else {
            this.drawGrid();
        }
    }

    worldToScreen(x, y, viewParams) {
        const { scale, centerWorldX, centerWorldY, canvasCenterX, canvasCenterY } = viewParams;
        return {
            x: canvasCenterX + (x - centerWorldX) * scale * this.zoom + this.panX,
            y: canvasCenterY - (y - centerWorldY) * scale * this.zoom + this.panY
        };
    }

    calculateViewParams(simulations, target, gravity = 9.81) {
        let minX = 0, maxX = 0;
        let minY = 0, maxY = 0;

        if (target) {
            minX = Math.min(minX, target.x);
            maxX = Math.max(maxX, target.x);
            minY = Math.min(minY, target.y);
            maxY = Math.max(maxY, target.y);
        }

        simulations.forEach(sim => {
            const h0 = sim.h0 || 0;
            minX = Math.min(minX, 0, sim.range);
            maxX = Math.max(maxX, 0, sim.range);
            minY = Math.min(minY, h0);
            maxY = Math.max(maxY, sim.maxHeight);

            // Check trajectory points for negative Y using correct gravity
            for (let i = 0; i <= 100; i++) {
                const t = (i / 100) * sim.totalTime;
                const y = h0 + sim.v0 * Math.sin(sim.angle * Math.PI / 180) * t - 0.5 * gravity * t * t;
                minY = Math.min(minY, y);
            }
        });

        const width = maxX - minX;
        const height = maxY - minY;

        const paddingX = Math.max(width * 0.15, 10);
        const paddingY = Math.max(height * 0.15, 10);

        minX -= paddingX;
        maxX += paddingX;
        minY -= paddingY;
        maxY += paddingY;

        const worldW = maxX - minX;
        const worldH = maxY - minY;

        const widgetPadding = 40;
        const availableW = this.width - 2 * widgetPadding;
        const availableH = this.height - 2 * widgetPadding;

        const scaleX = availableW / (worldW || 1);
        const scaleY = availableH / (worldH || 1);
        const scale = Math.min(scaleX, scaleY);

        return {
            scale,
            centerWorldX: (minX + maxX) / 2,
            centerWorldY: (minY + maxY) / 2,
            canvasCenterX: this.width / 2,
            canvasCenterY: this.height / 2,
            minX, maxX, minY, maxY
        };
    }

    drawGrid(viewParams) {
        this.ctx.fillStyle = "#0f172a";
        this.ctx.fillRect(0, 0, this.width, this.height);

        if (!viewParams) return;

        const effectiveScale = viewParams.scale * this.zoom;

        // Determine grid spacing - aim for 50-100 pixels between lines
        const targetPixelSpacing = 60;
        const rawSpacing = targetPixelSpacing / effectiveScale;
        const gridSpacing = this.getNiceGridSpacing(rawSpacing);

        // Draw grid lines
        this.ctx.strokeStyle = this.theme.gridLine;
        this.ctx.lineWidth = 1;
        this.ctx.beginPath();

        // Vertical lines (X axis)
        const minGridX = Math.floor(viewParams.minX / gridSpacing) * gridSpacing;
        const maxGridX = Math.ceil(viewParams.maxX / gridSpacing) * gridSpacing;
        for (let x = minGridX; x <= maxGridX; x += gridSpacing) {
            const screenPt = this.worldToScreen(x, 0, viewParams);
            this.ctx.moveTo(screenPt.x, 0);
            this.ctx.lineTo(screenPt.x, this.height);
        }

        // Horizontal lines (Y axis)
        const minGridY = Math.floor(viewParams.minY / gridSpacing) * gridSpacing;
        const maxGridY = Math.ceil(viewParams.maxY / gridSpacing) * gridSpacing;
        for (let y = minGridY; y <= maxGridY; y += gridSpacing) {
            const screenPt = this.worldToScreen(0, y, viewParams);
            this.ctx.moveTo(0, screenPt.y);
            this.ctx.lineTo(this.width, screenPt.y);
        }
        this.ctx.stroke();

        // Draw main axes (thicker)
        this.ctx.strokeStyle = this.theme.axis;
        this.ctx.lineWidth = 2;
        this.ctx.beginPath();

        const origin = this.worldToScreen(0, 0, viewParams);

        // X-Axis
        this.ctx.moveTo(0, origin.y);
        this.ctx.lineTo(this.width, origin.y);

        // Y-Axis
        this.ctx.moveTo(origin.x, 0);
        this.ctx.lineTo(origin.x, this.height);

        this.ctx.stroke();

        // Draw scale markers on axes
        this.ctx.fillStyle = '#94a3b8';
        this.ctx.font = '11px Inter, sans-serif';
        this.ctx.textAlign = 'center';

        // X-axis markers
        for (let x = minGridX; x <= maxGridX; x += gridSpacing) {
            if (x === 0) continue;
            const screenPt = this.worldToScreen(x, 0, viewParams);
            if (screenPt.x > 30 && screenPt.x < this.width - 30 && Math.abs(screenPt.y - origin.y) < this.height) {
                this.ctx.fillText(`${x}m`, screenPt.x, origin.y + 15);
            }
        }

        // Y-axis markers
        this.ctx.textAlign = 'right';
        for (let y = minGridY; y <= maxGridY; y += gridSpacing) {
            if (y === 0) continue;
            const screenPt = this.worldToScreen(0, y, viewParams);
            if (screenPt.y > 20 && screenPt.y < this.height - 20 && Math.abs(screenPt.x - origin.x) < this.width) {
                this.ctx.fillText(`${y}m`, origin.x - 10, screenPt.y + 4);
            }
        }
    }

    draw(simulations, target = null, highlightIndex = 0, gravity = 9.81) {
        this.lastSimulation = simulations;
        this.lastTarget = target;
        this.lastHighlightIndex = highlightIndex;
        this.lastGravity = gravity;
        this.lastSavedTrajectories = null; // Clear saved trajectories if simple draw is called

        const viewParams = this.calculateViewParams(simulations, target, gravity);


        this.ctx.clearRect(0, 0, this.width, this.height);
        this.drawGrid(viewParams);

        if (!simulations && !target) return;

        if (target) {
            const screenTarget = this.worldToScreen(target.x, target.y, viewParams);
            this.ctx.fillStyle = this.theme.target;
            this.ctx.beginPath();
            this.ctx.arc(screenTarget.x, screenTarget.y, 6, 0, Math.PI * 2);
            this.ctx.fill();
            this.ctx.fillStyle = '#fff';
            this.ctx.textAlign = 'center';
            this.ctx.fillText(`Target`, screenTarget.x, screenTarget.y - 15);
        }

        if (simulations) {
            simulations.forEach((sim, index) => {
                const isHighlighted = index === highlightIndex;

                // Different colors for different trajectories
                const color = index === 0 ? this.theme.trajectory1 : this.theme.trajectory2;
                const lightColor = index === 0 ? this.theme.trajectoryLight1 : this.theme.trajectoryLight2;

                this.ctx.beginPath();
                this.ctx.strokeStyle = isHighlighted ? color : lightColor;
                this.ctx.lineWidth = isHighlighted ? 3 : 2;

                const steps = 100;
                for (let i = 0; i <= steps; i++) {
                    const t = (i / steps) * sim.totalTime;
                    const x = sim.v0 * Math.cos(sim.angle * Math.PI / 180) * t;
                    const h0 = sim.h0 || 0;
                    const y = h0 + sim.v0 * Math.sin(sim.angle * Math.PI / 180) * t - 0.5 * gravity * t * t;

                    const screenPt = this.worldToScreen(x, y, viewParams);
                    if (i === 0) this.ctx.moveTo(screenPt.x, screenPt.y);
                    else this.ctx.lineTo(screenPt.x, screenPt.y);
                }
                this.ctx.stroke();

                // Draw dashed continuation of trajectory (if highlighted or only one trajectory)
                if (isHighlighted || simulations.length === 1) {
                    const h0 = sim.h0 || 0;
                    // Calculate continuation time based on distance along X axis
                    const vx = Math.abs(sim.v0 * Math.cos(sim.angle * Math.PI / 180));
                    let continuationTime = 0;
                    if (vx > 0.1) {
                        continuationTime = this.continuationDistance / vx;
                    } else {
                        continuationTime = 2.0; // Fallback for vertical shots
                    }

                    this.ctx.beginPath();
                    this.ctx.strokeStyle = isHighlighted ? color : lightColor;
                    this.ctx.lineWidth = isHighlighted ? 2 : 1;
                    this.ctx.setLineDash([5, 5]); // Dashed line
                    this.ctx.globalAlpha = 0.5;

                    const continuationSteps = 50;
                    for (let i = 0; i <= continuationSteps; i++) {
                        const t = sim.totalTime + (i / continuationSteps) * continuationTime;
                        const x = sim.v0 * Math.cos(sim.angle * Math.PI / 180) * t;
                        const y = h0 + sim.v0 * Math.sin(sim.angle * Math.PI / 180) * t - 0.5 * gravity * t * t;

                        const screenPt = this.worldToScreen(x, y, viewParams);
                        if (i === 0) this.ctx.moveTo(screenPt.x, screenPt.y);
                        else this.ctx.lineTo(screenPt.x, screenPt.y);
                    }
                    this.ctx.stroke();
                    this.ctx.setLineDash([]); // Reset to solid line
                    this.ctx.globalAlpha = 1.0;
                }

                // Draw key points with labels (only for highlighted trajectory)
                if (isHighlighted) {
                    this.ctx.font = '12px Inter, sans-serif';
                    this.ctx.textAlign = 'center';

                    // 1. Start point
                    const h0 = sim.h0 || 0;
                    const startPt = this.worldToScreen(0, h0, viewParams);
                    this.ctx.fillStyle = color;
                    this.ctx.beginPath();
                    this.ctx.arc(startPt.x, startPt.y, 4, 0, Math.PI * 2);
                    this.ctx.fill();
                    this.ctx.fillStyle = '#fff';
                    this.ctx.fillText(`Start (0m)`, startPt.x, startPt.y - 20);
                    this.ctx.fillText(`v₀=${sim.v0.toFixed(1)} m/s`, startPt.x, startPt.y - 8);

                    // 2. Apex (maximum height)
                    const tApex = sim.vy / gravity;
                    const xApex = sim.v0 * Math.cos(sim.angle * Math.PI / 180) * tApex;
                    const yApex = sim.maxHeight;
                    const apexPt = this.worldToScreen(xApex, yApex, viewParams);
                    this.ctx.fillStyle = color;
                    this.ctx.beginPath();
                    this.ctx.arc(apexPt.x, apexPt.y, 4, 0, Math.PI * 2);
                    this.ctx.fill();
                    this.ctx.fillStyle = '#fff';
                    this.ctx.fillText(`Apex (${xApex.toFixed(1)}m)`, apexPt.x, apexPt.y - 20);
                    this.ctx.fillText(`h=${sim.maxHeight.toFixed(1)} m`, apexPt.x, apexPt.y - 8);

                    // 3. Impact point
                    const t = sim.totalTime;
                    const impX = sim.v0 * Math.cos(sim.angle * Math.PI / 180) * t;
                    const impY = h0 + sim.v0 * Math.sin(sim.angle * Math.PI / 180) * t - 0.5 * gravity * t * t;

                    const screenImpact = this.worldToScreen(impX, impY, viewParams);

                    this.ctx.fillStyle = color;
                    this.ctx.beginPath();
                    this.ctx.arc(screenImpact.x, screenImpact.y, 5, 0, Math.PI * 2);
                    this.ctx.fill();
                    this.ctx.fillStyle = '#fff';
                    this.ctx.fillText(`Impact (${impX.toFixed(1)}m)`, screenImpact.x, screenImpact.y - 20);
                    this.ctx.fillText(`t=${sim.totalTime.toFixed(1)} s`, screenImpact.x, screenImpact.y - 8);
                } else if (simulations.length > 1) {
                    // Just draw impact for non-highlighted
                    const h0 = sim.h0 || 0;
                    const t = sim.totalTime;
                    const impX = sim.v0 * Math.cos(sim.angle * Math.PI / 180) * t;
                    const impY = h0 + sim.v0 * Math.sin(sim.angle * Math.PI / 180) * t - 0.5 * gravity * t * t;

                    const screenImpact = this.worldToScreen(impX, impY, viewParams);

                    this.ctx.fillStyle = lightColor;
                    this.ctx.beginPath();
                    this.ctx.arc(screenImpact.x, screenImpact.y, 3, 0, Math.PI * 2);
                    this.ctx.fill();
                }
            });
        }

        // Draw tooltip if mouse is over canvas
        if (this.showTooltip && simulations && simulations.length > 0) {
            const sim = simulations[highlightIndex];

            // Convert screen coordinates to world coordinates
            const worldX = (this.tooltipX - viewParams.canvasCenterX - this.panX) / (viewParams.scale * this.zoom) + viewParams.centerWorldX;

            // Calculate extended range (including continuation)
            const vx = Math.abs(sim.v0 * Math.cos(sim.angle * Math.PI / 180));

            let continuationTime = 0;
            if (vx > 0.1) {
                continuationTime = this.continuationDistance / vx;
            } else {
                continuationTime = 2.0;
            }

            const totalTime = sim.totalTime + continuationTime;
            const extendedRange = vx * totalTime; // This is approximate range logic for tooltip check

            // Only show tooltip if x is within extended trajectory range
            // Note: worldX check needs to be smarter for negative/backwards shots if we support them, 
            // but for now assuming positive X or standard range check. 
            // Better to check time t.

            // Calculate time at this X position
            // t = x / v_x
            // Note: we need the signed vx for this calculation
            const signedVx = sim.v0 * Math.cos(sim.angle * Math.PI / 180);
            const t = worldX / signedVx;

            if (t >= 0 && t <= totalTime) {
                // Calculate Y at this time
                const h0 = sim.h0 || 0;
                const worldY = h0 + sim.v0 * Math.sin(sim.angle * Math.PI / 180) * t - 0.5 * gravity * t * t;

                // Calculate velocity components at this point
                const vy = sim.vy - gravity * t;
                const velocity = Math.sqrt(vx * vx + vy * vy);

                // Determine if we're in the continuation part
                const isContinuation = t > sim.totalTime;

                // Draw tooltip
                const tooltipX = this.tooltipX + 15;
                const tooltipY = this.tooltipY - 90;
                const tooltipW = 150;
                const tooltipH = 100;

                // Background (different color for continuation)
                this.ctx.fillStyle = isContinuation ? 'rgba(15, 23, 42, 0.85)' : 'rgba(15, 23, 42, 0.95)';
                this.ctx.strokeStyle = isContinuation ? '#94a3b8' : '#3b82f6';
                this.ctx.lineWidth = 2;
                this.ctx.beginPath();
                this.ctx.roundRect(tooltipX, tooltipY, tooltipW, tooltipH, 8);
                this.ctx.fill();
                this.ctx.stroke();

                // Text
                this.ctx.fillStyle = '#fff';
                this.ctx.font = '12px Inter, sans-serif';
                this.ctx.textAlign = 'left';
                this.ctx.fillText(`X: ${worldX.toFixed(2)} m`, tooltipX + 10, tooltipY + 18);
                this.ctx.fillText(`Y: ${worldY.toFixed(2)} m`, tooltipX + 10, tooltipY + 34);
                this.ctx.fillText(`t: ${t.toFixed(2)} s`, tooltipX + 10, tooltipY + 50);
                this.ctx.fillText(`v: ${velocity.toFixed(2)} m/s`, tooltipX + 10, tooltipY + 66);
                this.ctx.fillText(`vₓ: ${vx.toFixed(2)} m/s`, tooltipX + 10, tooltipY + 82);
                this.ctx.fillText(`vᵧ: ${vy.toFixed(2)} m/s`, tooltipX + 10, tooltipY + 98);

                // Draw marker on trajectory (different style for continuation)
                const markerScreen = this.worldToScreen(worldX, worldY, viewParams);
                this.ctx.fillStyle = isContinuation ? '#94a3b8' : '#3b82f6';
                this.ctx.globalAlpha = isContinuation ? 0.6 : 1.0;
                this.ctx.beginPath();
                this.ctx.arc(markerScreen.x, markerScreen.y, 5, 0, Math.PI * 2);
                this.ctx.fill();
                this.ctx.globalAlpha = 1.0;
            }
        }
    }

    drawWithSaved(simulations, target = null, highlightIndex = 0, gravity = 9.81, savedTrajectories = []) {
        this.lastSimulation = simulations;
        this.lastTarget = target;
        this.lastHighlightIndex = highlightIndex;
        this.lastGravity = gravity;
        this.lastSavedTrajectories = savedTrajectories; // Store saved trajectories for redraw

        // Combine current and saved trajectories for view calculation
        const allSimulations = [...simulations];
        const visibleSaved = savedTrajectories.filter(t => t.visible);
        visibleSaved.forEach(t => allSimulations.push(t.simulation));

        const viewParams = this.calculateViewParams(allSimulations, target, gravity);

        this.ctx.clearRect(0, 0, this.width, this.height);
        this.drawGrid(viewParams);

        if (!simulations && !target && visibleSaved.length === 0) return;

        // Draw saved trajectories first (in background)
        visibleSaved.forEach(savedTraj => {
            const sim = savedTraj.simulation;
            const h0 = sim.h0 || 0;

            this.ctx.beginPath();
            this.ctx.strokeStyle = savedTraj.color;
            this.ctx.lineWidth = 2;
            this.ctx.globalAlpha = 0.6;

            const steps = 100;
            for (let i = 0; i <= steps; i++) {
                const t = (i / steps) * sim.totalTime;
                const x = sim.v0 * Math.cos(sim.angle * Math.PI / 180) * t;
                const y = h0 + sim.v0 * Math.sin(sim.angle * Math.PI / 180) * t - 0.5 * gravity * t * t;

                const screenPt = this.worldToScreen(x, y, viewParams);
                if (i === 0) this.ctx.moveTo(screenPt.x, screenPt.y);
                else this.ctx.lineTo(screenPt.x, screenPt.y);
            }
            this.ctx.stroke();

            // Draw dashed continuation for saved trajectory
            const vx = Math.abs(sim.v0 * Math.cos(sim.angle * Math.PI / 180));
            let continuationTime = 0;
            if (vx > 0.1) {
                continuationTime = this.continuationDistance / vx;
            } else {
                continuationTime = 2.0;
            }

            this.ctx.beginPath();
            this.ctx.strokeStyle = savedTraj.color;
            this.ctx.lineWidth = 1;
            this.ctx.setLineDash([5, 5]);
            this.ctx.globalAlpha = 0.4;

            const continuationSteps = 50;
            for (let i = 0; i <= continuationSteps; i++) {
                const t = sim.totalTime + (i / continuationSteps) * continuationTime;
                const x = sim.v0 * Math.cos(sim.angle * Math.PI / 180) * t;
                const y = h0 + sim.v0 * Math.sin(sim.angle * Math.PI / 180) * t - 0.5 * gravity * t * t;

                const screenPt = this.worldToScreen(x, y, viewParams);
                if (i === 0) this.ctx.moveTo(screenPt.x, screenPt.y);
                else this.ctx.lineTo(screenPt.x, screenPt.y);
            }
            this.ctx.stroke();
            this.ctx.setLineDash([]); // Reset line dash

            this.ctx.globalAlpha = 1.0;
        });

        // Draw target
        if (target) {
            const screenTarget = this.worldToScreen(target.x, target.y, viewParams);
            this.ctx.fillStyle = this.theme.target;
            this.ctx.beginPath();
            this.ctx.arc(screenTarget.x, screenTarget.y, 6, 0, Math.PI * 2);
            this.ctx.fill();
            this.ctx.fillStyle = '#fff';
            this.ctx.textAlign = 'center';
            this.ctx.fillText(`Target`, screenTarget.x, screenTarget.y - 15);
        }

        // Draw current trajectories (on top)
        if (simulations) {
            simulations.forEach((sim, index) => {
                const isHighlighted = index === highlightIndex;
                const color = index === 0 ? this.theme.trajectory1 : this.theme.trajectory2;
                const lightColor = index === 0 ? this.theme.trajectoryLight1 : this.theme.trajectoryLight2;

                this.ctx.beginPath();
                this.ctx.strokeStyle = isHighlighted ? color : lightColor;
                this.ctx.lineWidth = isHighlighted ? 3 : 2;

                const steps = 100;
                const h0 = sim.h0 || 0;
                for (let i = 0; i <= steps; i++) {
                    const t = (i / steps) * sim.totalTime;
                    const x = sim.v0 * Math.cos(sim.angle * Math.PI / 180) * t;
                    const y = h0 + sim.v0 * Math.sin(sim.angle * Math.PI / 180) * t - 0.5 * gravity * t * t;

                    const screenPt = this.worldToScreen(x, y, viewParams);
                    if (i === 0) this.ctx.moveTo(screenPt.x, screenPt.y);
                    else this.ctx.lineTo(screenPt.x, screenPt.y);
                }
                this.ctx.stroke();

                // Draw dashed continuation
                if (isHighlighted || simulations.length === 1) {
                    const vx = Math.abs(sim.v0 * Math.cos(sim.angle * Math.PI / 180));
                    let continuationTime = 0;
                    if (vx > 0.1) {
                        continuationTime = this.continuationDistance / vx;
                    } else {
                        continuationTime = 2.0;
                    }

                    this.ctx.beginPath();
                    this.ctx.strokeStyle = isHighlighted ? color : lightColor;
                    this.ctx.lineWidth = isHighlighted ? 2 : 1;
                    this.ctx.setLineDash([5, 5]);
                    this.ctx.globalAlpha = 0.5;

                    const continuationSteps = 50;
                    for (let i = 0; i <= continuationSteps; i++) {
                        const t = sim.totalTime + (i / continuationSteps) * continuationTime;
                        const x = sim.v0 * Math.cos(sim.angle * Math.PI / 180) * t;
                        const y = h0 + sim.v0 * Math.sin(sim.angle * Math.PI / 180) * t - 0.5 * gravity * t * t;

                        const screenPt = this.worldToScreen(x, y, viewParams);
                        if (i === 0) this.ctx.moveTo(screenPt.x, screenPt.y);
                        else this.ctx.lineTo(screenPt.x, screenPt.y);
                    }
                    this.ctx.stroke();
                    this.ctx.setLineDash([]);
                    this.ctx.globalAlpha = 1.0;
                }

                // Draw key points for highlighted
                if (isHighlighted) {
                    this.ctx.font = '12px Inter, sans-serif';
                    this.ctx.textAlign = 'center';

                    const startPt = this.worldToScreen(0, h0, viewParams);
                    this.ctx.fillStyle = color;
                    this.ctx.beginPath();
                    this.ctx.arc(startPt.x, startPt.y, 4, 0, Math.PI * 2);
                    this.ctx.fill();
                    this.ctx.fillStyle = '#fff';
                    this.ctx.fillText(`Start (0m)`, startPt.x, startPt.y - 20);
                    this.ctx.fillText(`v₀=${sim.v0.toFixed(1)} m/s`, startPt.x, startPt.y - 8);

                    const tApex = sim.vy / gravity;
                    const xApex = sim.v0 * Math.cos(sim.angle * Math.PI / 180) * tApex;
                    const yApex = sim.maxHeight;
                    const apexPt = this.worldToScreen(xApex, yApex, viewParams);
                    this.ctx.fillStyle = color;
                    this.ctx.beginPath();
                    this.ctx.arc(apexPt.x, apexPt.y, 4, 0, Math.PI * 2);
                    this.ctx.fill();
                    this.ctx.fillStyle = '#fff';
                    this.ctx.fillText(`Apex (${xApex.toFixed(1)}m)`, apexPt.x, apexPt.y - 20);
                    this.ctx.fillText(`h=${sim.maxHeight.toFixed(1)} m`, apexPt.x, apexPt.y - 8);

                    const t = sim.totalTime;
                    const impX = sim.v0 * Math.cos(sim.angle * Math.PI / 180) * t;
                    const impY = h0 + sim.v0 * Math.sin(sim.angle * Math.PI / 180) * t - 0.5 * gravity * t * t;

                    const screenImpact = this.worldToScreen(impX, impY, viewParams);

                    this.ctx.fillStyle = color;
                    this.ctx.beginPath();
                    this.ctx.arc(screenImpact.x, screenImpact.y, 5, 0, Math.PI * 2);
                    this.ctx.fill();
                    this.ctx.fillStyle = '#fff';
                    this.ctx.fillText(`Impact (${impX.toFixed(1)}m)`, screenImpact.x, screenImpact.y - 20);
                    this.ctx.fillText(`t=${sim.totalTime.toFixed(1)} s`, screenImpact.x, screenImpact.y - 8);
                } else if (simulations.length > 1) {
                    const t = sim.totalTime;
                    const impX = sim.v0 * Math.cos(sim.angle * Math.PI / 180) * t;
                    const impY = h0 + sim.v0 * Math.sin(sim.angle * Math.PI / 180) * t - 0.5 * gravity * t * t;

                    const screenImpact = this.worldToScreen(impX, impY, viewParams);

                    this.ctx.fillStyle = lightColor;
                    this.ctx.beginPath();
                    this.ctx.arc(screenImpact.x, screenImpact.y, 3, 0, Math.PI * 2);
                    this.ctx.fill();
                }
            });
        }

        // Draw tooltip (check saved trajectories for tooltip active)
        const tooltipTraj = savedTrajectories.find(t => t.tooltipActive);
        const tooltipSim = tooltipTraj ? tooltipTraj.simulation : (simulations && simulations[highlightIndex]);

        if (this.showTooltip && tooltipSim) {
            const worldX = (this.tooltipX - viewParams.canvasCenterX - this.panX) / (viewParams.scale * this.zoom) + viewParams.centerWorldX;

            const vx = Math.abs(tooltipSim.v0 * Math.cos(tooltipSim.angle * Math.PI / 180));
            let continuationTime = 0;
            if (vx > 0.1) {
                continuationTime = this.continuationDistance / vx;
            } else {
                continuationTime = 2.0;
            }

            const totalTime = tooltipSim.totalTime + continuationTime;
            const signedVx = tooltipSim.v0 * Math.cos(tooltipSim.angle * Math.PI / 180);

            // We calculate t directly from X to avoid range bounds issues for now, or just use ample bounds
            // Actually reusing previous logic:
            const t = worldX / signedVx;

            if (t >= 0 && t <= totalTime) {
                const h0 = tooltipSim.h0 || 0;
                const worldY = h0 + tooltipSim.v0 * Math.sin(tooltipSim.angle * Math.PI / 180) * t - 0.5 * gravity * t * t;

                const vy = tooltipSim.vy - gravity * t;
                const velocity = Math.sqrt(vx * vx + vy * vy);

                const isContinuation = t > tooltipSim.totalTime;

                const tooltipX = this.tooltipX + 15;
                const tooltipY = this.tooltipY - 90;
                const tooltipW = 150;
                const tooltipH = 100;

                this.ctx.fillStyle = isContinuation ? 'rgba(15, 23, 42, 0.85)' : 'rgba(15, 23, 42, 0.95)';
                this.ctx.strokeStyle = isContinuation ? '#94a3b8' : (tooltipTraj ? tooltipTraj.color : '#3b82f6');
                this.ctx.lineWidth = 2;
                this.ctx.beginPath();
                this.ctx.roundRect(tooltipX, tooltipY, tooltipW, tooltipH, 8);
                this.ctx.fill();
                this.ctx.stroke();

                this.ctx.fillStyle = '#fff';
                this.ctx.font = '12px Inter, sans-serif';
                this.ctx.textAlign = 'left';
                this.ctx.fillText(`X: ${worldX.toFixed(2)} m`, tooltipX + 10, tooltipY + 18);
                this.ctx.fillText(`Y: ${worldY.toFixed(2)} m`, tooltipX + 10, tooltipY + 34);
                this.ctx.fillText(`t: ${t.toFixed(2)} s`, tooltipX + 10, tooltipY + 50);
                this.ctx.fillText(`v: ${velocity.toFixed(2)} m/s`, tooltipX + 10, tooltipY + 66);
                this.ctx.fillText(`vₓ: ${vx.toFixed(2)} m/s`, tooltipX + 10, tooltipY + 82);
                this.ctx.fillText(`vᵧ: ${vy.toFixed(2)} m/s`, tooltipX + 10, tooltipY + 98);

                const markerScreen = this.worldToScreen(worldX, worldY, viewParams);
                this.ctx.fillStyle = isContinuation ? '#94a3b8' : (tooltipTraj ? tooltipTraj.color : '#3b82f6');
                this.ctx.globalAlpha = isContinuation ? 0.6 : 1.0;
                this.ctx.beginPath();
                this.ctx.arc(markerScreen.x, markerScreen.y, 5, 0, Math.PI * 2);
                this.ctx.fill();
                this.ctx.globalAlpha = 1.0;
            }
        }
    }
}

let currentResults = [];
let solutionSelector, radioButtons;
let selectedTrajectoryIndex = 0; // Track which trajectory is currently selected

// Saved trajectories system
let savedTrajectories = [];
let nextTrajectoryId = 1;
const trajectoryColors = ['#3b82f6', '#f59e0b', '#10b981', '#ef4444', '#8b5cf6', '#ec4899', '#14b8a6', '#f97316'];

document.addEventListener('DOMContentLoaded', () => {
    const solver = new ProjectileSolver();
    const visualizer = new Visualizer('simulation-canvas');
    const modeSelect = document.getElementById('calc-mode');
    const inputsContainer = document.getElementById('inputs-container');
    const gravityInput = document.getElementById('gravity');

    const statsPanel = document.getElementById('stats-panel');
    const elTime = document.getElementById('stat-time');
    const elHeight = document.getElementById('stat-height');
    const elRange = document.getElementById('stat-range');
    const elImpactVel = document.getElementById('stat-impact-vel');
    const elAngle = document.getElementById('stat-angle');
    const h0Input = document.getElementById('h0');

    solutionSelector = document.getElementById('solution-selector');
    radioButtons = document.querySelectorAll('input[name="solution"]');

    // Saved trajectories UI elements
    const saveTrajectoryBtn = document.getElementById('save-trajectory-btn');
    const savedTrajectoriesList = document.getElementById('saved-trajectories-list');

    // Continuation Length Controls
    const contDistInput = document.getElementById('continuation-dist');
    const contSlider = document.getElementById('continuation-slider');

    function updateContinuation(val) {
        const dist = parseFloat(val);
        if (isNaN(dist) || dist < 0) return;
        contDistInput.value = dist;
        contSlider.value = dist;
        visualizer.continuationDistance = dist;
        visualizer.redraw();
    }

    contDistInput.addEventListener('input', () => updateContinuation(contDistInput.value));
    contSlider.addEventListener('input', () => updateContinuation(contSlider.value));


    gravityInput.addEventListener('change', () => {
        solver.g = parseFloat(gravityInput.value) || 9.81;
        simulate();
    });

    h0Input.addEventListener('change', () => {
        simulate();
    });

    radioButtons.forEach(radio => {
        radio.addEventListener('change', () => {
            const index = parseInt(radio.value);
            selectedTrajectoryIndex = index; // Track selected trajectory
            if (currentResults[index]) {
                updateStats(currentResults[index]);
                visualizer.draw(currentResults, currentResults[index].target, index, solver.g);
            }
        });
    });

    const inputConfigs = {
        'forward': [
            { id: 'v0', label: 'Initial Velocity (m/s)', type: 'number', value: 50, min: 1 },
            { id: 'angle', label: 'Angle (degrees)', type: 'number', value: 45, min: -90, max: 90 }
        ],
        'inverse-angle': [
            { id: 'targetX', label: 'Target X (m)', type: 'number', value: 200, min: 0.1 },
            { id: 'targetY', label: 'Target Y (m)', type: 'number', value: 0 },
            { id: 'v0', label: 'Initial Velocity (m/s)', type: 'number', value: 50, min: 1 }
        ],
        'inverse-velocity': [
            { id: 'targetX', label: 'Target X (m)', type: 'number', value: 200, min: 0.1 },
            { id: 'targetY', label: 'Target Y (m)', type: 'number', value: 0 },
            { id: 'angle', label: 'Angle (degrees)', type: 'number', value: 45, min: -90, max: 90 }
        ],
        'inverse-vx': [
            { id: 'targetX', label: 'Target X (m)', type: 'number', value: 200, min: 0.1 },
            { id: 'targetY', label: 'Target Y (m)', type: 'number', value: 0 },
            { id: 'vx', label: 'Horizontal Velocity (m/s)', type: 'number', value: 20, min: 0.1 }
        ],
        'inverse-time': [
            { id: 'targetX', label: 'Target X (m)', type: 'number', value: 200, min: 0.1 },
            { id: 'targetY', label: 'Target Y (m)', type: 'number', value: 0 },
            { id: 'time', label: 'Flight Time (s)', type: 'number', value: 5, min: 0.1 }
        ]
    };

    // Store input values across mode changes
    const savedInputs = {};

    const formulaConfigs = {
        'forward': `
            <strong>Forward Kinematics:</strong><br><br>
            x = v₀ · cos(θ) · t<br>
            y = h₀ + v₀ · sin(θ) · t - ½gt²
        `,
        'inverse-angle': `
            <strong>Trajectory Equation:</strong><br><br>
            y = x·tan(θ) - (g·x²) / (2·v₀²·cos²(θ))<br><br>
            <em>Solved for θ (results in 2 angles)</em>
        `,
        'inverse-velocity': `
            <strong>Required Velocity:</strong><br><br>
            v₀ = √[ (g·x²) / (2·cos²(θ)·(x·tan(θ) - y + h₀)) ]
        `,
        'inverse-vx': `
            <strong>Fixed Horizontal Velocity:</strong><br><br>
            1. t_flight = x / vₓ<br>
            2. vᵧ₀ = (y - h₀ + ½gt²) / t<br>
            3. v₀ = √(vₓ² + vᵧ₀²), θ = atan(vᵧ₀/vₓ)
        `,
        'inverse-time': `
            <strong>Fixed Time:</strong><br><br>
            1. vₓ = x / t<br>
            2. vᵧ₀ = (y - h₀ + ½gt²) / t<br>
            3. v₀ = √(vₓ² + vᵧ₀²), θ = atan(vᵧ₀/vₓ)
        `
    };

    function renderInputs(mode) {
        // Update Formula Display
        const formulaDisplay = document.getElementById('formula-display');
        if (formulaDisplay) {
            formulaDisplay.innerHTML = formulaConfigs[mode] || '';
        }

        // Save current input values before clearing
        const currentInputs = inputsContainer.querySelectorAll('input');
        currentInputs.forEach(input => {
            savedInputs[input.id] = input.value;
        });

        inputsContainer.innerHTML = '';
        inputConfigs[mode].forEach(config => {
            const wrapper = document.createElement('div');
            wrapper.className = 'control-group';

            const label = document.createElement('label');
            label.for = config.id;
            label.textContent = config.label;

            const input = document.createElement('input');
            input.type = config.type;
            input.id = config.id;

            // Restore saved value if exists, otherwise use default
            input.value = savedInputs[config.id] !== undefined ? savedInputs[config.id] : config.value;

            if (config.min !== undefined) input.min = config.min;
            if (config.max !== undefined) input.max = config.max;

            // Trigger simulation on Enter or blur (for manual input)
            input.addEventListener('change', simulate);

            // Trigger simulation immediately on arrow keys
            input.addEventListener('keydown', (e) => {
                if (e.key === 'ArrowUp' || e.key === 'ArrowDown') {
                    // Small delay to let the value update
                    setTimeout(() => simulate(), 10);
                } else if (e.key === 'Enter') {
                    simulate();
                }
            });

            wrapper.appendChild(label);
            wrapper.appendChild(input);
            inputsContainer.appendChild(wrapper);
        });
    }

    function getInputs() {
        const inputs = {};
        const inputsList = inputsContainer.querySelectorAll('input');
        inputsList.forEach(input => {
            inputs[input.id] = parseFloat(input.value);
        });
        return inputs;
    }

    function updateStats(sim) {
        if (!sim) {
            statsPanel.style.display = 'none';
            return;
        }
        statsPanel.style.display = 'block';
        elTime.textContent = sim.totalTime.toFixed(2) + ' s';
        elHeight.textContent = sim.maxHeight.toFixed(2) + ' m';
        elRange.textContent = sim.range.toFixed(2) + ' m';
        elImpactVel.textContent = sim.impactVelocity.toFixed(2) + ' m/s';
        elAngle.textContent = sim.angle.toFixed(2) + '°';
    }

    function showToast(message) {
        let toast = document.getElementById('toast-notification');
        if (!toast) {
            toast = document.createElement('div');
            toast.id = 'toast-notification';
            toast.style.position = 'fixed';
            toast.style.bottom = '20px';
            toast.style.right = '20px';
            toast.style.backgroundColor = 'var(--accent-color)';
            toast.style.color = 'white';
            toast.style.padding = '1rem 1.5rem';
            toast.style.borderRadius = '0.5rem';
            toast.style.boxShadow = '0 4px 6px -1px rgba(0, 0, 0, 0.1)';
            toast.style.zIndex = '1000';
            toast.style.transition = 'opacity 0.3s ease';
            document.body.appendChild(toast);
        }

        toast.textContent = message;
        toast.style.opacity = '1';

        if (toast.timeoutId) clearTimeout(toast.timeoutId);

        toast.timeoutId = setTimeout(() => {
            toast.style.opacity = '0';
        }, 3000);
    }

    function simulate() {
        const mode = modeSelect.value;
        const inputs = getInputs();
        const h0 = parseFloat(h0Input.value) || 0;

        if (Object.values(inputs).some(isNaN)) return;

        currentResults = [];
        selectedTrajectoryIndex = 0; // Reset to first trajectory
        let target = null;
        solutionSelector.style.display = 'none';

        try {
            if (mode === 'forward') {
                const sim = solver.solveForward(inputs.v0, inputs.angle, 0, h0);
                currentResults.push(sim);
                updateStats(sim);
            } else if (mode === 'inverse-angle') {
                target = { x: inputs.targetX, y: inputs.targetY };
                let res = solver.solveInverseAngle(inputs.targetX, inputs.targetY, inputs.v0, h0);

                if (res.error) {
                    const minV = solver.solveMinimumVelocity(inputs.targetX, inputs.targetY);
                    const newV = Math.ceil((minV + 0.5) * 10) / 10;

                    const vInput = document.getElementById('v0');
                    if (vInput) {
                        vInput.value = newV;
                    }

                    showToast(`Speed too low! Increased velocity to ${newV} m/s`);
                    res = solver.solveInverseAngle(inputs.targetX, inputs.targetY, newV, h0);
                }

                currentResults = res.solutions;

                if (currentResults.length > 0) {
                    currentResults.forEach(sim => sim.target = target);
                    currentResults.sort((a, b) => b.angle - a.angle);
                    updateStats(currentResults[0]);

                    if (currentResults.length > 1) {
                        solutionSelector.style.display = 'block';
                        radioButtons[0].checked = true;

                        // Update labels with angle values
                        const labels = solutionSelector.querySelectorAll('label');
                        if (labels[0]) labels[0].innerHTML = `<input type="radio" name="solution" value="0" checked> High Arc (${currentResults[0].angle.toFixed(1)}°)`;
                        if (labels[1]) labels[1].innerHTML = `<input type="radio" name="solution" value="1"> Low Arc (${currentResults[1].angle.toFixed(1)}°)`;

                        // Re-attach listeners
                        const newRadios = solutionSelector.querySelectorAll('input[name="solution"]');
                        newRadios.forEach(radio => {
                            radio.addEventListener('change', () => {
                                const index = parseInt(radio.value);
                                if (currentResults[index]) {
                                    updateStats(currentResults[index]);
                                    visualizer.draw(currentResults, currentResults[index].target, index, solver.g);
                                }
                            });
                        });
                    }
                }
            } else if (mode === 'inverse-velocity') {
                target = { x: inputs.targetX, y: inputs.targetY };
                const sim = solver.solveInverseVelocity(inputs.targetX, inputs.targetY, inputs.angle, h0);
                if (sim.error) {
                    showToast(sim.error);
                } else {
                    sim.target = target;
                    currentResults.push(sim);
                    updateStats(sim);
                }
            } else if (mode === 'inverse-vx') {
                target = { x: inputs.targetX, y: inputs.targetY };
                const sim = solver.solveInverseVx(inputs.targetX, inputs.targetY, inputs.vx, h0);

                if (sim.error) {
                    showToast(sim.error);
                } else {
                    sim.target = target;
                    currentResults.push(sim);
                    updateStats(sim);
                }
            } else if (mode === 'inverse-time') {
                target = { x: inputs.targetX, y: inputs.targetY };
                const sim = solver.solveInverseTime(inputs.targetX, inputs.targetY, inputs.time, h0);

                if (sim.error) {
                    showToast(sim.error);
                } else {
                    sim.target = target;
                    currentResults.push(sim);
                    updateStats(sim);
                }
            }

            visualizer.draw(currentResults, target, 0, solver.g);

        } catch (e) {
            console.error(e);
        }
    }

    modeSelect.addEventListener('change', () => {
        renderInputs(modeSelect.value);
        simulate();
    });

    // Saved Trajectories Management
    function saveCurrentTrajectory() {
        if (currentResults.length === 0) {
            showToast('No trajectory to save!');
            return;
        }

        const mode = modeSelect.value;
        const inputs = getInputs();
        const h0 = parseFloat(h0Input.value) || 0;
        const g = solver.g;

        // Get the currently selected trajectory index
        // Prefer the explicitly tracked index, but double-check the UI state
        // This handles cases where simulate() might have reset things but UI is different
        let indexToSave = selectedTrajectoryIndex;

        // If we choose based on radio buttons in the UI (safeguard)
        const checkedRadio = document.querySelector('input[name="solution"]:checked');
        if (checkedRadio) {
            indexToSave = parseInt(checkedRadio.value);
        }

        const trajectory = currentResults[indexToSave] || currentResults[0];

        const savedTrajectory = {
            id: nextTrajectoryId++,
            name: `Trajectory #${nextTrajectoryId - 1}`,
            mode,
            params: { ...inputs, h0, g },
            simulation: { ...trajectory },
            color: trajectoryColors[(nextTrajectoryId - 2) % trajectoryColors.length],
            visible: true,
            tooltipActive: false
        };

        savedTrajectories.push(savedTrajectory);
        renderSavedTrajectories();
        redrawWithSaved();
        showToast(`Saved ${savedTrajectory.name}`);
    }

    function renderSavedTrajectories() {
        if (savedTrajectories.length === 0) {
            savedTrajectoriesList.innerHTML = '<p class="empty-message">No saved trajectories yet</p>';
            return;
        }

        savedTrajectoriesList.innerHTML = savedTrajectories.map(traj => `
            <div class="trajectory-item ${traj.visible ? 'visible' : ''}" data-id="${traj.id}">
                <div class="trajectory-header">
                    <div class="trajectory-name">
                        <div class="trajectory-color" style="background: ${traj.color}"></div>
                        <span title="${traj.name}">${traj.name}</span>
                        <button class="rename-btn" data-id="${traj.id}" title="Rename">✏️</button>
                    </div>
                </div>
                <div class="trajectory-params">
                    v₀=${traj.simulation.v0.toFixed(1)} m/s, θ=${traj.simulation.angle.toFixed(1)}°<br>
                    g=${traj.params.g.toFixed(2)} m/s², h₀=${traj.params.h0.toFixed(1)} m
                </div>
                <div class="trajectory-controls">
                    <button class="toggle-visibility ${traj.visible ? 'active' : ''}" data-id="${traj.id}">
                        ${traj.visible ? '👁️ Visible' : '👁️‍🗨️ Hidden'}
                    </button>
                    <button class="toggle-tooltip ${traj.tooltipActive ? 'active' : ''}" data-id="${traj.id}">
                        🎯 Tooltip
                    </button>
                    <button class="load-trajectory" data-id="${traj.id}">
                        📂 Load
                    </button>
                    <button class="delete-trajectory danger" data-id="${traj.id}">
                        🗑️ Delete
                    </button>
                </div>
            </div>
        `).join('');

        // Attach event listeners
        savedTrajectoriesList.querySelectorAll('.toggle-visibility').forEach(btn => {
            btn.addEventListener('click', () => toggleVisibility(parseInt(btn.dataset.id)));
        });

        savedTrajectoriesList.querySelectorAll('.toggle-tooltip').forEach(btn => {
            btn.addEventListener('click', () => toggleTooltip(parseInt(btn.dataset.id)));
        });

        savedTrajectoriesList.querySelectorAll('.load-trajectory').forEach(btn => {
            btn.addEventListener('click', () => loadTrajectory(parseInt(btn.dataset.id)));
        });

        savedTrajectoriesList.querySelectorAll('.delete-trajectory').forEach(btn => {
            btn.addEventListener('click', () => deleteTrajectory(parseInt(btn.dataset.id)));
        });

        savedTrajectoriesList.querySelectorAll('.rename-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                e.stopPropagation();
                renameTrajectory(parseInt(btn.dataset.id));
            });
        });
    }

    function renameTrajectory(id) {
        const traj = savedTrajectories.find(t => t.id === id);
        if (!traj) return;

        const newName = prompt("Enter new name for the trajectory:", traj.name);
        if (newName && newName.trim() !== "") {
            traj.name = newName.trim();
            renderSavedTrajectories();
        }
    }

    function toggleVisibility(id) {
        const traj = savedTrajectories.find(t => t.id === id);
        if (traj) {
            traj.visible = !traj.visible;
            renderSavedTrajectories();
            redrawWithSaved();
        }
    }

    function toggleTooltip(id) {
        savedTrajectories.forEach(t => {
            if (t.id === id) {
                t.tooltipActive = !t.tooltipActive; // Toggle clicked
            } else {
                t.tooltipActive = false; // Disable others
            }
        });
        renderSavedTrajectories();
        redrawWithSaved();
    }

    function loadTrajectory(id) {
        const traj = savedTrajectories.find(t => t.id === id);
        if (traj) {
            // Restore mode
            modeSelect.value = traj.mode;
            renderInputs(traj.mode);

            // Restore inputs
            const inputElements = document.querySelectorAll('#inputs-container input');
            inputElements.forEach(input => {
                if (traj.params[input.id] !== undefined) {
                    input.value = traj.params[input.id];
                }
            });

            // Restore h0 and Gravity
            h0Input.value = traj.params.h0;
            solver.g = traj.params.g;
            gravityInput.value = traj.params.g;

            simulate();
            showToast(`Loaded ${traj.name}`);
        }
    }

    function deleteTrajectory(id) {
        if (confirm('Are you sure you want to delete this trajectory?')) {
            savedTrajectories = savedTrajectories.filter(t => t.id !== id);
            renderSavedTrajectories();
            redrawWithSaved();
        }
    }

    function redrawWithSaved() {
        visualizer.drawWithSaved(currentResults, currentResults[selectedTrajectoryIndex]?.target, selectedTrajectoryIndex, solver.g, savedTrajectories);
    }

    // Export functionality
    const exportBtn = document.getElementById('export-btn');
    exportBtn.addEventListener('click', () => {
        if (savedTrajectories.length === 0) {
            showToast('No trajectories to export');
            return;
        }

        const dataStr = JSON.stringify(savedTrajectories, null, 2);
        const blob = new Blob([dataStr], { type: "application/json" });
        const url = URL.createObjectURL(blob);

        const a = document.createElement('a');
        a.href = url;
        const timestamp = new Date().toISOString().slice(0, 19).replace(/:/g, "-");
        a.download = `trajectories-${timestamp}.json`;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);

        showToast('Exported trajectories');
    });

    // Import functionality
    const importBtn = document.getElementById('import-btn');
    const importFile = document.getElementById('import-file');

    importBtn.addEventListener('click', () => {
        importFile.click();
    });

    importFile.addEventListener('change', (e) => {
        const file = e.target.files[0];
        if (!file) return;

        const reader = new FileReader();
        reader.onload = (event) => {
            try {
                const importedData = JSON.parse(event.target.result);

                if (!Array.isArray(importedData)) {
                    throw new Error('Invalid file format: expected an array');
                }

                // Validate imported items
                const validItems = importedData.filter(item =>
                    item.name && item.params && item.simulation
                );

                if (validItems.length === 0) {
                    throw new Error('No valid trajectories found in file');
                }

                // Append imported trajectories, assigning new IDs
                validItems.forEach(item => {
                    item.id = nextTrajectoryId++;
                    // Ensure color is preserved or reassigned if missing
                    if (!item.color) {
                        item.color = trajectoryColors[(nextTrajectoryId - 2) % trajectoryColors.length];
                    }
                    savedTrajectories.push(item);
                });

                renderSavedTrajectories();
                redrawWithSaved();
                showToast(`Imported ${validItems.length} trajectories`);

            } catch (error) {
                console.error('Import error:', error);
                showToast('Error importing file: ' + error.message);
            }

            // Reset file input so same file can be selected again
            importFile.value = '';
        };
        reader.readAsText(file);
    });

    // Save button listener
    saveTrajectoryBtn.addEventListener('click', saveCurrentTrajectory);

    renderInputs('forward');
    visualizer.drawGrid();
    simulate();
});
