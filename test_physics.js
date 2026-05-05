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

    solveForward(v0, angleDeg) {
        const theta = this.toRadians(angleDeg);
        const vx = v0 * Math.cos(theta);
        const vy = v0 * Math.sin(theta);

        const totalTime = (2 * vy) / this.g;
        const maxHeight = (vy * vy) / (2 * this.g);
        const range = vx * totalTime;

        return { v0, angle: angleDeg, totalTime, maxHeight, range };
    }

    solveInverseAngle(targetX, targetY, v0) {
        const v2 = v0 * v0;
        const v4 = v2 * v2;
        const gx = this.g * targetX;

        const discriminant = v4 - this.g * (this.g * targetX * targetX + 2 * targetY * v2);

        if (discriminant < 0) return { solutions: [] };

        const sqrtD = Math.sqrt(discriminant);
        const tanTheta1 = (v2 + sqrtD) / gx;
        const tanTheta2 = (v2 - sqrtD) / gx;

        const theta1 = Math.atan(tanTheta1);
        const theta2 = Math.atan(tanTheta2);

        const solutions = [];
        if (theta1 >= 0) solutions.push(this.solveForward(v0, this.toDegrees(theta1)));
        if (theta2 >= 0 && Math.abs(theta1 - theta2) > 0.001) solutions.push(this.solveForward(v0, this.toDegrees(theta2)));

        return { solutions };
    }
}

// Tests
const solver = new ProjectileSolver();
const eps = 0.1;

console.log("Running Physics Verification...");

// Test 1: Forward 45 degrees, 100 m/s
// R = v^2/g * sin(2*45) = 10000/9.81 * 1 = 1019.36
const t1 = solver.solveForward(100, 45);
console.log(`Test 1 (Forward): Range=${t1.range.toFixed(2)} (Expected ~1019.37) -> ${Math.abs(t1.range - 1019.37) < eps ? 'PASS' : 'FAIL'}`);

// Test 2: Inverse Angle for Target (500, 0) wit v=100
// Should hit 500m. 
const t2 = solver.solveInverseAngle(500, 0, 100);
console.log(`Test 2 (Inverse Angle): Found ${t2.solutions.length} solutions.`);
t2.solutions.forEach((sol, i) => {
    console.log(`  Sol ${i}: Angle=${sol.angle.toFixed(2)}, Range=${sol.range.toFixed(2)} -> ${Math.abs(sol.range - 500) < eps ? 'PASS' : 'FAIL'}`);
});

// Test 3: Vertical Shot (90 deg)
const t3 = solver.solveForward(100, 90);
console.log(`Test 3 (Vertical): Range=${t3.range.toFixed(2)} (Expected ~0) -> ${t3.range < 0.001 ? 'PASS' : 'FAIL'}`);
console.log(`Test 3 (Vertical): Height=${t3.maxHeight.toFixed(2)} (Expected ~509.68) -> ${Math.abs(t3.maxHeight - 509.68) < eps ? 'PASS' : 'FAIL'}`);

// Test 4: Impossible Target
// v=10, target (1000, 1000) - clearly out of reach
const t4 = solver.solveInverseAngle(1000, 1000, 10);
console.log(`Test 4 (Impossible): Solutions=${t4.solutions.length} (Expected 0) -> ${t4.solutions.length === 0 ? 'PASS' : 'FAIL'}`);
