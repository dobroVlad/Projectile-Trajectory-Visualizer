const g = 9.81;
const v0 = 50;
const x = 200;
const y = 50;

console.log(`Checking target: x=${x}, y=${y}, v0=${v0}`);

// 1. Check Envelope of Reachability
// y_max(x) = v^2/(2g) - g*x^2/(2v^2)
const term1 = (v0 * v0) / (2 * g); // v^2 / 2g
const term2 = (g * x * x) / (2 * v0 * v0); // gx^2 / 2v^2

const max_y_at_x = term1 - term2;

console.log(`Max reachable height at x=${x} is y=${max_y_at_x.toFixed(4)} meters.`);
console.log(`Target y is ${y}. Reachable? ${y <= max_y_at_x}`);

// 2. Check Discriminant
const v2 = v0 * v0;
const v4 = v2 * v2;
const discriminant = v4 - g * (g * x * x + 2 * y * v2);
console.log(`Discriminant: ${discriminant}`);
