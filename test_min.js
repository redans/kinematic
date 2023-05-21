const Optimization = require('optimization-js');

function loss(X) {
  var x = X, y = X;
  return Math.sin(y) * x + Math.sin(x) * y + x * x + y *y;
}
var solution = fmin.nelderMead(loss, [-3.5, 3.5]);
console.log("solution is at " + solution.x);

