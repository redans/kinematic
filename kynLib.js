
function motionAinv(n, p) {
   
   const a= [    [n[0][0], n[0][1], n[0][2], n[0][2]*p[0][1] - n[0][1]*p[0][2], n[0][0]*p[0][2] - n[0][2]*p[0][0], n[0][1]*p[0][0] - n[0][0]*p[0][1]],
      [n[1][0], n[1][1], n[1][2], n[1][2]*p[1][1] - n[1][1]*p[1][2], n[1][0]*p[1][2] - n[1][2]*p[1][0], n[1][1]*p[1][0] - n[1][0]*p[1][1]],
      [n[2][0], n[2][1], n[2][2], n[2][2]*p[2][1] - n[2][1]*p[2][2], n[2][0]*p[2][2] - n[2][2]*p[2][0], n[2][1]*p[2][0] - n[2][0]*p[2][1]],
      [n[3][0], n[3][1], n[3][2], n[3][2]*p[3][1] - n[3][1]*p[3][2], n[3][0]*p[3][2] - n[3][2]*p[3][0], n[3][1]*p[3][0] - n[3][0]*p[3][1]],
      [n[4][0], n[4][1], n[4][2], n[4][2]*p[4][1] - n[4][1]*p[4][2], n[4][0]*p[4][2] - n[4][2]*p[4][0], n[4][1]*p[4][0] - n[4][0]*p[4][1]],
      [n[5][0], n[5][1], n[5][2], n[5][2]*p[5][1] - n[5][1]*p[5][2], n[5][0]*p[5][2] - n[5][2]*p[5][0], n[5][1]*p[5][0] - n[5][0]*p[5][1]],
    ];
  return math.inv(a);  
}

function motionB(n,scale) {
    let b = new Array(6); // create an array of length 6
    for (let i = 0; i < 6; i++) {
      b[i] = new Array(6).fill(0); // initialize each element of b to an array of length 6 filled with 0s
    }
  
    for (let ii = 0; ii < 6; ii++) {
      b[ii][ii] = -scale * math.pow(n[ii][0], 2) - scale * math.pow(n[ii][1], 2) - scale * math.pow(n[ii][2], 2);
    }
    
    return b;
}

function Rt(Rx, Ry, Rz) {
    return [
      [1, -Rz, Ry],
      [Rz, 1, -Rx],
      [-Ry, Rx, 1]
    ];
}


function normrP(deform) {
  for (let i = 0; i < deform.length; i++) {
    const slice = deform[i];
    for (let j = 0; j < slice.length; j++) {
      const row = slice[j];
      const norm = math.norm(row, 2); // Compute the L2 norm of the row
      if (norm !== 0) { // Check if the norm is not zero
        slice[j] = row.map(x => x / norm); // Divide each element of the row by its norm
      } else {
        slice[j] = row; // If the norm is zero, leave the row unchanged
      }
    }
  }
  return deform;
}

function calculateDeformation(p,  DOF) {
  var deformation = [];
 //console.log(p)
 //console.log(DOF)
  const pPrime = math.transpose(p);
  
  for (var i = 0; i <= 5; i++) {
    const temp = math.subtract(math.transpose(p), math.add(math.multiply(Rt(DOF[3][i], DOF[4][i], DOF[5][i]), pPrime), math.transpose([[DOF[0][i], DOF[1][i], DOF[2][i]]])));
    deformation.push(math.transpose(temp.valueOf()));
  }
  //console.log(deformation)
  return deformation;
}


function ForceA(p, F) {
 const af = [    [F[0][0], F[1][0], F[2][0], F[3][0], F[4][0], F[5][0]],
    [F[0][1], F[1][1], F[2][1], F[3][1], F[4][1], F[5][1]],
    [F[0][2], F[1][2], F[2][2], F[3][2], F[4][2], F[5][2]],
    [F[0][1]*p[0][2] - F[0][2]*p[0][1], F[1][1]*p[1][2] - F[1][2]*p[1][1], F[2][1]*p[2][2] - F[2][2]*p[2][1], F[3][1]*p[3][2] - F[3][2]*p[3][1], F[4][1]*p[4][2] - F[4][2]*p[4][1], F[5][1]*p[5][2] - F[5][2]*p[5][1]],
    [F[0][2]*p[0][0] - F[0][0]*p[0][2], F[1][2]*p[1][0] - F[1][0]*p[1][2], F[2][2]*p[2][0] - F[2][0]*p[2][2], F[3][2]*p[3][0] - F[3][0]*p[3][2], F[4][2]*p[4][0] - F[4][0]*p[4][2], F[5][2]*p[5][0] - F[5][0]*p[5][2]],
    [F[0][0]*p[0][1] - F[0][1]*p[0][0], F[1][0]*p[1][1] - F[1][1]*p[1][0], F[2][0]*p[2][1] - F[2][1]*p[2][0], F[3][0]*p[3][1] - F[3][1]*p[3][0], F[4][0]*p[4][1] - F[4][1]*p[4][0], F[5][0]*p[5][1] - F[5][1]*p[5][0]]
  ];

  return af;
}

function ForceB(Fc, coordFc) {
  const b = [
    -Fc[0],
    -Fc[1],
    -Fc[2],
    coordFc[1]*Fc[2] - coordFc[2]*Fc[1],
    coordFc[2]*Fc[0] - coordFc[0]*Fc[2],
    coordFc[0]*Fc[2] - coordFc[1]*Fc[0]
  ];

  return b;
}


function kynCalc(n,p,mu,scale, Fc, coordFc) {
  //const { norm: _norm, inv, multiply, transpose, subtract, add } = require('mathjs');

  const Ma = motionAinv(n, p); // calculate motion matrix a

  const Mb=motionB(n,scale) // calculate motion matrix b
  
  const WCSdeform = math.multiply(Ma, Mb); // calculate WCS deformation (motion matrices per retracted interface)

  const PointDeformation=calculateDeformation(p,WCSdeform) // calculate deformation per point

  const normFw = normrP(PointDeformation); // normalized friction vectors

  
  
  const F=normrP(math.add(math.multiply(normFw,mu),n)) // normalized force vectors 
  //console.table('stop')
  // calculate all 6 force matrices a
  const CC = [];
  for (var i = 0; i <= 5; i++) {
  CC.push(ForceA(p, F[i]));
  }

  const BB=ForceB(Fc, coordFc) // calculate force vector b

  // calculate interface forces
  const Finterface = [];
  for (var i = 0; i <= 5; i++) {
  Finterface.push(math.multiply(math.inv(CC[i]), BB));
}

  //console.table(transpose(Finterface))

  return [Finterface, WCSdeform]
}


function populateTable(tableId, data, fontSize, numberFormat, rowNames, columnNames) {
  const table = document.getElementById(tableId);
  table.style.fontSize = fontSize;
  table.innerHTML = ""; // Clear the table

  // Create header row with column names
  const headerRow = document.createElement("tr");
  headerRow.appendChild(document.createElement("td")); // Empty cell for row labels
  for (let i = 0; i < columnNames.length; i++) {
    const cell = document.createElement("td");
    cell.appendChild(document.createTextNode(columnNames[i]));
    headerRow.appendChild(cell);
  }
  table.appendChild(headerRow);

  // Create rows for each case with row label and data values
  for (let i = 0; i < data.length; i++) {
    const row = document.createElement("tr");

    // Add row name as the first cell
    const rowNameCell = document.createElement("td");
    rowNameCell.appendChild(document.createTextNode(rowNames[i]));
    row.appendChild(rowNameCell);

    for (let j = 0; j < data[i].length; j++) {
      const cell = document.createElement("td");
      if (data[i][j] < 0) {
        cell.style.color = "red";
      }

      // Format the number based on the specified format
      if (numberFormat === "exponential") {
        cell.appendChild(document.createTextNode(data[i][j].toExponential(1)));
      } else if (numberFormat === "fixed") {
        cell.appendChild(document.createTextNode(data[i][j].toFixed(2)));
      } else {
        // Default: No specific format, use the number as is
        cell.appendChild(document.createTextNode(data[i][j]));
      }

      row.appendChild(cell);
    }
    table.appendChild(row);
  }

  // Create a style element and append the CSS rules
  const style = document.createElement("style");
  style.innerHTML = `
    #${tableId} th,
    #${tableId} td {
      border: 1px solid black;
      padding: 6px;
      text-align: center;
    }
  `;

  // Append the style element to the head section of the document
  document.head.appendChild(style);
}






function findNegativeForceMuPlus(nn, pp, Fcc, coordFcc) {
  let mu = 0;
  while (true) {
      // Calculate the forces and deformations for the current mu value
      const [FinterfaceF, WCSdeformF] = kynCalc(nn, pp, mu, 0.1, Fcc, coordFcc);

      // Check if any of the FinterfaceF values are negative
      if (FinterfaceF.some((row) => row.some((val) => val < 0))) {
          // Negative force found, return the current mu value
          return mu;
      }

      // Increment the mu value and continue looping
      mu += 0.01;
  }
}

function findNegativeForceMuMin(nn, pp, Fcc, coordFcc) {
  let mu = 0;
  while (true) {
      // Calculate the forces and deformations for the current mu value
      const [FinterfaceF, WCSdeformF] = kynCalc(nn, pp, mu, 0.1, Fcc, coordFcc);

      // Check if any of the FinterfaceF values are negative
      if (FinterfaceF.some((row) => row.some((val) => val < 0))) {
          // Negative force found, return the current mu value
          return mu;
      }

      // Increment the mu value and continue looping
      mu -= 0.01;
  }
}

function findLargestNumbers(array) {
  const largestPositiveNumbers = [];
  const largestNegativeNumbers = [];

  for (let i = 0; i < array.length; i++) {
    let maxPositive = -Infinity;
    let maxNegative = -Infinity;

    for (let j = 0; j < array[i].length; j++) {
      const value = array[i][j];

      if (value > 0 && value > maxPositive) {
        maxPositive = value;
      } else if (value < 0 && value > maxNegative) {
        maxNegative = value;
      }
    }

    largestPositiveNumbers.push(maxPositive);
    largestNegativeNumbers.push(maxNegative);
  }

  return [largestPositiveNumbers, largestNegativeNumbers];
}


function flipLR(matrix) {
  const flippedMatrix = [];
  
  for (let i = 0; i < matrix.length; i++) {
    flippedMatrix.push(matrix[i].reverse());
  }
  
  return flippedMatrix;
}
